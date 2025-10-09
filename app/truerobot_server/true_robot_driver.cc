#include "true_robot_driver.h"
#include <spdlog/spdlog.h>

#include "state/message/TDMessageBus.h"
#include "state/overall_system_nrtstate.pb.h"
#include "state/overall_system_rtstate.pb.h"
#include "sub.hpp"
#include "google/protobuf/timestamp.pb.h"

#include "cpp_rpc.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "cmd/message/rpc_client.h"

namespace arm_driver {

static TrueArmDriver* instance_ = nullptr;

static void OnRTState(const system_rtstate::SystemRtState& msg)
{
    TrueArmDriver* arm_driver = instance_;

    if (arm_driver == nullptr) {
        return;
    }

    if (arm_driver->HasError()) {
        return;
    }

    SystemStateData data;
    display_rt(msg, data);

    if (data.models_joints.size()
        != arm_driver->GetRightJointPosition().size()
               + arm_driver->GetRightJointPosition().size()) {
        spdlog::warn("joint position size not match, {} != {}", data.models_joints.size(),
            arm_driver->GetRightJointPosition().size() + arm_driver->GetLeftJointPosition().size());

        arm_driver->SetError();
        return;
    }

    std::vector<double> joint_pos;
    for (int i = 0; i < arm_driver->GetRightJointPosition().size(); i++) {
        arm_driver->GetRightJointPosition()[i] = data.models_joints[i].position;
        arm_driver->GetRightJointTorque()[i] = data.models_joints[i].torque;
        arm_driver->GetJointError()[i] = data.models_joints[i].error_code;
    }

    for (int i = 0; i < arm_driver->GetLeftJointPosition().size(); i++) {
        if(i == 0 || i == 2 || i == 4 || i == 6)
        {
            arm_driver->GetLeftJointPosition()[i] = -data.models_joints[i + 7].position;
            arm_driver->GetLeftJointPosition()[i] = -data.models_joints[i + 7].torque;
        }
        else 
        {
            arm_driver->GetLeftJointPosition()[i] = data.models_joints[i + 7].position;
            arm_driver->GetLeftJointPosition()[i] = data.models_joints[i + 7].torque;
        }

        arm_driver->GetJointError()[i] = data.models_joints[i + 7].error_code;
    }

    for (int i = 0; i < 6; i++) {
        arm_driver->GetRightToolForce()[i] = data.controller.ftvalues[0][i];
        arm_driver->GetLeftToolForce()[i] = data.controller.ftvalues[1][i];
    }

}

static void OnNoRTState(const system_nrtstate::SystemNrtState& msg)
{
    TrueArmDriver* arm_driver = instance_;

    if (arm_driver == nullptr) {
        return;
    }

    if (arm_driver->HasError()) {
        return;
    }
}

// Singleton instance getter - using static local variable (Meyer's Singleton)
TrueArmDriver* TrueArmDriver::GetInstance()
{
    if (instance_ == nullptr) {
        instance_ = new TrueArmDriver();
    }
    return instance_;
}

TrueArmDriver::TrueArmDriver()
{
    NodeOptions system_rtstate_options;
    system_rtstate_options.node_name = "true_robot_servert";
    system_rtstate_options.sub_url = "tcp://192.168.11.11:19091";
    node_ = std::make_shared<message_bus::Node>(system_rtstate_options);
    cmd_client_ = std::make_shared<cpp_rpc::CPPClient>("192.168.11.11", 5868);
}

TrueArmDriver::~TrueArmDriver()
{
    free(instance_);
    instance_ = nullptr;
}

bool TrueArmDriver::Init(std::string config_file)
{
    joint_num_ = 7;
    right_joint_pos_.resize(joint_num_, 0.0);
    left_joint_pos_.resize(joint_num_, 0.0);

    right_joint_vel_.resize(joint_num_, 0.0);
    left_joint_vel_.resize(joint_num_, 0.0);

    target_right_joint_pos_.resize(joint_num_, 0.0);
    target_left_joint_pos_.resize(joint_num_, 0.0);

    joint_error_.resize(joint_num_, 0);

    max_joint_vel_ = 2.0;
    max_joitn_acc_ = 4;
    right_joint_vel_gain_ = 0.5;
    right_joint_acc_gain_ = 0.5;

    left_joint_vel_gain_ = 0.5;
    left_joint_acc_gain_ = 0.5;

    right_joint_torque_.resize(6, 0.0);
    left_joint_torque_.resize(6, 0.0);

    right_tool_force_.resize(6, 0.0);
    left_tool_force_.resize(6, 0.0);

    return true;
}

void TrueArmDriver::DeInit() { }

State TrueArmDriver::GetState()
{
    return state_;
}

void TrueArmDriver::SendCommand(Command command)
{
    command_ = command;
}

void TrueArmDriver::MainLoop()
{
    switch (state_) {
        case State::None: {
            /* Do somenthing */
            node_->Start();
            rt_state_subscription_ = node_->CreateSubscription<system_rtstate::SystemRtState>(
                "system_rtstate", OnRTState);

            nrt_state_subscription_ = node_->CreateSubscription<system_nrtstate::SystemNrtState>(
                "system_nrtstate", OnNoRTState);

            /* Auto transmit */
            state_ = State::Init;
            break;
        }
        case State::Init: {
            if (HasError()) {
                state_ = State::Error;
                break;
            }

            /* Do something */
            if (command_ == Command::Init || command_ == Command::Start) {
                /* Do something */
                for (int i = 0; i < joint_num_; i++) {
                    target_left_joint_pos_[i] = left_joint_pos_[i];
                    target_right_joint_pos_[i] = right_joint_pos_[i];
                }

                state_ = State::Stop;
            }

            break;
        }
        case State::Stop: {

            if (HasError()) {
                state_ = State::Error;
                break;
            }

            if (command_ == Command::Start) {

                // clang-format off
                std::vector<std::string> init_cmds = {
                    "{Clear}",
                    "{Claer}",
                    "{Disable}",
                    "{Mode}",
                    "{SetMaxToq}",
                    "{Recover}",
                    "{SetRate}",
                    "{Enable}",
                    "{Var --clear}",
                    "{Recover}",
                };
                // clang-format on
                SendRpcsy(init_cmds);

                /* Do something */
                state_ = State::Run;
            }
            break;
        }
        case State::Run: {
            if (HasError()) {
                state_ = State::Error;
                break;
            }

            SendJointPositionCmd();

            if (command_ == Command::Stop) {
                /* Do something */
                // clang-format off
                std::vector<std::string> stop_cmds = {
                    "{Stop}",
                    "{Disable}",
                };
                // clang-format on
                SendRpcsy(stop_cmds);
                state_ = State::Stop;
            }
            break;
        }
        case State::Error: {
            if (command_ == Command::Reset) {
                /* Do something */
            }
            break;
        }
        case State::Reset: {
            break;
        }
        default:
            break;
    }

    if (state_ != last_state_) {
        spdlog::info("State change: {} ---->>>> {}", static_cast<int>(last_state_),
            static_cast<int>(state_));
        last_state_ = state_;
    }

    for(int i =0; i < joint_error_.size() ;i++)
    {
        if(joint_error_[i] != 0)
        {
            SetError();
            break;
        }
    }
}

bool TrueArmDriver::HasError()
{
    return has_master_error_;
}

std::string TrueArmDriver::ErrorString()
{
    return std::string("");
}

std::vector<double>& TrueArmDriver::GetRightJointPosition()
{
    return right_joint_pos_;
}

bool TrueArmDriver::SetRightJointPosition(
    std::vector<double>& joint_position, float vel_gain, float acc_gain)
{
    if (vel_gain > 1.0f || acc_gain > 1.0f) {
        return false;
    }

    if (target_right_joint_pos_.size() != joint_position.size()) {
        spdlog::warn("Taget left joint size({}) and joint position size({}) mismatch",
            target_right_joint_pos_.size(), joint_position.size());
        return false;
    }

    target_right_joint_pos_ = joint_position;
    right_joint_vel_gain_ = vel_gain;
    right_joint_acc_gain_ = acc_gain;

    return false;
}

std::vector<double>& TrueArmDriver::GetLeftJointPosition()
{
    return left_joint_pos_;
}

bool TrueArmDriver::SetLeftJointPosition(
    std::vector<double>& joint_position, float vel_gain, float acc_gain)
{
    if (vel_gain > 1.0f || acc_gain > 1.0f) {
        return false;
    }


    if (target_left_joint_pos_.size() != joint_position.size()) {
        spdlog::warn("Taget left joint size({}) and joint position size({}) mismatch",
            target_left_joint_pos_.size(), joint_position.size());
        return false;
    }

    target_left_joint_pos_ = joint_position;

    target_left_joint_pos_[0] = -target_left_joint_pos_[0];
    target_left_joint_pos_[2] = -target_left_joint_pos_[2];
    target_left_joint_pos_[4] = -target_left_joint_pos_[4];
    target_left_joint_pos_[6] = -target_left_joint_pos_[6];

    left_joint_vel_gain_ = vel_gain;
    left_joint_acc_gain_ = acc_gain;

    return false;
}

std::string DdoubleVecToString(std::vector<double> value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6); // Set precision for double values

    for (size_t i = 0; i < value.size(); ++i) {
        if (i > 0) {
            oss << ",";
        }
        oss << value[i];
    }

    return oss.str();
}

void TrueArmDriver::SendJointPositionCmd()
{
    std::vector<std::string> cmd_vec;

    std::string right_joint_pos_str = DdoubleVecToString(target_right_joint_pos_);
    std::string left_joint_pos_str = DdoubleVecToString(target_left_joint_pos_);

    std::string right_joint_max_vel_str = std::to_string(max_joint_vel_ * right_joint_vel_gain_);
    std::string right_max_acc_str = std::to_string(max_joitn_acc_ * right_joint_acc_gain_);

    std::string left_joint_max_vel_str = std::to_string(max_joint_vel_ * left_joint_vel_gain_);
    std::string left_max_acc_str = std::to_string(max_joitn_acc_ * left_joint_acc_gain_);

    std::string arm_pos_cmd = "{JogAnyJ2 --joint_pos={" + right_joint_pos_str + "} --joint_vel={"
                              + right_joint_max_vel_str + "} --joint_acc={" + right_max_acc_str
                              + "}||JogAnyJ2 --joint_pos={" + left_joint_pos_str + "} --joint_vel={"
                              + left_joint_max_vel_str + "} --joint_acc={" + left_max_acc_str
                              + "}}";

    // This command is not suitable
#if 0
    // std::string delete_var
    //     = "{DeleteVar --name=target_right_joint_pos || DeleteVar --name=target_left_joint_pos}";
    // cmd_vec.push_back("{DeleteVar --name=target_right_joint_pos}");
    // cmd_vec.push_back("{DeleteVar --name=target_left_joint_pos}");

    std::string right_joint_var_cmd
        = "{ReVarValue --type=jointtarget --name=target_right_joint_pos --value={" + right_joint_pos_str
          + "}}";

    std::string left_joint_var_cmd
        = "{ReVarValue --type=jointtarget --name=target_left_joint_pos --value={" + left_joint_pos_str
          + "}}";



    std::string cmd
        = "{MoveAbsJ --jointtarget_var=target_right_joint_pos||MoveAbsJ "
          "--jointtarget_var=target_left_joint_pos}";

    spdlog::debug("test point cmd {}", cmd);

    cmd_vec.push_back(right_joint_var_cmd);
    cmd_vec.push_back(left_joint_var_cmd);
    cmd_vec.push_back(cmd);

    SendRpcsy(cmd_vec);
#endif
    spdlog::debug("test point cmd {}", arm_pos_cmd);

    cmd_vec.push_back(arm_pos_cmd);

    SendRpcsy(cmd_vec);
}

void TrueArmDriver::SendRpcsy(const std::vector<std::string>& cmd_cmd, int outim_num, int sleep_num)
{
    for (const auto& cmd : cmd_cmd) {

        char* fake_argv[] = {const_cast<char*>("dummy"), const_cast<char*>(cmd.c_str())};

        srand(time(0));
        int i = rand();

        std::string msg(fake_argv[1]);

        core::Msg sync_msg(msg);
        sync_msg.setMsgID(10001);
        sync_msg.setMsgSeqID(i);

        auto res = cmd_client_->CallAwait<RespDemo>(sync_msg, outim_num);
        if (0 == res.first) {
            // std::cout << "*************Sync***************" << std::endl;
            // std::cout << "model size:" << res.second.size() << std::endl;
            for (const auto& r : res.second) {
                // std::cout << "subcmd_index:" << r.subcmd_index << std::endl;
                // std::cout << "return_code:" << r.return_code << std::endl;
                // std::cout << "return_message:" << r.return_message << std::endl;
            }
            // std::cout << "****************************" << std::endl;
        } else {
            // std::cout << "同步请求失败!确保out time大于指令生效时间!错误码: " << res.first
            //          << std::endl;
            // 自动清理错误
            // client.ClearErr();
            spdlog::error("Request failed, {}", res.first);
        }

        // std::cout << "over!!!" << std::endl;
        // std::cout << "****************************" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_num));
    }
}

void TrueArmDriver::SendRpcAsy(const std::vector<std::string>& cmd_cmd, int outim_num, int wait_num)
{

    for (const auto& cmd : cmd_cmd) {

        char* fake_argv[] = {const_cast<char*>("dummy"), const_cast<char*>(cmd.c_str())};

        // 获取随机数
        srand(time(0));
        int i = rand();

        std::string msg(fake_argv[1]);

        core::Msg message(msg);
        cmd_client_->CallAsync<RespDemo>(
            message, outim_num, [&](int ret, std::vector<RespDemo> res) {
                // std::cout << "**************Async**************" << std::endl;
                if (ret < 0) {
                    spdlog::error("Async request failed. ret: {}", ret);
                }

                // std::cout << "model size:" << res.size() << std::endl;
                for (const auto& r : res) {
                    //    std::cout << "subcmd_index:" << r.subcmd_index << std::endl;
                    //    std::cout << "return_code:" << r.return_code << std::endl;
                    //    std::cout << "return_message:" << r.return_message << std::endl;
                }
                // std::cout << "*********************************" << std::endl;
            });

        core::Msg sync_msg(msg);
        sync_msg.setMsgID(10001);
        sync_msg.setMsgSeqID(i);
    }
}

std::vector<double>& TrueArmDriver::GetRightJointTorque()
{
    return right_joint_torque_;
}

std::vector<double>& TrueArmDriver::GetLeftJointTroque()
{
    return left_joint_torque_;
}

std::vector<double>& TrueArmDriver::GetRightToolForce()
{
    return right_tool_force_;
}

std::vector<double>& TrueArmDriver::GetLeftToolForce()
{
    return left_tool_force_;
}

}
