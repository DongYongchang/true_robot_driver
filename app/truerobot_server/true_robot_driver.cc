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
#include"cmd/message/rpc_client.h"

namespace arm_driver {

static TrueArmDriver* instance_ = nullptr;

static void OnState(const system_rtstate::SystemRtState& msg)
{
    TrueArmDriver* arm_driver = instance_;

    if (arm_driver == nullptr) {
        return;
    }

    // state.joint_position = msg.joint_position();
    SystemStateData data;
    display_rt(msg, data);

    if (data.models_joints.size() != arm_driver->GetRightJointPosition().size()) {
        spdlog::warn("joint position size not match, {} != {}", data.models_joints.size(),
            arm_driver->GetRightJointPosition().size());
        return;
    }

    std::vector<double> joint_pos;
    for (int i = 0; i < data.models_joints.size(); i++) {
        arm_driver->GetRightJointPosition()[i] = data.models_joints[i].position;
        arm_driver->GetJointError()[i] = data.models_joints[i].error_code;
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
    system_rtstate_options.node_name = "test1";
    system_rtstate_options.sub_url = "tcp://192.168.11.11:19091";
    node_ = std::make_shared<message_bus::Node>(system_rtstate_options);

    cmd_client_ = std::make_unique<cpp_rpc::CPPClient>("192.168.11.11", 5868);
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
    joint_error_.resize(joint_num_, 0);

    node_->CreateSubscription<system_rtstate::SystemRtState>("OverallSystemRtState", OnState);

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
            /* Auto transmit */
            state_ = State::Init;
            break;
        }
        case State::Init: {
            /* Do something */
            if (command_ == Command::Init || command_ == Command::Start) {
                /* Do something */
                node_->Start();
                state_ = State::Stop;
            }
            break;
        }
        case State::Stop: {
            if (command_ == Command::Start) {

                // clang-format off
                std::vector<std::string> init_cmds = {
                    "{Clear}",
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

                for (auto& cmd : init_cmds) {
                    SendArmCommand(cmd, true);
                }

                std::string right_joint_var_cmd = "{Var --type=jointtarget --name=target_right_joint_pos --value={0,0,0,0,0,0,0,0,0,0}}";
                SendArmCommand(right_joint_var_cmd, true);

                std::string left_joint_var_cmd = "{Var --type=jointtarget --name=target_left_joint_pos --value={0,0,0,0,0,0,0,0,0,0}}";
                SendArmCommand(left_joint_var_cmd, true);

                /* Do something */
                state_ = State::Run;
            }
            break;
        }
        case State::Run: {
            if (command_ == Command::Stop) {
                // node_->Cancel();
                /* Do something */
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

void TrueArmDriver::SetRightJointPosition(std::vector<double>& joint_position)
{
    target_right_joint_pos_ = joint_position;
}

std::vector<double>& TrueArmDriver::GetLeftJointPosition()
{
    return left_joint_pos_;
}

void TrueArmDriver::SetLeftJointPosition(std::vector<double>& joint_position)
{
    target_left_joint_pos_ = joint_position;
}

bool TrueArmDriver::SendArmCommand(std::string cmd, bool sync)
{
    if (sync) {
        char* fake_argv[] = {const_cast<char*>("dummy"), const_cast<char*>(cmd.c_str())};

        // 获取随机数
        srand(time(0));
        int i = rand();

        std::string msg(fake_argv[1]);

        core::Msg sync_msg(msg);
        sync_msg.setMsgID(10001);
        sync_msg.setMsgSeqID(i);

        auto res = cmd_client_->CallAwait<RespDemo>(sync_msg, 500);
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
            spdlog::warn("同步请求失败!确保out time大于指令生效时间!错误码: {}", res.first);
            // std::cout << "同步请求失败!确保out time大于指令生效时间!错误码: " << res.first
            //           << std::endl;
            // 自动清理错误
            // client.ClearErr();
        }

        // std::cout << "over!!!" << std::endl;
        // std::cout << "****************************" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
        char* fake_argv[] = {const_cast<char*>("dummy"), const_cast<char*>(cmd.c_str())};

        // 获取随机数
        srand(time(0));
        int i = rand();

        std::string msg(fake_argv[1]);

        core::Msg message(msg);
        cmd_client_->CallAsync<RespDemo>(message, 1000, [&](int ret, std::vector<RespDemo> res) {
            // std::cout << "**************Async**************" << std::endl;
            if (ret < 0) {
                spdlog::warn("异步请求失败. ret:{} out time !", ret);
                // std::cout << "Async request failed. ret:" << ret << " " << "out time !"
                //           << std::endl;
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

        // std::this_thread::sleep_for(std::chrono::seconds(wait_num)); // 等待异步操作完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
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
    std::string right_joint_pos_str = DdoubleVecToString(target_right_joint_pos_);
    std::string left_joint_pos_str = DdoubleVecToString(target_left_joint_pos_);

    std::string right_joint_var_cmd = "{Var --type=jointtarget --name=target_right_joint_pos --value={" + right_joint_pos_str + "}}";
    SendArmCommand(right_joint_var_cmd);
    std::string left_joint_var_cmd = "{Var --type=jointtarget --name=target_left_joint_pos --value={" + left_joint_pos_str + "}}";
    SendArmCommand(left_joint_var_cmd);
    std::string cmd = "{MoveAbsJ --jointtarget_var=target_right_joint_pos|| MoveAbsJ --jointtarget_var=target_left_joint_pos}";
    SendArmCommand(cmd);
}

}