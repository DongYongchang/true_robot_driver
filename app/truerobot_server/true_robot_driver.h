#pragma once

#include <string>
#include <memory>
#include <vector>


#include "arm_driver_def.h"


namespace message_bus {
class Node;
class Subscription;
}

namespace cpp_rpc
{
    class CPPClient;
}

namespace arm_driver {

class TrueArmDriver
{
public:
    // Static method to get the singleton instance
    static TrueArmDriver* GetInstance();

    /* */
    std::vector<double>& GetRightJointPosition();

    std::vector<double>& GetLeftJointPosition();

    std::vector<double>& GetRightJointTorque();

    std::vector<double>& GetLeftJointTroque();

    std::vector<double>& GetRightToolForce();

    std::vector<double>& GetLeftToolForce();

    bool SetRightJointPosition(std::vector<double>& joint_position, float vel_gain = 0.5, float acc_gain = 0.5);

    bool SetLeftJointPosition(std::vector<double>& joint_position, float vel_gain = 0.5, float acc_gain = 0.5);

    /* Base state machine */
    bool Init(std::string config_file = "");

    void DeInit();

    State GetState();

    void SendCommand(Command command);

    void MainLoop();

    void SetError() {has_master_error_ = true;}
    bool HasError();

    std::string ErrorString();

    std::vector<int>& GetJointError() {return joint_error_;};
    TrueArmDriver();
    ~TrueArmDriver();

    // Private constructor and destructor
    bool SendArmCommand(std::string cmd, bool sync = false);

    void SendRpcsy(const std::vector<std::string>& cmd_cmd, int outim_num = 500, int sleep_num = 100);

    void SendRpcAsy(const std::vector<std::string>& cmd_cmd, int outim_num = 50000, int wait_num = 100);

    void PrintArmInfo() const;

    void SendJointPositionCmd();

    State state_ = State::None;
    State last_state_ = State::None;
    Command command_ = Command::None;

    bool has_master_error_ = false;

    std::shared_ptr<message_bus::Node> node_;
    std::shared_ptr<cpp_rpc::CPPClient> cmd_client_;
    std::shared_ptr<message_bus::Subscription> nrt_state_subscription_;
    std::shared_ptr<message_bus::Subscription> rt_state_subscription_;

    uint8_t joint_num_ = 0;
    std::vector<double> right_joint_pos_;
    std::vector<double> left_joint_pos_;

    std::vector<double> right_joint_vel_;
    std::vector<double> left_joint_vel_;

    std::vector<double> right_joint_torque_;
    std::vector<double> left_joint_torque_;

    std::vector<double> target_right_joint_pos_;
    std::vector<double> target_left_joint_pos_;

    double max_joint_vel_ = 0.0;
    double max_joitn_acc_ = 0.0;
    float right_joint_vel_gain_ = 0.0;
    float right_joint_acc_gain_ = 0.0;

    float left_joint_vel_gain_ = 0.0;
    float left_joint_acc_gain_ = 0.0;

    std::vector<int> joint_error_;

    std::vector<double> right_tool_force_;
    std::vector<double> left_tool_force_;
};

}
