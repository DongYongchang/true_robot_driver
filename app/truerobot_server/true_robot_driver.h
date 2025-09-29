#pragma once

#include <string>
#include <memory>
#include <vector>


#include "arm_driver_def.h"

namespace message_bus {
class Node;
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

    void SetRightJointPosition(std::vector<double>& joint_position);

    std::vector<double>& GetLeftJointPosition();

    void SetLeftJointPosition(std::vector<double>& joint_position);


    /* Base state machine */
    bool Init(std::string config_file = "");

    void DeInit();

    State GetState();

    void SendCommand(Command command);

    void MainLoop();

    bool HasError();

    std::string ErrorString();

    std::vector<int>& GetJointError() {return joint_error_;};

private:
    // Private constructor and destructor
    TrueArmDriver();
    ~TrueArmDriver();

    bool SendArmCommand(std::string cmd, bool sync = false);


    void PrintArmInfo() const;


    void SendJointPositionCmd();

    State state_ = State::None;
    State last_state_ = State::None;
    Command command_ = Command::None;

    bool has_master_error_ = false;

    std::shared_ptr<message_bus::Node> node_;
    std::unique_ptr<cpp_rpc::CPPClient> cmd_client_;
 

    uint8_t joint_num_ = 0;
    std::vector<double> right_joint_pos_;
    std::vector<double> left_joint_pos_;

    std::vector<double> target_right_joint_pos_;
    std::vector<double> target_left_joint_pos_;

    std::vector<int> joint_error_;
};

}