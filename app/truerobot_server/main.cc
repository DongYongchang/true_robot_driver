

#include <iostream>
#include <thread>
#include <string>
#include <stdlib.h>

#include "spdlog/spdlog.h"

#include "true_robot_driver.h"

using namespace arm_driver;

void PrintDriverInfo()
{
    TrueArmDriver* driver = TrueArmDriver::GetInstance();

    spdlog::info("Arm has error {}", driver->HasError());

    /* Right joint postion */
    auto right_joint_position = driver->GetRightJointPosition();
    printf("right joint position : ");
    for (int i = 0; i < right_joint_position.size(); i++) {
        printf(" %3lf", right_joint_position[i]);
    }
    printf("\n");

    /* Left joint postion */
    auto left_joint_position = driver->GetLeftJointPosition();
    printf("left joint position : ");
    for (int i = 0; i < left_joint_position.size(); i++) {
        printf(" %3lf", left_joint_position[i]);
    }
    printf("\n");

    /* Right joint torque */
    auto right_joint_torque = driver->GetRightJointTorque();
    printf("right joint torque : ");
    for (int i = 0; i < right_joint_torque.size(); i++) {
        printf(" %3lf", right_joint_torque[i]);
    }
    printf("\n");

    /* Left joint postion */
    auto left_joint_torque = driver->GetLeftJointTroque();
    printf("left joint torque : ");
    for (int i = 0; i < left_joint_torque.size(); i++) {
        printf(" %3lf", left_joint_torque[i]);
    }
    printf("\n");

    /* Right tool force */
    auto right_tool_force = driver->GetRightToolForce();
    printf("right tool force : ");
    for (int i = 0; i < right_tool_force.size(); i++) {
        printf(" %3lf", right_tool_force[i]);
    }
    printf("\n");

    /* Left tool force */
    auto left_tool_force = driver->GetLeftToolForce();
    printf("left tool force : ");
    for (int i = 0; i < left_tool_force.size(); i++) {
        printf(" %3lf", left_tool_force[i]);
    }
    printf("\n");
}

void SetArmPositinTest(int i, double joint_position, bool inv = true)
{
    {
        TrueArmDriver* driver = TrueArmDriver::GetInstance();
        std::vector pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        pos[i] = joint_position;
        driver->SetRightJointPosition(pos);
        driver->SetLeftJointPosition(pos);
        driver->MainLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    {
        TrueArmDriver* driver = TrueArmDriver::GetInstance();
        std::vector pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        driver->SetRightJointPosition(pos);
        driver->SetLeftJointPosition(pos);
        driver->MainLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    if(inv != true)
    {
        return;
    }

    {

        TrueArmDriver* driver = TrueArmDriver::GetInstance();
        std::vector pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        pos[i] = -joint_position;
        driver->SetRightJointPosition(pos);
        driver->SetLeftJointPosition(pos);
        driver->MainLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    {

        TrueArmDriver* driver = TrueArmDriver::GetInstance();
        std::vector pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        driver->SetRightJointPosition(pos);
        driver->SetLeftJointPosition(pos);
        driver->MainLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}

int main()
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::info("Hello, World!");

    // Use the singleton instance:
    TrueArmDriver* driver = TrueArmDriver::GetInstance();

    driver->Init("");

    PrintDriverInfo();

    driver->SendCommand(Command::Init);

    while (1) {
        driver->MainLoop();

        if (driver->GetState() == State::Init) {

            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    PrintDriverInfo();

    driver->SendCommand(Command::Start);

    while (1) {
        driver->MainLoop();

        if (driver->GetState() == State::Run) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    PrintDriverInfo();

    int test_cnt = 0;
    while (1) {
        driver->MainLoop();
#if 0
        static bool flag = true;
        if (flag == true) {
             std::vector pos = {1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
             driver->SetRightJointPosition(pos);
             driver->SetLeftJointPosition(pos);
            flag = false;
        } else {
             std::vector pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
             driver->SetRightJointPosition(pos);
             driver->SetLeftJointPosition(pos);
            flag = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
#endif

#if 1

        SetArmPositinTest(0, 1.5);


        SetArmPositinTest(1, 0.5);

          SetArmPositinTest(2, 1.5);
          SetArmPositinTest(3, 1.5, false);
          SetArmPositinTest(4, 1.5);
          SetArmPositinTest(5, -1.5, false);
          SetArmPositinTest(6, 1.5);
#endif
        driver->MainLoop();

        PrintDriverInfo();
    }

    driver->SendCommand(Command::Stop);

    driver->DeInit();

    return 0;
}