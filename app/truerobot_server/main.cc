

#include <iostream>
#include <thread>
#include <string>
#include <stdlib.h>

#include "spdlog/spdlog.h"

#include "true_robot_driver.h"

using namespace arm_driver;

int main()
{

    spdlog::info("Hello, World!");

    // Instead of creating instances:
    // TrueArmDriver driver;  // Old way - no longer works

    // Use the singleton instance:
    TrueArmDriver* driver = TrueArmDriver::GetInstance();

    // All other usage remains the same:
    // driver.Init();
    // driver.SendCommand(Command::Start);
    // driver.MainLoop();
    // State current_state = driver.GetState();

    driver->Init("");

    driver->SendCommand(arm_driver::Command::Start);

    while (1) {
        driver->MainLoop();

        State current_state = driver->GetState();
        spdlog::info("Current state: {}", static_cast<int>(current_state));

        auto position = driver->GetRightJointPosition();
        printf("joint position : ");
        for (int i = 0; i < position.size(); i++) {
            //  spdlog::info("Joint {} position: {}", i, position[i]);
            printf(" %lf\n", i, position[i]);
        }
        printf("\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    driver->DeInit();

    return 0;
}