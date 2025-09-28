

#include <iostream>
#include <thread>
#include <string>

#include "message/TDMessageBus.h"
#include "overall_system_nrtstate.pb.h"
#include "overall_system_rtstate.pb.h"
#include "sub.hpp"
#include "google/protobuf/timestamp.pb.h"


using namespace std;
typedef std::size_t Size;
int main() {
  using namespace message_bus;
  using namespace message_bus::detail;

  // 配置&创建节点  订阅远程地址 192.168.2.10:19091
  // 的"system_rtstate"及"system_nrtstate"主题 回调函数接收
  // system_rtstate::SystemState 消息
  NodeOptions system_rtstate_options;
  system_rtstate_options.node_name = "test1";
  system_rtstate_options.sub_url = "tcp://192.168.11.11:19091";
  Node system_rtstate_node(system_rtstate_options);
  system_rtstate_node.Start();
  auto subrt = system_rtstate_node.CreateSubscription<
      system_rtstate::SystemRtState>(
      "system_rtstate", [](const system_rtstate::SystemRtState &tt) {
        SystemStateData rtparm;
        display_rt(tt, rtparm);

        /****************************************************************************/
        std::cout << "1.rtheader: " << rtparm.header_timestamp << " "
                  << rtparm.header_frame_id << std::endl;
        std::cout << "2.system_running_state: " << rtparm.system_running_state
                  << std::endl;

        std::cout << "3.Controller Info:" << std::endl;
        std::cout << "  3.1.controller_name: "
                  << rtparm.controller.controller_name << std::endl;
        std::cout << "  3.2.control_cycle: " << rtparm.controller.control_cycle
                  << std::endl;
        std::cout << "  3.3.global_count: " << rtparm.controller.global_count
                  << std::endl;
        std::cout << "  3.4.master_info: " << rtparm.controller.master_info
                  << std::endl;
        std::cout << "  3.5.is_link_up: " << rtparm.controller.is_link_up
                  << std::endl;

        // 显示六维力传感器数据
        std::cout << "  3.6.FT Sensor Values:" << std::endl;
        for (size_t i = 0; i < rtparm.controller.ftvalues.size(); ++i) {
          std::cout << "    3.6." << i + 1 << ". FT Sensor " << i << ": ";
          std::cout << "fx=" << rtparm.controller.ftvalues[i][0] << ", ";
          std::cout << "fy=" << rtparm.controller.ftvalues[i][1] << ", ";
          std::cout << "fz=" << rtparm.controller.ftvalues[i][2] << ", ";
          std::cout << "mx=" << rtparm.controller.ftvalues[i][3] << ", ";
          std::cout << "my=" << rtparm.controller.ftvalues[i][4] << ", ";
          std::cout << "mz=" << rtparm.controller.ftvalues[i][5] << std::endl;
        }

        std::cout << "4.Models Info:" << std::endl;
        for (size_t i = 0; i < rtparm.models.size(); ++i) {
          std::cout << "  4." << i + 1 << ".  Model " << i << ":" << std::endl;
          std::cout << "    4a.model_name: " << rtparm.models[i].model_name
                    << std::endl;
          std::cout << "    4b.model_type: " << rtparm.models[i].model_type
                    << std::endl;

          // 计算当前模型的关节起始索引
          //  此处每个模型的关节数量相同，可根据实际情况进行逻辑设计
          size_t joints_per_model =
              rtparm.models_joints.size() / rtparm.models.size();
          size_t joint_start_index = i * joints_per_model;

          std::cout << "  5.Joints Info for Model " << i << ":" << std::endl;
          for (size_t j = 0;
               j < joints_per_model &&
               (joint_start_index + j) < rtparm.models_joints.size();
               ++j) {
            size_t joint_index = joint_start_index + j;
            std::cout << "    5." << j + 1 << ".  Joint " << j << ":"
                      << std::endl;
            std::cout << "      5a.joint_type: "
                      << rtparm.models_joints[joint_index].joint_type
                      << std::endl;
            std::cout << "      5b.position: "
                      << rtparm.models_joints[joint_index].position
                      << std::endl;
            std::cout << "      5c.torque: "
                      << rtparm.models_joints[joint_index].torque << std::endl;
            std::cout << "      5d.is_enabled: "
                      << rtparm.models_joints[joint_index].is_enabled
                      << std::endl;
            std::cout << "      5e.mode: "
                      << rtparm.models_joints[joint_index].mode << std::endl;
            std::cout << "      5f.error_code: "
                      << rtparm.models_joints[joint_index].error_code
                      << std::endl;
            std::cout << "      5g.digit_output: "
                      << rtparm.models_joints[joint_index].digit_output
                      << std::endl;
            std::cout << "      5h.digit_input: "
                      << rtparm.models_joints[joint_index].digit_input
                      << std::endl;
          }

          // 检查是否有对应的当前点信息
          if (i < rtparm.models_current_points.size()) {
            std::cout << "  6.Current Point Info for Model " << i << ":"
                      << std::endl;
            std::cout << "    6.1.point_name: "
                      << rtparm.models_current_points[i].point_name
                      << std::endl;
            std::cout << "    6.2.tool_name: "
                      << rtparm.models_current_points[i].tool_name << std::endl;
            std::cout << "    6.3.wobj_name: "
                      << rtparm.models_current_points[i].wobj_name << std::endl;

            std::cout << "    6.4.Tool Data: ";
            for (size_t k = 0;
                 k < rtparm.models_current_points[i].tool_data.size(); ++k) {
              std::cout << rtparm.models_current_points[i].tool_data[k] << " ";
            }
            std::cout << std::endl;

            std::cout << "    6.5.Wobj Data: ";
            for (size_t k = 0;
                 k < rtparm.models_current_points[i].wobj_data.size(); ++k) {
              std::cout << rtparm.models_current_points[i].wobj_data[k] << " ";
            }
            std::cout << std::endl;

            std::cout << "    6.6.Robot Target: ";
            for (size_t k = 0;
                 k < rtparm.models_current_points[i].robottarget.size(); ++k) {
              std::cout << rtparm.models_current_points[i].robottarget[k]
                        << " ";
            }
            std::cout << std::endl;

            std::cout << "    6.7.Joint Target: ";
            for (size_t k = 0;
                 k < rtparm.models_current_points[i].jointtarget.size(); ++k) {
              std::cout << rtparm.models_current_points[i].jointtarget[k]
                        << " ";
            }
            std::cout << std::endl;
          }

          std::cout << "  7.Model Error Info for Model " << i << ":"
                    << std::endl;
          if (i < rtparm.models_info.size()) {
            std::cout << "    7.1.error_code: "
                      << rtparm.models_info[i].error_code << std::endl;
            std::cout << "    7.2.error_msg: "
                      << rtparm.models_info[i].error_msg << std::endl;
            std::cout << "    7.3.model_state: "
                      << rtparm.models_info[i].model_state << std::endl;
            std::cout << "    7.4.model_time_rate: "
                      << rtparm.models_info[i].model_time_rate << std::endl;
            std::cout << "    7.5.current_func_name: "
                      << rtparm.models_info[i].current_func_name << std::endl;
            std::cout << "    7.6.current_func_info: "
                      << rtparm.models_info[i].current_func_info << std::endl;
            std::cout << "    7.7.func_count: "
                      << rtparm.models_info[i].func_count << std::endl;
          } else {
            std::cout << "    7.1.No model info available for model " << i
                      << std::endl;
          }
        }

        std::cout << std::endl;
        /****************************************************************************/
      });

#if 0
  NodeOptions system_nrtstate_options;
  system_nrtstate_options.node_name = "test2";
  system_nrtstate_options.sub_url = "tcp://192.168.11.11:19091";
  Node system_nrtstate_node(system_nrtstate_options);
  system_nrtstate_node.Start();
  auto subnrt =
      system_nrtstate_node.CreateSubscription<system_nrtstate::SystemNrtState>(
          "system_nrtstate", [](const system_nrtstate::SystemNrtState &tt) {
            SystemStateData nrtparm;
            display_nrt(tt, nrtparm);

            /****************************************************************/
            // 打印 header 信息
            std::cout << "N 1.nrtheader: timestamp=" << nrtparm.header_timestamp
                      << ", frame_id=" << nrtparm.header_frame_id << std::endl;

            // 打印 system_running_state
            std::cout << "N 2.system_running_state: "
                      << nrtparm.system_running_state << std::endl;

            // 打印 slaves 信息
            std::cout << "N 3.slaves: " << std::endl;
            for (size_t i = 0; i < nrtparm.slaves.size(); ++i) {
              std::cout << "  N 3." << i + 1
                        << ". slave_name=" << nrtparm.slaves[i].slave_name
                        << ", phy_id=" << nrtparm.slaves[i].phy_id
                        << ", alias=" << nrtparm.slaves[i].alias
                        << ", slave_state=" << nrtparm.slaves[i].slave_state
                        << ", is_online=" << nrtparm.slaves[i].is_online
                        << ", is_virtual=" << nrtparm.slaves[i].is_virtual
                        << ", is_error=" << nrtparm.slaves[i].is_error
                        << std::endl;
            }

            // 计算每个模型的数据量（假设每个模型的数据量相同）
            size_t joints_per_model =
                nrtparm.models_joints.size() / nrtparm.models.size();
            size_t tools_per_model =
                nrtparm.models_tools.size() / nrtparm.models.size();
            size_t wobjs_per_model =
                nrtparm.models_wobjs.size() / nrtparm.models.size();
            size_t loads_per_model =
                nrtparm.models_loads.size() / nrtparm.models.size();
            size_t teach_points_per_model =
                nrtparm.models_teach_points.size() / nrtparm.models.size();

            // 打印 models 信息
            std::cout << "N 4.models: " << std::endl;
            for (size_t model_idx = 0; model_idx < nrtparm.models.size();
                 ++model_idx) {
              const auto &model = nrtparm.models[model_idx];
              std::cout << "  N 4." << model_idx + 1
                        << ". model_name=" << model.model_name
                        << ", model_type=" << model.model_type
                        << ", is_using_sp=" << model.is_using_sp
                        << ", is_collision_detection="
                        << model.is_collision_detection << std::endl;

              // 计算当前模型的起始索引
              size_t joint_start_idx = model_idx * joints_per_model;
              size_t tool_start_idx = model_idx * tools_per_model;
              size_t wobj_start_idx = model_idx * wobjs_per_model;
              size_t load_start_idx = model_idx * loads_per_model;
              size_t teach_point_start_idx = model_idx * teach_points_per_model;

              // 打印 joints 信息
              std::cout << "    N 5.joints for Model " << model_idx << ":"
                        << std::endl;
              for (size_t j = 0;
                   j < joints_per_model &&
                   (joint_start_idx + j) < nrtparm.models_joints.size();
                   ++j) {
                const auto &joint = nrtparm.models_joints[joint_start_idx + j];
                std::cout << "      N 5." << j + 1
                          << ". max_position=" << joint.max_position
                          << ", min_position=" << joint.min_position
                          << ", max_vel=" << joint.max_vel
                          << ", min_vel=" << joint.min_vel
                          << ", max_acc=" << joint.max_acc
                          << ", min_acc=" << joint.min_acc
                          << ", max_collision_torque="
                          << joint.max_collision_torque << std::endl;
              }

              // 打印 tools 信息
              std::cout << "    N 6.tools for Model " << model_idx << ":"
                        << std::endl;
              for (size_t j = 0;
                   j < tools_per_model &&
                   (tool_start_idx + j) < nrtparm.models_tools.size();
                   ++j) {
                const auto &tool = nrtparm.models_tools[tool_start_idx + j];
                std::cout << "      N 6." << j + 1
                          << ". tool_name=" << tool.tool_name << std::endl;
                std::cout << "        data: ";
                for (const auto &data : tool.data) {
                  std::cout << data << " ";
                }
                std::cout << std::endl;
              }

              // 打印 wobjs 信息
              std::cout << "    N 7.wobjs for Model " << model_idx << ":"
                        << std::endl;
              for (size_t j = 0;
                   j < wobjs_per_model &&
                   (wobj_start_idx + j) < nrtparm.models_wobjs.size();
                   ++j) {
                const auto &wobj = nrtparm.models_wobjs[wobj_start_idx + j];
                std::cout << "      N 7." << j + 1
                          << ". wobj_name=" << wobj.wobj_name << std::endl;
                std::cout << "        data: ";
                for (const auto &data : wobj.data) {
                  std::cout << data << " ";
                }
                std::cout << std::endl;
              }

              // 打印 loads 信息
              std::cout << "    N 8.loads for Model " << model_idx << ":"
                        << std::endl;
              for (size_t j = 0;
                   j < loads_per_model &&
                   (load_start_idx + j) < nrtparm.models_loads.size();
                   ++j) {
                const auto &load = nrtparm.models_loads[load_start_idx + j];
                std::cout << "      N 8." << j + 1
                          << ". load_name=" << load.load_name << std::endl;
                std::cout << "        data: ";
                for (const auto &data : load.data) {
                  std::cout << data << " ";
                }
                std::cout << std::endl;
              }

              // 打印 teach_points 信息
              std::cout << "    N 9.teach_points for Model " << model_idx << ":"
                        << std::endl;
              for (size_t j = 0; j < teach_points_per_model &&
                                 (teach_point_start_idx + j) <
                                     nrtparm.models_teach_points.size();
                   ++j) {
                const auto &point =
                    nrtparm.models_teach_points[teach_point_start_idx + j];
                std::cout << "      N 9." << j + 1
                          << ". point_name=" << point.point_name
                          << ", tool_name=" << point.tool_name
                          << ", wobj_name=" << point.wobj_name << std::endl;

                std::cout << "        tool_data: ";
                for (const auto &data : point.tool_data) {
                  std::cout << data << " ";
                }
                std::cout << std::endl;

                std::cout << "        wobj_data: ";
                for (const auto &data : point.wobj_data) {
                  std::cout << data << " ";
                }
                std::cout << std::endl;

                std::cout << "        robottarget: ";
                for (const auto &target : point.robottarget) {
                  std::cout << target << " ";
                }
                std::cout << std::endl;

                std::cout << "        jointtarget: ";
                for (const auto &target : point.jointtarget) {
                  std::cout << target << " ";
                }
                std::cout << std::endl;
              }
            }

            // 打印 subsystems 信息
            std::cout << "N 10.subsystems: " << std::endl;
            for (size_t i = 0; i < nrtparm.subsystems.size(); ++i) {
              const auto &subsystem = nrtparm.subsystems[i];
              std::cout << "  N 10." << i + 1
                        << ". subsystem_name=" << subsystem.subsystem_name
                        << ", id=" << subsystem.id
                        << ", state=" << subsystem.state << std::endl;
            }

            // 打印 sensors 信息
            std::cout << "N 11.sensors: " << std::endl;
            for (size_t i = 0; i < nrtparm.sensors.size(); ++i) {
              const auto &sensor = nrtparm.sensors[i];
              std::cout << "  N 11." << i + 1
                        << ". sensor_name=" << sensor.sensor_name
                        << ", id=" << sensor.id << ", state=" << sensor.state
                        << std::endl;
            }

            // 打印 interfaces 信息
            std::cout << "N 12.interfaces: " << std::endl;
            for (size_t i = 0; i < nrtparm.interfaces.size(); ++i) {
              const auto &interface = nrtparm.interfaces[i];
              std::cout << "  N 12." << i + 1
                        << ". interface_name=" << interface.interface_name
                        << ", id=" << interface.id
                        << ", state=" << interface.state << std::endl;
            }

            std::cout << std::endl;
            /****************************************************************/
          });

#endif

  NodeOptions system_basic_options;
  system_basic_options.node_name = "test3";
  system_basic_options.sub_url = "tcp://192.168.11.11:19091";
  Node system_basic_node(system_basic_options);
  system_basic_node.Start();

  #if 0
  auto subrt =
      system_basic_node.CreateSubscription<system_rtstate::SystemRtState>(
          "system_rtstate", [](const system_rtstate::SystemRtState &tt) {
            SystemStateData rtparm;
            display_basic(tt, rtparm);

            /****************************************************************************/
            std::cout << std::endl;
            std::cout << "4.Models Info:" << std::endl;
            std::cout << std::endl;
            for (size_t i = 0; i < rtparm.models.size(); ++i) {
              std::cout << "  4." << i + 1 << ".  Model " << i << ":"
                        << std::endl;
              std::cout << "    4a.model_name: " << rtparm.models[i].model_name
                        << std::endl;
              std::cout << "    4b.model_type: " << rtparm.models[i].model_type
                        << std::endl;
              std::cout << std::endl;
              // 计算当前模型的关节起始索引
              //  此处每个模型的关节数量相同，可根据实际情况进行逻辑设计
              size_t joints_per_model =
                  rtparm.models_joints.size() / rtparm.models.size();
              size_t joint_start_index = i * joints_per_model;

              std::cout << "   5.Joints Info for Model " << i << ":"
                        << std::endl;
              for (size_t j = 0;
                   j < joints_per_model &&
                   (joint_start_index + j) < rtparm.models_joints.size();
                   ++j) {
                size_t joint_index = joint_start_index + j;
                std::cout << "     Joint " << j << ":" << std::endl;
                // std::cout << "       a.joint_type: " <<
                // rtparm.models_joints[joint_index].joint_type << std::endl;
                std::cout << "       b.position: "
                          << rtparm.models_joints[joint_index].position
                          << std::endl;
                std::cout << "       d.is_enabled: "
                          << rtparm.models_joints[joint_index].is_enabled
                          << std::endl;
                std::cout << std::endl;
              }
              std::cout << std::endl;
              // 检查是否有对应的当前点信息
              if (i < rtparm.models_current_points.size()) {
                std::cout << "   6.Current Point Info for Model " << i << ":"
                          << std::endl;

                std::cout << "     6.6.Robot Target: ";
                for (size_t k = 0;
                     k < rtparm.models_current_points[i].robottarget.size();
                     ++k) {
                  std::cout << rtparm.models_current_points[i].robottarget[k]
                            << " ";
                }

                std::cout << "     6.7.Joint Target: ";
                for (size_t k = 0;
                     k < rtparm.models_current_points[i].jointtarget.size();
                     ++k) {
                  std::cout << rtparm.models_current_points[i].jointtarget[k]
                            << " ";
                }
                std::cout << std::endl;
                std::cout << std::endl;
              }
            }

            std::cout << std::endl;
            /****************************************************************************/
          });

#endif

  while (true) {
    std::this_thread::sleep_for(std::chrono::microseconds(20));
  }
  return 0;
}