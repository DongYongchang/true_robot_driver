#include "cpp_rpc.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h> 
#include"message/rpc_client.h"

int main() {

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
    "{Var --type=jointtarget --name=j0 --value={0,0,0,0,0,0,0,0,0,0}}",
    "{Var --type=jointtarget --name=j1 --value={0.1,-1.5,0,0,0,0,0,0,0,0}}",
    "{Var --type=jointtarget --name=j2 --value={0.2,0,0,0,0,0,0,0,0,0}}",
    "{Var --type=jointtarget --name=j3 --value={-0.1,0,0,0,0,0,0,0,0,0}}",
    "{Var --type=jointtarget --name=j4 --value={-0.2,0,0,0,0,0,0,0,0,0}}"
    };

    std::vector<std::string> motion_cmds = {
    "{MoveAbsJ --jointtarget_var=j0||MoveAbsJ --jointtarget_var=j0}",   //"{MoveAbsJ --jointtarget_var=j0||NotRunExecute}"   //双臂情况时的单臂指令
    "{MoveAbsJ --jointtarget_var=j1||MoveAbsJ --jointtarget_var=j1}",
    "{MoveAbsJ --jointtarget_var=j2||MoveAbsJ --jointtarget_var=j2}"
    };

    std::vector<std::string> your_cmds = {

        //add your cmds

    };

    // 创建同步客户端
    std::cout << "Connecting: " << std::endl;
    cpp_rpc::CPPClient client("192.168.11.11", 5868);
    std::cout << "Connected: " << std::endl;

    // 发送初始化指令
    send_rpcsy(client, init_cmds, 500, 100);  //同步rpc (连接client,指令,out time, sleep time)

    // 主循环发送运动指令
    while (1) {

        send_rpcsy(client, motion_cmds, 50000, 1000);
        //send_rpcAsy(client, motion_cmds,10000,500);  //异步rpc (连接client,指令,out time,sleep time)
    }

    return 0;
}

void delay_ms(unsigned int milliseconds);

/*  同步RPC  char* argv[] */
void send_rpcsy(cpp_rpc::CPPClient& client, const std::vector<std::string>& cmd_cmd, int outim_num, int sleep_num) {
    for (const auto& cmd : cmd_cmd) {

        char* fake_argv[] = { const_cast<char*>("dummy"), const_cast<char*>(cmd.c_str()) };

        //获取随机数
        srand(time(0));
        int i = rand();

        std::string msg(fake_argv[1]);


        core::Msg sync_msg(msg);
        sync_msg.setMsgID(10001);
        sync_msg.setMsgSeqID(i);

        auto res = client.CallAwait<RespDemo>(sync_msg, outim_num);
        if (0 == res.first) {
            std::cout << "*************Sync***************" << std::endl;
            std::cout << "model size:" << res.second.size() << std::endl;
            for (const auto& r : res.second) {
                std::cout << "subcmd_index:" << r.subcmd_index << std::endl;
                std::cout << "return_code:" << r.return_code << std::endl;
                std::cout << "return_message:" << r.return_message << std::endl;
            }
            std::cout << "****************************" << std::endl;
        }
        else {
            std::cout << "同步请求失败!确保out time大于指令生效时间!错误码: " << res.first << std::endl;
            // 自动清理错误
            // client.ClearErr();
        }

        std::cout << "over!!!" << std::endl;
        std::cout << "****************************" << std::endl;
        delay_ms(sleep_num);
    }

}

/*  异步RPC  */
void send_rpcAsy(cpp_rpc::CPPClient& client, const std::vector<std::string>& cmd_cmd, int outim_num, int wait_num) {
    for (const auto& cmd : cmd_cmd) {

        char* fake_argv[] = { const_cast<char*>("dummy"), const_cast<char*>(cmd.c_str()) };

        //获取随机数
        srand(time(0));
        int i = rand();

        std::string msg(fake_argv[1]);


        core::Msg message(msg);
        client.CallAsync<RespDemo>(message, outim_num, [&](int ret, std::vector<RespDemo> res) {

            std::cout << "**************Async**************" << std::endl;
            if (ret < 0) {
                std::cout << "Async request failed. ret:" << ret << " " << "out time !" << std::endl;
            }

            std::cout << "model size:" << res.size() << std::endl;
            for (const auto& r : res) {
                std::cout << "subcmd_index:" << r.subcmd_index << std::endl;
                std::cout << "return_code:" << r.return_code << std::endl;
                std::cout << "return_message:" << r.return_message << std::endl;
            }
            std::cout << "*********************************" << std::endl;
            });

        core::Msg sync_msg(msg);
        sync_msg.setMsgID(10001);
        sync_msg.setMsgSeqID(i);

        // std::this_thread::sleep_for(std::chrono::seconds(wait_num)); // 等待异步操作完成
        delay_ms(wait_num);  // 等待异步操作完成
    }
}





#ifdef _WIN32
#include <windows.h>
void delay_ms(unsigned int ms) { Sleep(ms); }
#else
#include <unistd.h>
void delay_ms(unsigned int ms) { usleep(ms * 1000); }
#endif