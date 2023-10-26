/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <chrono>

#include <ctime>  
#include <stdio.h>  

#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "unitree_legged_sdk/state_estimator_lcmt.hpp"
#include "unitree_legged_sdk/leg_control_data_lcmt.hpp"
#include "unitree_legged_sdk/pd_tau_targets_lcmt.hpp"
#include "unitree_legged_sdk/rc_command_lcmt.hpp"


#include <plog/Log.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>



using namespace std;
using namespace UNITREE_LEGGED_SDK;


struct Joint // This is our custom type that we want to print to the log stream.
{
    float p[12] = {0};
    float b[2] = {0};
};

namespace plog
{
    Record& operator<<(Record& record, const Joint& jp) // Implement a stream operator for our type.
    {
        return record 
        << ","
        << jp.p[0] << ","<< jp.p[1]<< "," << jp.p[2]<< ","  // csv ，号换列
        << jp.p[3] << ","<< jp.p[4]<< "," << jp.p[5]<< "," 
        << jp.p[6] << ","<< jp.p[7]<< "," << jp.p[8]<< "," 
        << jp.p[9] << ","<< jp.p[10]<< "," << jp.p[11]<< "," 
        ;
    }
}

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level) {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void init();
    void handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg);
    void _simpleLCMThread();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    float frequency = 0;
    
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    int show = 0;
    int show_count = 0;
    float dt = 0.002;     // 0.001~0.01 循环速度2ms

    lcm::LCM _simpleLCM;
    std::thread _simple_LCM_thread;
    bool _firstCommandReceived;
    bool _firstRun;
    state_estimator_lcmt body_state_simple = {0};
    leg_control_data_lcmt joint_state_simple = {0};
    pd_tau_targets_lcmt joint_command_simple = {0};
    rc_command_lcmt rc_command = {0};

    xRockerBtnDataStruct _keyData;
    int mode = 0;

};

void Custom::init()
{
    _simpleLCM.subscribe("pd_plustau_targets", &Custom::handleActionLCM, this);
    _simple_LCM_thread = std::thread(&Custom::_simpleLCMThread, this);

    _firstCommandReceived = false;
    _firstRun = true;

    // set nominal pose

    for(int i = 0; i < 12; i++){
        joint_command_simple.qd_des[i] = 0;
        joint_command_simple.tau_ff[i] = 0;
        joint_command_simple.kp[i] = 20.;
        joint_command_simple.kd[i] = 0.5;
    }

    joint_command_simple.q_des[0] = -0.3;
    joint_command_simple.q_des[1] = 1.2;
    joint_command_simple.q_des[2] = -2.721;
    joint_command_simple.q_des[3] = 0.3;
    joint_command_simple.q_des[4] = 1.2;
    joint_command_simple.q_des[5] = -2.721;
    joint_command_simple.q_des[6] = -0.3;
    joint_command_simple.q_des[7] = 1.2;
    joint_command_simple.q_des[8] = -2.721;
    joint_command_simple.q_des[9] = 0.3;
    joint_command_simple.q_des[10] = 1.2;
    joint_command_simple.q_des[11] = -2.721;

    printf("SET NOMINAL POSE");


}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg){
    (void) rbuf;
    (void) chan;

    joint_command_simple = *msg;
    _firstCommandReceived = true;

}

void Custom::_simpleLCMThread(){
    while(true){
        _simpleLCM.handle();
    }
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);

    memcpy(&_keyData, &state.wirelessRemote[0], 40);

    rc_command.left_stick[0] = _keyData.lx;
    rc_command.left_stick[1] = _keyData.ly;
    rc_command.right_stick[0] = _keyData.rx;
    rc_command.right_stick[1] = _keyData.ry;
    rc_command.right_lower_right_switch = _keyData.btn.components.R2;
    rc_command.right_upper_switch = _keyData.btn.components.R1;
    rc_command.left_lower_left_switch = _keyData.btn.components.L2;
    rc_command.left_upper_switch = _keyData.btn.components.L1;
    
    if (_keyData.btn.components.start == 1)
    {
        show = 1;
    }
    
    if(_keyData.btn.components.A > 0){
        mode = 0;
    } else if(_keyData.btn.components.B > 0){
        mode = 1;
    }else if(_keyData.btn.components.X > 0){
        mode = 2;
    }else if(_keyData.btn.components.Y > 0){
        mode = 3;
    }else if(_keyData.btn.components.up > 0){
        mode = 4;
    }else if(_keyData.btn.components.right > 0){
        mode = 5;
    }else if(_keyData.btn.components.down > 0){
        mode = 6;
    }else if(_keyData.btn.components.left > 0){
        mode = 7;
    }

    rc_command.mode = mode;

    // printf("joint_state_simple[1]:%f",joint_state_simple.q[1]);
    // publish state to LCM
    for(int i = 0; i < 12; i++){
        joint_state_simple.q[i] = state.motorState[i].q;  // pos
        joint_state_simple.qd[i] = state.motorState[i].dq;  //vel
        joint_state_simple.tau_est[i] = state.motorState[i].tauEst; //tau
    }
    // printf("motorState[1]:%f",state.motorState[1].q);
    for(int i = 0; i < 4; i++){
        body_state_simple.quat[i] = state.imu.quaternion[i]; //4
    }
    // printf("\n");
    for(int i = 0; i < 3; i++){
        body_state_simple.rpy[i] = state.imu.rpy[i];
        body_state_simple.aBody[i] = state.imu.accelerometer[i];
        body_state_simple.omegaBody[i] = state.imu.gyroscope[i];
    }
    for(int i = 0; i < 4; i++){
        body_state_simple.contact_estimate[i] = state.footForce[i];
    }
    
    _simpleLCM.publish("state_estimator_data", &body_state_simple);
    _simpleLCM.publish("leg_control_data", &joint_state_simple);
    _simpleLCM.publish("rc_command", &rc_command);

    if(_firstRun && joint_state_simple.q[0] != 0){
        for(int i = 0; i < 12; i++){
            joint_command_simple.q_des[i] = joint_state_simple.q[i];
        }
        _firstRun = false;
    }

    Joint J_p ;
    Joint J_cmd_p ;
    Joint J_tau ;
    Joint J_cmd_tau ;
    Joint bt ;
    bt.b[0] = {0};
    
    for(int i = 0; i < 12; i++){
        J_p.p[i] = joint_state_simple.q[i];
        J_tau.p[i] = joint_state_simple.tau_est[i];
        J_cmd_p.p[i] = joint_command_simple.q_des[i];
        J_cmd_tau.p[i] = joint_command_simple.tau_ff[i];
        
    }
    bt.b[0] = rc_command.right_lower_right_switch;
    bt.b[1] = motiontime;
    
    
    

    
    // 打印信息
    if(show == 1){
        show_count++;
        if (show_count >= 5000)
        {
            show = 0;
            show_count = 0;
            // printf("show end");
        }
        
        if (show_count%100 == 0){
            // 打印关节位置
            PLOGI <<","<< bt.b[1] << "," << "joint_state_q" << J_p <<bt.b[0]; 
            // 打印关节力矩
            PLOGI <<","<< bt.b[1] << "," <<"joint_state_tau_est" << J_tau <<bt.b[0]; 
            // 打印关节位置命令
            PLOGI <<","<< bt.b[1] << "," <<"joint_command_q" << J_cmd_p <<bt.b[0];
            // 打印关节位置命令
            PLOGI <<","<< bt.b[1] << "," <<"joint_command_tau" << J_cmd_tau <<bt.b[0]; 
            printf("footForce:[");
            for(int i = 0; i < 4; i++){
                printf("%i,", state.footForce[i]);
            }
            printf("]\n");
            }
        
        
        
    }

    // 发送命令
    for(int i = 0; i < 12; i++){
        cmd.motorCmd[i].q = joint_command_simple.q_des[i];
        cmd.motorCmd[i].dq = joint_command_simple.qd_des[i];
        cmd.motorCmd[i].Kp = joint_command_simple.kp[i];
        cmd.motorCmd[i].Kd = joint_command_simple.kd[i];
        cmd.motorCmd[i].tau = joint_command_simple.tau_ff[i];        
    }


    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, 10);
    udp.SetSend(cmd);
}




int main(void)
{
    plog::init(plog::debug, "lcm_log.csv"); // Initialize the logger.
    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::get()->addAppender(&consoleAppender); // Also add logging to the console.

    


    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    // PLOG_WARNING << "Communication level is set to LOW-level.";
    // PLOG_WARNING << "WARNING: Make sure the robot is hung up.";
    // PLOG_WARNING << "Press Enter to continue...";
    std::cin.ignore();
    PLOG_INFO <<","<< "step" <<","<<"name"<<","
    << "joint 0" <<","<< "joint 1" <<","<< "joint 2" <<","
    << "joint 3" <<","<< "joint 4" <<","<< "joint 5" <<","
    << "joint 6" <<","<< "joint 7" <<","<< "joint 8" <<","
    << "joint 9" <<","<< "joint 10" <<","<< "joint 11" <<","
    << "button R2";

    

    Custom custom(LOWLEVEL);
    custom.init();
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0;
}
