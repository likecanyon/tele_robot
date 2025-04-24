
#include <stdio.h>
#include "dhdc.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <Eigen/Dense>
#include <thread>
using namespace ur_rtde;
using namespace std::chrono;
// simple spring model which pulls the device
// towards the center of the workspace;
// if the user lifts the device 5cm above the center,
// the application exits
using namespace ur_rtde;
int main(int argc,
         char **argv)
{
    RTDEControlInterface rtde_control("192.168.3.101");
    RTDEReceiveInterface rtde_receive("192.168.3.101");
    if (dhdOpen() < 0)
    {
        printf("error: cannot open device\n");
    }
    int button_state;
    std::vector<double> TCPPose(6);
    std::vector<double> TCPPoseInit(6);
    double current_position[3] = {0.0, 0.0, 0.0};
    double current_position_Init[3] = {0.0, 0.0, 0.0};
    double current_orientRad[3] = {0.0, 0.0, 0.0};
    double current_orientRad_Init[3] = {0.0, 0.0, 0.0};
    double Omega_matrix[3][3];
    double Omega_matrixInit[3][3];
    Eigen::Matrix3d Omega_matrixEigen;
    Eigen::Matrix3d Omega_matrixInitEigen;
    Eigen::Matrix3d DeltaR;
    Eigen::Matrix3d Tcp_Rotation;
    Eigen::Vector3d axis;
    double alpha{0.0};
    Eigen::Vector3d axisInit;
    double alphaInit{0.0};
    Eigen::Matrix3d Tcp_Rotation_Init;
    Tcp_Rotation_Init.setIdentity();
    Eigen::Matrix3d R;
    Eigen::AngleAxisd K;
    Eigen::Vector3d Khat;

    double velocity = 0.5;
    double acceleration = 0.5;
    double dt = 1.0 / 500; // 2ms
    double lookahead_time = 0.1;
    double gain = 300;

    rtde_control.moveJ({0, -90 * 3.1415926 / 180, -90 * 3.1415926 / 180, -90 * 3.1415926 / 180, 90 * 3.1415926 / 180, 0});

    while (1)
    {
        button_state = dhdGetButton(0);
        auto t_start = high_resolution_clock::now();
        if (button_state)
        {
            dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
            dhdGetOrientationFrame(Omega_matrix);
            TCPPose[0] = TCPPoseInit[0] + (current_position[0] - current_position_Init[0]);
            TCPPose[1] = TCPPoseInit[1] + (current_position[1] - current_position_Init[1]);
            TCPPose[2] = TCPPoseInit[2] + (current_position[2] - current_position_Init[2]);

            // 计算姿态
            // 首先计算deltaR，deltaR=omega新位姿*omegaInit位姿的转置
            Omega_matrixEigen << Omega_matrix[0][0], Omega_matrix[0][1], Omega_matrix[0][2],
                Omega_matrix[1][0], Omega_matrix[1][1], Omega_matrix[1][2],
                Omega_matrix[2][0], Omega_matrix[2][1], Omega_matrix[2][2];

            DeltaR = Omega_matrixEigen * Omega_matrixInitEigen.transpose();
            // 新的tcp姿态=deltaR*tcpinit姿态,进而得到TcpPose[3],TcpPose[4],TcpPose[5]
            Tcp_Rotation = DeltaR * Tcp_Rotation_Init;
            R = Tcp_Rotation_Init.inverse() * Tcp_Rotation;
            K.fromRotationMatrix(R);
            Khat = K.axis();
            alpha = K.angle();
            R = Eigen::AngleAxisd(alpha, Khat);
            Tcp_Rotation = Tcp_Rotation_Init * R;
            K.fromRotationMatrix(Tcp_Rotation);
            Khat = K.axis();
            alpha = K.angle();
            TCPPose[3] = Khat[0] * alpha;
            TCPPose[4] = Khat[1] * alpha;
            TCPPose[5] = Khat[2] * alpha;
            rtde_control.servoL(TCPPose, velocity, acceleration, dt, lookahead_time, gain);
        }

        else
        {
            TCPPoseInit = rtde_receive.getActualTCPPose();
            dhdGetPosition(&current_position_Init[0], &current_position_Init[1], &current_position_Init[2]);
            dhdGetOrientationFrame(Omega_matrixInit);
            Omega_matrixInitEigen << Omega_matrixInit[0][0], Omega_matrixInit[0][1], Omega_matrixInit[0][2],
                Omega_matrixInit[1][0], Omega_matrixInit[1][1], Omega_matrixInit[1][2],
                Omega_matrixInit[2][0], Omega_matrixInit[2][1], Omega_matrixInit[2][2];

            // 计算机器人Init姿态
            axisInit = {TCPPoseInit[3], TCPPoseInit[4], TCPPoseInit[5]};
            axisInit.normalize();
            alphaInit = TCPPoseInit[3] / axisInit[0];
            Tcp_Rotation_Init = Eigen::AngleAxisd(alphaInit, axisInit);
        }

        auto t_stop = high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);

        if (t_duration.count() < 0.005)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.005 - t_duration.count()));
        }
    }

    return 0;
}