#ifndef __MAHONY__H_
#define __MAHONY__H_

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// eigen
#include <Eigen/Geometry>

#include <deque>

struct mahonyData
{
    double timestamp;
    Eigen::Quaterniond q;
    mahonyData()
    {
        timestamp = -1;
        q = q.setIdentity();
    }
    mahonyData(double _timestamp, Eigen::Quaterniond _q)
    {
        timestamp = _timestamp;
        q = _q;
    }
    bool empty()
    {
        return timestamp <= 0 ? true : false;
    }
};

mahonyData find_mahonyData(std::deque<mahonyData> &_deque, const double findTime, bool clear_en = false);

class Mahony
{
public:
    bool file_en;
    bool ESKF_peidict_en;
    std::string fp_path;
    double fusionFactor;

public:
    Mahony()
    {
        twoKp = 1.0 * 0.5;  // 2 * proportional gain
        twoKi = 0.01 * 0.5; // 2 * integral gain
        q0 = 1.0;
        q1 = q2 = q3 = 0.0;
        fusionFactor = 0.0;
        integralFBx = integralFBy = integralFBz = 0.0;
        biasGyro.setZero();
        biasAcc.setZero();
        file_en = false;
        ESKF_peidict_en = false;
    };
    ~Mahony() {};
    Eigen::Quaterniond MahonyAHRSupdateIMU(geometry_msgs::Vector3 gyro, geometry_msgs::Vector3 acc, double deltaT);
    void setBias(Eigen::Vector3d gyro, Eigen::Vector3d acc);
    void calcBias(geometry_msgs::Vector3 gyro, geometry_msgs::Vector3 acc);

private:
    float invSqrt(float x)
    {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long *)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float *)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
    void outputQ(); // output quaternion to txt
    void setfp();

private:
    FILE *fp;
    double twoKp;
    double twoKi;
    double q0, q1, q2, q3;                        // quaternion of sensor frame relative to auxiliary frame
    double integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki
    Eigen::Vector3d biasGyro, biasAcc;            // bias
};

#endif