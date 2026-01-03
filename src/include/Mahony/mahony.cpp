#include "mahony.h"
// glog
#include "glog/logging.h" //头文件

mahonyData find_mahonyData(std::deque<mahonyData> &_deque, const double findTime, bool clear_en)
{
    mahonyData _data;
    for (int i = 0; i < _deque.size(); i++)
    {
        if (findTime == _deque[i].timestamp)
        {
            _data = _deque[i];
            if (clear_en)
            {
                _deque.pop_front();
            }
            break;
        }
        else if (findTime > _deque[i].timestamp)
        {
            if (clear_en)
            {
                _deque.pop_front();
            }
        }
        else if (findTime < _deque[i].timestamp)
        {
            break;
        }
    }
    return _data;
}

Eigen::Quaterniond Mahony::MahonyAHRSupdateIMU(geometry_msgs::Vector3 gyro, geometry_msgs::Vector3 acc, double deltaT)
{
#define GRAV (9.81) // imu acc norm = 1
                    // #define GRAV (9.81) // imu acc norm = 9.81
    double gx = gyro.x - biasGyro(0), gy = gyro.y - biasGyro(1), gz = gyro.z - biasGyro(2);
    double ax = acc.x * GRAV - biasAcc(0), ay = acc.y * GRAV - biasAcc(1), az = acc.z * GRAV - biasAcc(2);
    // double ax = acc.x, ay = acc.y, az = acc.z;
    double recipNorm;
    double halfvx, halfvy, halfvz;
    double halfex, halfey, halfez;
    double qa, qb, qc;

    // if(biasGyro.isZero() && biasAcc.isZero()){
    //     LOG_EVERY_T(INFO, 1) << "Please set bias, Mahony is not running...";
    // 	Eigen::Quaterniond q;
    //     return q.setIdentity();
    // }
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * deltaT; // integral error scaled by Ki
            integralFBy += twoKi * halfey * deltaT;
            integralFBz += twoKi * halfez * deltaT;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * deltaT); // pre-multiply common factors
    gy *= (0.5f * deltaT);
    gz *= (0.5f * deltaT);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    if (file_en)
    {
        outputQ();
    }
    Eigen::Quaterniond quat(q0, q1, q2, q3);
    return quat;
}

void Mahony::setBias(Eigen::Vector3d gyro, Eigen::Vector3d acc)
{
    biasGyro = gyro;
    biasAcc = acc;
}
void Mahony::calcBias(geometry_msgs::Vector3 gyro, geometry_msgs::Vector3 acc)
{
    static uint16_t cnt = 0;
    static Eigen::Vector3d sum_gyro, sum_acc;
    const uint16_t cntLimit = 2000;
    if (!(biasGyro.isZero() && biasAcc.isZero()))
    {
        return;
    }
    if (cnt < cntLimit)
    {
        cnt++;
        sum_gyro(0) += gyro.x;
        sum_gyro(1) += gyro.y;
        sum_gyro(2) += gyro.z;
        sum_acc(0) += acc.x;
        sum_acc(1) += acc.y;
        sum_acc(2) += acc.z;
    }
    else
    {
        biasGyro(0) = sum_gyro(0) / cnt;
        biasGyro(1) = sum_gyro(1) / cnt;
        biasGyro(2) = sum_gyro(2) / cnt;
        biasAcc(0) = sum_acc(0) / cnt;
        biasAcc(1) = sum_acc(1) / cnt;
        biasAcc(2) = sum_acc(2) / cnt - 9.81;
        std::cout << "calc finish, bias gyro = " << biasGyro(0) << " " << biasGyro(1) << " " << biasGyro(2)
                  << "\nbias acc = " << biasAcc(0) << " " << biasAcc(1) << " " << biasAcc(2) << std::endl;
    }
}
void Mahony::outputQ()
{
    if (fp == nullptr)
    {
        setfp();
        if (fp == nullptr)
        {
            std::cout << "Warning: fp is not init!!!" << std::endl;
            return;
        }
    }
    fprintf(fp, "%lf %lf %lf %lf ", q0, q1, q2, q3);
    fprintf(fp, "\r\n");
    fflush(fp);
}
void Mahony::setfp()
{
    std::cout << "file_en = " << file_en << " path = " << fp_path << std::endl;
    if (file_en)
    {
        fp = fopen(fp_path.c_str(), "w");
    }
}