//
// Created by 34575 on 25-6-6.
//

#ifndef CONTROL_H
#define CONTROL_H

#include <config.h>

struct SysData {
    float pitch{}, roll{}, yaw{};
    float yaw_offset{}; // 偏航角偏移量
    float yaw_init{}; // 偏航角初始值
    enum _gyro {
        x,y,z
    };
    short gyro[3] = {0, 0, 0}; // x,y,z轴陀螺仪原始数据
    short acc[3] = {0, 0, 0}; // x,y,z轴加速度计原始数据
    short gyro_kalman[3] = {0, 0, 0}; // x,y,z轴陀螺仪卡尔曼滤波数据
    short gyro_lowpass[3] = {0, 0, 0}; // x,y,z轴陀螺仪低通滤波数据
    float Get_yaw_offset() {
        if (yaw < 0) {
            yaw_offset = -yaw;
            return yaw_offset;
        } else {
            yaw_offset = yaw;
            return yaw;
        }
    }
    void Yaw_Init() {
        yaw_init = yaw_offset;
    }
};

extern SysData data;

namespace Control {

    namespace Filter {
        class LowPassFilter {
        public:
            float alpha;
            float last_output;
            explicit LowPassFilter(const float alpha) : alpha(alpha), last_output(0.0f) {}
            float update(float input) {
                last_output = alpha * input + (1 - alpha) * last_output;
                return last_output;
            }
        };
        class KalmanFilter {
        public:
            float Q;  // 过程噪声协方差
            float R;  // 测量噪声协方差
            float X;  // 估计值
            float P = 1.0f;  // 估计误差协方差
            float K = 0.0f;  // 卡尔曼增益
            KalmanFilter (const float process_noise, const float measurement_noise, const float initial_estimate)
                : Q(process_noise), R(measurement_noise), X(initial_estimate) {}
            float Update(float measurement) {
                // 预测更新
                P += Q;
                // 计算卡尔曼增益
                K = P / (P + R);
                // 更新估计值
                X += K * (measurement - X);
                // 更新误差协方差
                P *= (1.0f - K);
                return X;
            }
        };
    }
    class PID {
    public:
        virtual ~PID() = default;

        float p;
        float i;
        float d;
        float _k;
        float error{};
        float last_error{};
        float integral{};
        float integral_value{};
        float derivative_Input{};
        float derivative{};
        float derivative_value{};
        float output{};

        float max_output{};
        float min_output{};

        float max_integral{};
        float min_integral{};

        PID(float p, float i, float d,float _k = 1) : p(p), i(i), d(d), _k(_k) {}
        virtual float update(float target, float current) {
            error = target - current;
            integral += error;
            derivative = derivative_Input;
            /*计算*/
            integral_value = integral * i;
            derivative_value = derivative * d;
            /*积分限幅*/
            integral_value = (integral_value >= max_integral) ? max_integral : integral_value;
            integral_value = (integral_value <= -max_integral) ? -max_integral : integral_value;
            if ((error <= 1)&&(error >= -1)) {
                integral = 0; // 当误差小于1时，清零积分
                integral_value = 0;
            }
            /*计算输出*/
            output = p * error + integral_value + derivative_value;
            /*输出限幅*/
            output = output > max_output ? max_output : output;
            output = output < -max_output ? -max_output : output;
            last_error = error;
            return output;
        }
    };
    #define left_Motor Dirver::PWM::Channel_1
    #define right_Motor Dirver::PWM::Channel_2
    class Turn_Control_PID : public PID {
    public:
        Turn_Control_PID(const float p, const float i, const float d) : PID(p, i, d, 1) {}
        float update(float target, float current) override {
            error = target - current;
            integral += error;
            derivative = data.gyro[SysData::z];
            /*计算*/
            integral_value = integral * i;
            derivative_value = derivative * d;
            /*积分限幅*/
            integral_value = (integral_value >= max_integral) ? max_integral : integral_value;
            integral_value = (integral_value <= -max_integral) ? -max_integral : integral_value;
            if ((error <= 1)&&(error >= -1)) {
                integral = 0; // 当误差小于1时，清零积分
                integral_value = 0;
            }
            /*计算输出*/
            output = p * error + integral_value + derivative_value;
            /*输出限幅*/
            output = output > max_output ? max_output : output;
            output = output < -max_output ? -max_output : output;
            last_error = error;
            return output;
        }
    };
    class Speed_Control_PID : public PID {
    public:
        Speed_Control_PID(const float p, const float i, const float d) : PID(p, i, d, 1) {}
        float update(float target, float current) override {
            error = target - current;
            integral += error;
            derivative = data.acc[SysData::y];
            /*计算*/
            integral_value = integral * i;
            derivative_value = derivative * d;
            /*积分限幅*/
            integral_value = (integral_value >= max_integral) ? max_integral : integral_value;
            integral_value = (integral_value <= -max_integral) ? -max_integral : integral_value;
            if ((error <= 1)&&(error >= -1)) {
                integral = 0; // 当误差小于1时，清零积分
                integral_value = 0;
            }
            /*计算输出*/
            output = p * error + integral_value + derivative_value;
            /*输出限幅*/
            output = output > max_output ? max_output : output;
            output = output < -max_output ? -max_output : output;
            last_error = error;
            return output;
        }
    };
    class Upright_Control_PID : public PID {
    public:
        Upright_Control_PID(const float p, const float i, const float d) : PID(p, i, d, 1) {}
        float update(float target, float current) override {
            error = target - current;
            integral += error;
            derivative = data.gyro[SysData::y];
            /*计算*/
            integral_value = integral * i;
            derivative_value = derivative * d;
            Filter::LowPassFilter lowpass_filter(0.7f);
            derivative_value = lowpass_filter.update(derivative_value);
            /*积分限幅*/
            integral_value = (integral_value >= max_integral) ? max_integral : integral_value;
            integral_value = (integral_value <= -max_integral) ? -max_integral : integral_value;
            if ((error <= 1)&&(error >= -1)) {
                integral = 0; // 当误差小于1时，清零积分
            }
            /*计算输出*/
            output = p * error + integral_value + derivative_value;
            /*输出限幅*/
            output = output > max_output ? max_output : output;
            output = output < -max_output ? -max_output : output;
            last_error = error;
            return output;
        }
    };
    class Speed_Control final : public Speed_Control_PID {
    public:
        Speed_Control(System::Dirver::PWM::Channel Chx,float p, float i, float d,float max_output ,float max_integral) :
        Speed_Control_PID(p,i,d) {
            this->max_output = max_output;
            this->max_integral = max_integral;
        };
        void control(float target);
    };
    class Upright_Control final : public Upright_Control_PID{
        System::Dirver::PWM::Channel Chx;
    public:
        Upright_Control(const System::Dirver::PWM::Channel Chx,float p, float i, float d,float max_output ,float min_output,float max_integral) :
        Upright_Control_PID(p,i,d),Chx(Chx) {
            this->max_output = max_output;
            this->min_output = min_output;
            this->max_integral = max_integral;
        };
        void control(float target,float current);
    };
    class Turn_Control final : public Turn_Control_PID {
    public:
        Turn_Control(float p, float i, float d,float max_output ,float max_integral) :
        Turn_Control_PID(p,i,d) {
            this->max_output = max_output;
            this->max_integral = max_integral;
        };
        void control(float target,float current);
    };
}

#endif //CONTROL_H
