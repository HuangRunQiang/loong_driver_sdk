/* Copyright 2025 人形机器人（上海）有限公司
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Designed and built with love @zhihu by @cjrcl.
 */

#pragma once

#include <vector>
#include <string>
#include <cmath>

namespace DriverSDK     
{
    float const Pi = std::acos(-1); // 圆周率Π

    struct imuStruct // IMU 结构体
    {
        float rpy[3]; // 滚转、俯仰、偏航角
        float gyr[3]; // 陀螺仪数据
        float acc[3]; // 加速度计数据
    };

    struct sensorStruct // 传感器结构体
    {
        float F[3];              // 力
        float M[3];              // 劳动矩
        unsigned int statusCode; // 状态码
    };

    struct digitTargetStruct // 数字目标结构体
    {
        unsigned short pos; // 位置: 0 ~ 90: 放松 ~ 紧张
    };

    struct digitActualStruct // 数字实际结构体
    {
        unsigned short pos; // 位置: 0 ~ 90: 放松 ~ 紧张
    };

    struct motorTargetStruct // 电机目标结构体
    {
        float pos;   // 位置
        float vel;   // 速度
        float tor;   // 力矩
        int enabled; // -1: 清除错误, 0: 禁用, 1: 启用
    };

    struct motorActualStruct // 电机实际结构体
    {
        float pos;                 // 位置
        float vel;                 // 速度
        float tor;                 // 力矩
        short temp;                // 温度
        unsigned short statusWord; // 状态字: 0: 准备操作; 65535: 不活动
        unsigned short errorCode;  // 错误码
    };

    class motorSDOClass // 电机SDO类
    {
    public:
        long value;              // 值
        int i;                   // 驱动器索引[i]
        short state;             // 状态: -1: 错误; 0: 待处理; 1, 2: 处理中; 3: 完成
        unsigned short index;    // 索引
        unsigned char subindex;  // 子索引
        unsigned char signed_;   // 0: 无符号; 1: 有符号
        unsigned char bitLength; // 位长度: 8, 16 或 32
        unsigned char operation; // 操作: 0: 写入; 1: 读取
        motorSDOClass(int i);    // 构造函数
        ~motorSDOClass();        // 析构函数
    };

    class DriverSDK
    {
    public:
        static DriverSDK &instance();
        void setCPU(unsigned short const cpu);
        void setMaxCurr(std::vector<unsigned short> const &maxCurr);
        int setMode(std::vector<char> const &mode);
        void init(char const *xmlFile);
        int getLeftDigitNr();
        int getRightDigitNr();
        int getTotalMotorNr();
        std::vector<int> getActiveMotors();
        int setCntBias(std::vector<int> const &cntBias);
        int fillSDO(motorSDOClass &data, char const *object);
        void getIMU(imuStruct &data);
        int getSensor(std::vector<sensorStruct> &data);
        int setDigitTarget(std::vector<digitTargetStruct> const &data);
        int getDigitActual(std::vector<digitActualStruct> &data);
        int setMotorTarget(std::vector<motorTargetStruct> const &data);
        int getMotorActual(std::vector<motorActualStruct> &data);
        int sendMotorSDORequest(motorSDOClass const &data);
        int recvMotorSDOResponse(motorSDOClass &data);
        int calibrate(int const i);
        void advance();
        std::string version();

    private:
        class impClass;
        impClass &imp;
        DriverSDK();
        ~DriverSDK();
        DriverSDK(DriverSDK const &) = delete;
        DriverSDK &operator=(DriverSDK const &) = delete;
    };
}