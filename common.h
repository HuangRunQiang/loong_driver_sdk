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

#pragma once                                                               // 防止头文件重复包含的预处理指令

#include <ecrt.h>                                                          // 包含EtherCAT实时库头文件
#include <string>                                                          // 包含C++标准字符串库
#include <atomic>                                                          // 包含C++原子操作库

#define NSEC_PER_SEC 1000000000L                                           // 定义每秒的纳秒数常量
#define TIMESPEC2NS(T) T.tv_sec *NSEC_PER_SEC + T.tv_nsec                 // 定义时间结构体转换为纳秒的宏

namespace DriverSDK                                                        // 定义驱动SDK命名空间
{                                                                          // 命名空间开始
    // 定义交换节点类
    class SwapNode                                                         
    {                                                                      
    public:                                                                
        unsigned char *memPtr;                                             // 内存指针，指向数据缓冲区
        SwapNode *previous, *next;                                         // 前一个和后一个节点指针，形成双向链表
        SwapNode(int const size);                                          // 构造函数声明，参数为缓冲区大小
        ~SwapNode();                                                       // 析构函数声明
    };                                                                     // SwapNode类定义结束

    // 定义交换列表类
    class SwapList                                                         
    {                                                                      
    public:                                                                
        std::atomic<SwapNode *> nodePtr;                                   // 原子操作的节点指针，用于线程安全
        SwapList(int const size);                                          // 构造函数声明，参数为节点大小
        void advanceNodePtr();                                             // 前进节点指针的方法声明
        void copyTo(unsigned char *domainPtr, int const domainSize);       // 复制数据到域指针的方法声明
        void copyFrom(unsigned char const *domainPtr, int const domainSize);// 从域指针复制数据的方法声明
        ~SwapList();                                                       // 析构函数声明
    };                                                                     // SwapList类定义结束

    // 定义SDO消息结构体
    struct SDOMsg                                                          
    {                                                                      // 结构体定义开始
        ec_sdo_request_t *sdoHandler;                                      // SDO请求处理器指针
        long value;                                                        // SDO数值
        int alias;                                                         // 别名标识符
        short state; // -1: error; 0: pending; 1, 2: processing; 3: completed // 状态标识：-1错误，0等待，1-2处理中，3完成
        unsigned short index;                                              // SDO索引值
        unsigned char subindex;                                            // SDO子索引值
        unsigned char signed_;   // 0: unsigned; 1: signed                 // 符号标识：0无符号，1有符号
        unsigned char bitLength; // 8, 16 or 32                           // 位长度：8、16或32位
        unsigned char operation; // 0: write; 1: read                     // 操作类型：0写入，1读取
        int recycled;                                                      // 回收标识
    };                                                                     // SDOMsg结构体定义结束

    // 定义驱动器接收数据结构体
    struct DriverRxData                                                    
    {                                                                      // 结构体定义开始
        int TargetPosition;                                                // 目标位置
        int TargetVelocity;                                                // 目标速度
        short TargetTorque;                                                // 目标扭矩
        unsigned short ControlWord;                                        // 控制字
        char Mode;                                                         // 模式
        char Undefined;                                                    // 未定义字段
        short TorqueOffset;                                                // 扭矩偏移
        int VelocityOffset;                                                // 速度偏移
    };                                                                     // DriverRxData结构体定义结束

    // 定义驱动器发送数据结构体
    struct DriverTxData                                                    
    {                                                                      // 结构体定义开始
        int ActualPosition;                                                // 实际位置
        int ActualVelocity;                                                // 实际速度
        short ActualTorque;                                                // 实际扭矩
        unsigned short StatusWord;                                         // 状态字
        char ModeDisplay;                                                  // 模式显示
        char Undefined;                                                    // 未定义字段
        unsigned short ErrorCode;                                          // 错误代码
    };                                                                     // DriverTxData结构体定义结束

    // 定义电机参数类
    class MotorParameters                                                  
    {                                                                      
    public:                                                                
        float polarity, countBias, encoderResolution, gearRatioTor, gearRatioPosVel, ratedCurrent, torqueConstant, ratedTorque, maximumTorque, minimumPosition, maximumPosition; // 电机相关参数：极性、计数偏差、编码器分辨率、扭矩齿轮比、位置速度齿轮比、额定电流、扭矩常数、额定扭矩、最大扭矩、最小位置、最大位置
        SDOMsg sdoTemplate, temperatureSDO, clearErrorSDO;                 // SDO消息模板、温度SDO、清除错误SDO
        MotorParameters();                                                 // 构造函数声明
        int load(std::string const &bus, int const alias, std::string const &type, ec_sdo_request_t *const sdoHandler); // 加载参数方法声明
        ~MotorParameters();                                                // 析构函数声明
    };                                                                     // MotorParameters类定义结束

    // 定义手部接收数据结构体
    struct HandRxData                                                     
    {                                                                      // 结构体定义开始
        unsigned char stop;                                                // 停止命令
        char Undefined;                                                    // 未定义字段
        unsigned short TargetSpeedThumb;                                   // 拇指目标速度
        unsigned short TargetSpeedThumbBend;                               // 拇指弯曲目标速度
        unsigned short TargetSpeedForefinger;                              // 食指目标速度
        unsigned short TargetSpeedMiddle;                                  // 中指目标速度
        unsigned short TargetSpeedRing;                                    // 无名指目标速度
        unsigned short TargetSpeedLittle;                                  // 小指目标速度
        unsigned short TargetAngleThumb;                                   // 拇指目标角度
        unsigned short TargetAngleThumbBend;                               // 拇指弯曲目标角度
        unsigned short TargetAngleForefinger;                              // 食指目标角度
        unsigned short TargetAngleMiddle;                                  // 中指目标角度
        unsigned short TargetAngleRing;                                    // 无名指目标角度
        unsigned short TargetAngleLittle;                                  // 小指目标角度
        unsigned short CurrentLimitThumb;                                  // 拇指电流限制
        unsigned short CurrentLimitThumbBend;                              // 拇指弯曲电流限制
        unsigned short CurrentLimitForefinger;                             // 食指电流限制
        unsigned short CurrentLimitMiddle;                                 // 中指电流限制
        unsigned short CurrentLimitRing;                                   // 无名指电流限制
        unsigned short CurrentLimitLittle;                                 // 小指电流限制
    };                                                                     // HandRxData结构体定义结束

    // 定义手部发送数据结构体
    struct HandTxData                                                     
    {                                                                      // 结构体定义开始
        unsigned short TouchSensorThumb[4];                                // 拇指触觉传感器数组（4个传感器）
        unsigned short TouchSensorForefinger[4];                           // 食指触觉传感器数组（4个传感器）
        unsigned short TouchSensorMiddle[4];                               // 中指触觉传感器数组（4个传感器）
        unsigned short TouchSensorRing[4];                                 // 无名指触觉传感器数组（4个传感器）
        unsigned short TouchSensorLittle[4];                               // 小指触觉传感器数组（4个传感器）
        unsigned short ActualAngleThumb;                                   // 拇指实际角度
        unsigned short ActualAngleThumbBend;                               // 拇指弯曲实际角度
        unsigned short ActualAngleForefinger;                              // 食指实际角度
        unsigned short ActualAngleMiddle;                                  // 中指实际角度
        unsigned short ActualAngleRing;                                    // 无名指实际角度
        unsigned short ActualAngleLittle;                                  // 小指实际角度
        unsigned short ActualCurrentThumb;                                 // 拇指实际电流
        unsigned short ActualCurrentThumbBend;                             // 拇指弯曲实际电流
        unsigned short ActualCurrentForefinger;                            // 食指实际电流
        unsigned short ActualCurrentMiddle;                                // 中指实际电流
        unsigned short ActualCurrentRing;                                  // 无名指实际电流
        unsigned short ActualCurrentLittle;                                // 小指实际电流
    };                                                                     // HandTxData结构体定义结束

    // 定义数字输入接收数据结构体
    struct DigitRxData                                                    
    {                                                                      // 结构体定义开始
        unsigned short TargetPosition;                                     // 目标位置
    };                                                                     // DigitRxData结构体定义结束

    // 定义数字输入发送数据结构体
    struct DigitTxData                                                    
    {                                                                      // 结构体定义开始
        unsigned short ActualPosition;                                     // 实际位置
    };                                                                     // DigitTxData结构体定义结束

    // 定义转换器数据结构体
    struct ConverterDatum                                                 
    {                                                                      // 结构体定义开始
        unsigned short Index;                                              // 索引
        unsigned short ID;                                                 // 标识符
        unsigned short Length;                                             // 数据长度
        unsigned char Data[64];                                            // 数据缓冲区（64字节）
    };                                                                     // ConverterDatum结构体定义结束

    // 定义转换器接收数据结构体
    struct ConverterRxData                                                
    {                                                                      // 结构体定义开始
        ConverterDatum channels[8];                                        // 8个通道的转换器数据
    };                                                                     // ConverterRxData结构体定义结束

    // 定义转换器发送数据结构体
    struct ConverterTxData                                                
    {                                                                      // 结构体定义开始
        ConverterDatum channels[8];                                        // 8个通道的转换器数据
    };                                                                     // ConverterTxData结构体定义结束

    // 定义执行器参数类
    class EffectorParameters                                              
    {                                                                      
    public:                                                                
        EffectorParameters();                                              // 构造函数声明
        int load(std::string const &bus, int const alias, std::string const &type, ec_sdo_request_t *const sdoHandler); // 加载参数方法声明
        ~EffectorParameters();                                             // 析构函数声明
    };                                                                     // EffectorParameters类定义结束

    // 定义传感器接收数据结构体
    struct SensorRxData                                                   
    {                                                                      // 结构体定义开始
        int ControlCode;                                                   // 控制代码
        float x;                                                           // X轴数据
        float y;                                                           // Y轴数据
        float z;                                                           // Z轴数据
        float a;                                                           // A轴数据
        float b;                                                           // B轴数据
        float c;                                                           // C轴数据
        float d;                                                           // D轴数据
    };                                                                     // SensorRxData结构体定义结束

    // 定义传感器发送数据结构体
    struct SensorTxData                                                   
    {                                                                      // 结构体定义开始
        int Fx;                                                            // X轴力
        int Fy;                                                            // Y轴力
        int Fz;                                                            // Z轴力
        int Mx;                                                            // X轴力矩
        int My;                                                            // Y轴力矩
        int Mz;                                                            // Z轴力矩
        unsigned int StatusCode;                                           // 状态代码
        unsigned int SampleCounter;                                        // 采样计数器
        int Temper;                                                        // 温度
    };                                                                     // SensorTxData结构体定义结束

    // 定义传感器参数类
    class SensorParameters                                                
    {                                                                      
    public:                                                                
        SensorParameters();                                                // 构造函数声明
        int load(std::string const &bus, int const alias, std::string const &type, ec_sdo_request_t *const sdoHandler); // 加载参数方法声明
        ~SensorParameters();                                               // 析构函数声明
    };                                                                     // SensorParameters类定义结束

    template <typename Data>                                               // 定义数据包装器模板类  
    // 数据包装器类定义
    class DataWrapper                                                      
    {                                                                      
    public:                                                                
        Data *data;                                                        
        int offset;                                                        // 数据偏移量
        SwapList *swap;                                                    // 交换列表指针
         // 构造函数定义
        DataWrapper()                                                     
        {                                                                  
            data = new Data();                                             
            memset(data, 0, sizeof(Data));                                 
            offset = -1;                                                   
            swap = nullptr;                                                
        }                                                                  
        // 初始化方法定义
        void init(int const offset)                                        
        {                                                                  
            this->offset = offset;                                         
        }                                                                  
        // 配置方法定义
        void config(SwapList *const swap)                                  
        {                                                                  
            this->swap = swap;                                             
        }                                                                  
        // 箭头操作符重载定义
        Data *operator->()                                                 
        {                                                                  
            if (swap != nullptr)                                         
            {                                                              
                return (Data *)(swap->nodePtr.load()->memPtr + offset);    // 返回交换列表中对应偏移位置的数据指针
            }                                                              
            return data;                                                   
        }                                                                  
        // 析构函数定义
        ~DataWrapper()                                                     
        {                                                                 
            delete data;                                                   
        }                                                                  
    };                                                                    

    template <typename RxData, typename TxData, typename Parameters>       // 定义包装器对模板类，包含接收数据、发送数据和参数三个模板参数
    // 包装器对类定义
    class WrapperPair                                                      
    {                                                                      
    public:                                                                
        int order, domain, slave, alias, enabled;                         // 顺序、域、从站、别名、启用状态
        std::string bus, type;                                             // 总线名称、类型字符串
        DataWrapper<RxData> rx;                                            // 接收数据包装器
        DataWrapper<TxData> tx;                                            // 发送数据包装器
        ec_sdo_request_t *sdoHandler;                                      // SDO请求处理器指针
        Parameters parameters;                                             // 参数对象
        // 构造函数定义
        WrapperPair()                                                     
        {                                                                  // 构造函数体开始
            order = -1;                                                     
            domain = -1;                                                   
            slave = -1;                                                    
            alias = 0;                                                     
            enabled = 0;                                                   
            bus = "";                                                      
            type = "";                                                     
            sdoHandler = nullptr;                                          
        }                                                                  
        // 初始化方法定义，参数包括总线、顺序、域、从站、别名、类型、接收偏移、发送偏移、SDO处理器
        int init(std::string const &bus, int const order, int const domain, int const slave, int const alias, std::string const &type, int const rxOffset, int const txOffset, ec_sdo_request_t *const sdoHandler) 
        {                                                                  
            if (this->order != -1)                                         
            {                                                              
                printf("trying to re-init %s slave %d:%d with alias %d\n", bus.c_str(), order, slave, alias); 
                return -1;                                                 
            }                                                              
            this->order = order;                                           // 设置顺序
            this->domain = domain;                                         // 设置域
            this->slave = slave;                                           
            this->alias = alias;                                           
            this->bus = bus;                                               // 设置总线名称
            this->type = type;                                             // 设置类型
            rx.init(rxOffset);                                             
            tx.init(txOffset);                                             
            this->sdoHandler = sdoHandler;                                 
            return 0;                                                      
        }                                                                  
        // 配置方法定义，参数包括总线、顺序、域、接收交换列表、发送交换列表
        int config(std::string const &bus, int const order, int const domain, SwapList *const rxSwap, SwapList *const txSwap) 
        {                                                                 
            if (this->order == -1)                                         
            {                                                              
                return 2;                                                
            }                                                              
            if (this->order != order || this->domain != domain || this->bus != bus) // 如果顺序、域或总线不匹配
            {                                                               
                return 1;                                                   
            }                                                              
            rx.config(rxSwap);                                             // 配置接收数据包装器的交换列表
            tx.config(txSwap);                                             // 配置发送数据包装器的交换列表
            if (parameters.load(bus, alias, type, sdoHandler) < 0)       
            {                                                              
                printf("loading parameters failed for %s slave %d:%d with alias %d\n", bus.c_str(), order, slave, alias); 
                return -1;                                                 
            }                                                              
            return 0;                                                      
        }                                                                  
        // 析构函数定义
        ~WrapperPair()                                                     
        {                                                               
        }                                                                 
    };                                                                     
}                                                                         