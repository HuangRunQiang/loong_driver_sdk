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

#include "config_xml.h"
#include "common.h"
#include <limits>
#include <cmath>

namespace DriverSDK
{
    extern ConfigXML *configXML;

    /**
     * @brief SwapNode构造函数，初始化双向链表节点
     * @param size 要分配的内存大小（字节）
     * 
     * 分配指定大小的内存并清零，初始化前后指针为nullptr
     */
    SwapNode::SwapNode(int const size)
    {
        memPtr = (unsigned char *)calloc(size, 1);
        previous = nullptr;
        next = nullptr;
    }

    /**
     * @brief SwapNode析构函数，释放分配的内存
     * 
     * 释放构造函数中分配的内存块
     */
    SwapNode::~SwapNode()
    {
        free(memPtr);
    }

    /**
     * @brief SwapList构造函数，创建3个节点的循环双向链表
     * @param size 每个节点的内存大小（字节）
     * 
     * 创建一个包含3个SwapNode的循环双向链表，用于线程安全的内存交换操作
     */
    SwapList::SwapList(int const size)
    {
        nodePtr.store(new SwapNode(size));
        SwapNode *current = nodePtr.load();
        int i = 1;
        while (i < 3)
        {
            SwapNode *node = new SwapNode(size);
            node->previous = current;
            current->next = node;
            current = current->next;
            i++;
        }
        nodePtr.load()->previous = current;
        current->next = nodePtr.load();
    }

    /**
     * @brief 线程安全地前进到下一个节点
     * 
     * 原子操作将当前节点指针移动到链表中的下一个节点
     */
    void SwapList::advanceNodePtr()
    {
        nodePtr.store(nodePtr.load()->next);
    }

    /**
     * @brief 从链表复制数据到目标内存区域
     * @param domainPtr 目标内存指针
     * @param domainSize 要复制的数据大小（字节）
     * 
     * 从当前节点的前一个节点复制数据到指定的内存区域
     */
    void SwapList::copyTo(unsigned char *domainPtr, int const domainSize)
    {
        memcpy(domainPtr, nodePtr.load()->previous->memPtr, domainSize);
    }

    /**
     * @brief 从源内存区域复制数据到链表
     * @param domainPtr 源内存指针
     * @param domainSize 要复制的数据大小（字节）
     * 
     * 将数据复制到当前节点的下一个节点，并前进节点指针
     */
    void SwapList::copyFrom(unsigned char const *domainPtr, int const domainSize)
    {
        SwapNode *node = nodePtr.load();
        memcpy(node->next->memPtr, domainPtr, domainSize);
        nodePtr.store(node->next);
    }

    /**
     * @brief SwapList析构函数，释放所有节点内存
     * 
     * 遍历循环链表并删除所有节点，释放分配的内存
     */
    SwapList::~SwapList()
    {
        SwapNode *current = nodePtr.load();
        while (current != nullptr)
        {
            SwapNode *node = current;
            current = current->next;
            node->previous->next = nullptr;
            delete node;
        }
    }

    /**
     * @brief MotorParameters构造函数，初始化电机参数默认值
     * 
     * 设置电机参数的默认值，大部分参数设为1.0，位置偏差设为0.0，最小位置设为-1.0
     */
    MotorParameters::MotorParameters()
    {
        countBias = 0.0;
        polarity = encoderResolution = gearRatioTor = gearRatioPosVel = ratedCurrent = torqueConstant = ratedTorque = maximumTorque = maximumPosition = 1.0;
        minimumPosition = -1.0;
    }

    /**
     * @brief 从XML配置文件加载电机参数
     * @param bus 总线类型（"ECAT"、"CAN"或"RS485"）
     * @param alias 电机别名/ID
     * @param type 电机类型
     * @param sdoHandler SDO处理句柄（用于ECAT总线）
     * @return 成功返回0，失败返回-1
     * 
     * 从全局configXML对象中读取指定别名的电机参数，并根据总线类型配置SDO消息
     * 验证所有参数是否正确设置，如有参数缺失则返回错误
     */
    int MotorParameters::load(std::string const &bus, int const alias, std::string const &type, ec_sdo_request_t *const sdoHandler)
    {
        polarity = configXML->readMotorParameter(alias, "Polarity");
        countBias = configXML->readMotorParameter(alias, "CountBias");
        encoderResolution = configXML->readMotorParameter(alias, "EncoderResolution");
        gearRatioTor = configXML->readMotorParameter(alias, "GearRatioTor");
        gearRatioPosVel = configXML->readMotorParameter(alias, "GearRatioPosVel");
        ratedCurrent = configXML->readMotorParameter(alias, "RatedCurrent");
        torqueConstant = configXML->readMotorParameter(alias, "TorqueConstant");
        ratedTorque = configXML->readMotorParameter(alias, "RatedTorque");
        maximumTorque = configXML->readMotorParameter(alias, "MaximumTorque");
        minimumPosition = configXML->readMotorParameter(alias, "MinimumPosition");
        maximumPosition = configXML->readMotorParameter(alias, "MaximumPosition");
        if (polarity == std::numeric_limits<float>::min() ||
            countBias == std::numeric_limits<float>::min() ||
            encoderResolution == std::numeric_limits<float>::min() ||
            gearRatioTor == std::numeric_limits<float>::min() ||
            gearRatioPosVel == std::numeric_limits<float>::min() ||
            ratedCurrent == std::numeric_limits<float>::min() ||
            torqueConstant == std::numeric_limits<float>::min() ||
            ratedTorque == std::numeric_limits<float>::min() ||
            maximumTorque == std::numeric_limits<float>::min() ||
            minimumPosition == std::numeric_limits<float>::min() ||
            maximumPosition == std::numeric_limits<float>::min())
        {
            printf("a motor parameter is incorrectly set in xml\n");
            return -1;
        }
        if (bus == "ECAT")
        {
            sdoTemplate = SDOMsg{
                sdoHandler,
                0,
                alias,
                0,
                0x0000,
                0x00,
                0,
                0,
                0,
                0};
            std::vector<std::string> entry = configXML->entry(configXML->busDevice("ECAT", type.c_str()), "Temperature");
            temperatureSDO = SDOMsg{
                sdoHandler,
                0,
                alias,
                0,
                (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
                (unsigned char)strtoul(entry[2].c_str(), nullptr, 16),
                (unsigned char)strtoul(entry[3].c_str(), nullptr, 10),
                (unsigned char)strtoul(entry[4].c_str(), nullptr, 10),
                (unsigned char)strtoul(entry[5].c_str(), nullptr, 10),
                0};
            entry = configXML->entry(configXML->busDevice("ECAT", type.c_str()), "ClearError");
            clearErrorSDO = SDOMsg{
                sdoHandler,
                1,
                alias,
                0,
                (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
                (unsigned char)strtoul(entry[2].c_str(), nullptr, 16),
                (unsigned char)strtoul(entry[3].c_str(), nullptr, 10),
                (unsigned char)strtoul(entry[4].c_str(), nullptr, 10),
                (unsigned char)strtoul(entry[5].c_str(), nullptr, 10),
                0};
        }
        else if (bus == "CAN")
        {
            ;
        }
        else if (bus == "RS485")
        {
            ;
        }
        return 0;
    }

    /**
     * @brief MotorParameters析构函数
     * 
     * 目前为空实现，无需特殊清理操作
     */
    MotorParameters::~MotorParameters()
    {
    }

    /**
     * @brief EffectorParameters构造函数
     * 
     * 目前为空实现，用于初始化从站参数对象
     */
    EffectorParameters::EffectorParameters()
    {
    }

    /**
     * @brief 从配置加载从站参数
     * @param bus 总线类型（"ECAT"、"CAN"或"RS485"）
     * @param alias 从站别名/ID
     * @param type 从站类型
     * @param sdoHandler SDO处理句柄（用于ECAT总线）
     * @return 目前总是返回0（成功）
     * 
     * 预留的从站参数加载函数，目前为空实现
     */
    int EffectorParameters::load(std::string const &bus, int const alias, std::string const &type, ec_sdo_request_t *const sdoHandler)
    {
        return 0;
    }

    /**
     * @brief EffectorParameters析构函数
     * 
     * 目前为空实现，无需特殊清理操作
     */
    EffectorParameters::~EffectorParameters()
    {
    }

    /**
     * @brief SensorParameters构造函数
     * 
     * 目前为空实现，用于初始化传感器参数对象
     */
    SensorParameters::SensorParameters()
    {
    }

    /**
     * @brief 从配置加载传感器参数
     * @param bus 总线类型（"ECAT"、"CAN"或"RS485"）
     * @param alias 传感器别名/ID
     * @param type 传感器类型
     * @param sdoHandler SDO处理句柄（用于ECAT总线）
     * @return 目前总是返回0（成功）
     * 
     * 预留的传感器参数加载函数，目前为空实现
     */
    int SensorParameters::load(std::string const &bus, int const alias, std::string const &type, ec_sdo_request_t *const sdoHandler)
    {
        return 0;
    }

    /**
     * @brief SensorParameters析构函数
     * 
     * 目前为空实现，无需特殊清理操作
     */
    SensorParameters::~SensorParameters()
    {
    }
}