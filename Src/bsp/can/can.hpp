#pragma once

#include "bsp/dwt/dwt.h"
#include "fdcan.h"
#include "tool/daemon/daemon.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>

namespace bsp {
struct can_params {
    FDCAN_HandleTypeDef* can_handle = nullptr; // can句柄
    uint32_t tx_id                  = 0;       // 发送id
    uint32_t rx_id                  = 0;       // 接收id
    std::function<void()> callback  = nullptr;
};
class can {
public:
    explicit can(const can_params& params)
        : can_handle_(params.can_handle)
        , tx_id_(params.tx_id)
        , rx_id_(params.rx_id)
        , callback_(params.callback) {
        DuplicateDetect(this);
        Init();
        register_instance(this);
        if (can_instance_count_ == 1) {
            ServiceInit();
        }
    }
    ~can() { unregister_instance(this); }

    uint8_t Transmit(uint8_t* tx_buff, uint8_t tx_size = 8, float timeout = 1.0f) {
        auto data_length = txconf_.DataLength;
        if (data_length > 0xff)
            data_length >>= 16;
        if (tx_size != data_length) {
            return 0;                          // 错误：缓冲区长度不匹配
        }
        static uint32_t busy_count;
        // static volatile float wait_time __attribute__((unused));         // for cancel warning
        float dwt_start = DWT_GetTimeline_ms();
        while (HAL_FDCAN_GetTxFifoFreeLevel(can_handle_) == 0) // 等待邮箱空闲
        {
            if (DWT_GetTimeline_ms() - dwt_start > timeout)    // 超时
            {
                busy_count++;
                if (busy_count > 300) {
                    HAL_FDCAN_Stop(can_handle_);
                    HAL_FDCAN_AbortTxRequest(can_handle_, FDCAN_TX_BUFFER0);
                    HAL_FDCAN_AbortTxRequest(can_handle_, FDCAN_TX_BUFFER1);
                    HAL_FDCAN_AbortTxRequest(can_handle_, FDCAN_TX_BUFFER2);
                    HAL_FDCAN_Start(can_handle_);
                }
                return 0;
            }
        }
        // wait_time = DWT_GetTimeline_ms() - dwt_start;
        // tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
        if (HAL_FDCAN_AddMessageToTxFifoQ(can_handle_, &txconf_, tx_buff)) {
            // busy_count++;
            return 0;
        }
        return 1; // 发送成功
    }
    void SetDLC(uint8_t length) {
        // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
        if (length > 8 || length == 0) // 安全检查
            while (true)
                ;
        txconf_.DataLength = length;
    }
    void OnRxFifoCallback(uint8_t* data, uint32_t length) {
        rx_len_ = length;
        std::copy(data, data + length, rx_buff_);
        daemon_.reload();
        if (callback_)
            callback_();
    }
    void SetCallback(const std::function<void()>& callback) { callback_ = callback; }
    void SetOfflineCallback(const std::function<void()>& callback) {
        daemon_.set_callback(callback);
    }
    static can* get_instance(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_id) {
        for (auto* instance : can_instances_) {
            if (instance && instance->can_handle_ == hfdcan && instance->rx_id_ == rx_id) {
                return instance;
            }
        }
        return nullptr;
    }
    void GetRxData(uint8_t* data) { std::copy(rx_buff_, rx_buff_ + rx_len_, data); }
    [[nodiscard]] uint32_t GetRxId() const { return rx_id_; }

private:
    FDCAN_HandleTypeDef* can_handle_;  // can句柄
    FDCAN_TxHeaderTypeDef txconf_;     // CAN报文发送配置
    uint32_t tx_id_;                   // 发送id
    // uint32_t tx_mailbox_;              // CAN消息填入的邮箱号
    // uint8_t tx_buff_[8]; // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff_[8];                                  // 接收缓存,最大消息长度为8
    uint32_t rx_id_;                                      // 接收id
    uint8_t rx_len_;                                      // 接收长度,可能为0-8
    std::function<void()> callback_;
    tool::daemon daemon_ = tool::daemon(0.1);

    void Init() {
        txconf_.Identifier          = tx_id_;             // 发送id
        txconf_.IdType              = FDCAN_STANDARD_ID;  // 标准ID
        txconf_.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf_.DataLength          = FDCAN_DLC_BYTES_8;  // 发送数据长度
        txconf_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 设置错误状态指示
        txconf_.BitRateSwitch       = FDCAN_BRS_OFF;      // 不开启可变波特率
        txconf_.FDFormat            = FDCAN_CLASSIC_CAN;  // 普通CAN格式
        txconf_.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
        txconf_.MessageMarker       = 0x00;
        AddFilter(this);                                  // 添加CAN过滤器规则
    }

    /* ----------------two static function called by CANRegister()-------------------- */
    /**
     * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
     *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
     * @note
     * h7的bxCAN有128个过滤器,这里将其配置为前14个过滤器给CAN1使用,中14个被CAN2使用，后14个被CAN3使用
     *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
     */
    static void AddFilter(can* _instance) {
        FDCAN_FilterTypeDef can_filter_conf;
        static uint8_t can1_filter_idx = 0, can2_filter_idx = 14,
                       can3_filter_idx = 28; // 0-13给can1用,14-27给can2用,28-41给can3用

        can_filter_conf.IdType = FDCAN_STANDARD_ID;                             // 标准ID
        can_filter_conf.FilterIndex =
            (_instance->can_handle_ == &hfdcan1)
                ? (can1_filter_idx++)
                : ((_instance->can_handle_ == &hfdcan2) ? (can2_filter_idx++)
                                                        : (can3_filter_idx++)); // 滤波器索引
        can_filter_conf.FilterType =
            FDCAN_FILTER_DUAL; // 允许接收两个ID TODO: 后续可以优化使其能充分利用第二个ID位置
        can_filter_conf.FilterConfig = (_instance->rx_id_ & 1)
                                         ? FDCAN_FILTER_TO_RXFIFO0
                                         : FDCAN_FILTER_TO_RXFIFO1; // 过滤器0关联到FIFO0
        can_filter_conf.FilterID1    = 0x000;                       // 32位ID接收ID1
        can_filter_conf.FilterID2    = _instance->rx_id_;           // 接收ID2
        HAL_FDCAN_ConfigFilter(_instance->can_handle_, &can_filter_conf);
    }
    /**
     * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
     * @note 此函数会启动CAN1和CAN2、CAN3,开启CAN1和CAN2、CAN3的FIFO0 & FIFO1溢出通知
     */
    static void ServiceInit() {
        HAL_FDCAN_Start(&hfdcan1);
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_Start(&hfdcan2);
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_Start(&hfdcan3);
        HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

        HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
        HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
        HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
        HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
        HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
        HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO1, 1);
    }

    static constexpr size_t MAX_CAN_INSTANCES = 3 * 8;
    static std::array<can*, MAX_CAN_INSTANCES> can_instances_;
    static size_t can_instance_count_;

    static void register_instance(can* instance) {
        auto it = std::find(can_instances_.begin(), can_instances_.end(), nullptr);
        if (it != can_instances_.end()) {
            *it = instance;
            can_instance_count_++;
        }
    }
    static void unregister_instance(can* instance) {
        auto it = std::find(can_instances_.begin(), can_instances_.end(), instance);
        if (it != can_instances_.end()) {
            *it = nullptr;
            can_instance_count_--;
        }
    }
    static void DuplicateDetect(can* instance) {
        for (auto* i : can_instances_) {
            if (i != instance && instance->rx_id_ != 0 && i->rx_id_ == instance->rx_id_) {
                while (true)
                    ; // error: duplicate can id
            }
        }
    }
    friend void CANFIFOxCallback(FDCAN_HandleTypeDef* _hfdcan, uint32_t fifox);
};
} // namespace bsp