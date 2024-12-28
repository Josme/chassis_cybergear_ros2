#include <iostream>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
class BatteryManager : public rclcpp::Node {
public:
    BatteryManager()
        :Node("battery_publisher"), smoothedCharge_(100.0f) {
                    this->declare_parameter("min_voltage", minVoltage_);
        this->declare_parameter("max_voltage", maxVoltage_);

        // 获取初始参数值
        this->get_parameter("min_voltage", minVoltage_);
        this->get_parameter("max_voltage", maxVoltage_);
              battery_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "/battery_data", 20);

        }

    // 更新当前电压值，返回平滑后的电量
    void update(float currentVoltage) {
        // 根据电压计算当前电量百分比
        float rawCharge = calculateRawCharge(currentVoltage);
        currentVoltage_ = currentVoltage;

        // 确保电量只减不增（允许5%的误差）
        if (rawCharge < smoothedCharge_ || rawCharge > smoothedCharge_ + 5.0f) {
            smoothedCharge_ = rawCharge;
        }
        publishBatteryState();
    }

private:
    float minVoltage_;  // 6S 电池最低电压
    float maxVoltage_;  // 6S 电池最高电压
    float smoothedCharge_; // 平滑后的电量百分比
    float currentVoltage_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;

    // 根据当前电压计算原始电量百分比
    float calculateRawCharge(float voltage) const {
        // 将电压值映射到 0-100% 的范围
        float charge = (voltage - minVoltage_) / (maxVoltage_ - minVoltage_) * 100.0f;
        // 确保电量在 0%-100% 范围内
        return std::clamp(charge, 0.0f, 100.0f);
    }
    void publishBatteryState() {
        auto message = sensor_msgs::msg::BatteryState();

        // 设置消息数据
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "battery_frame";
        message.voltage = currentVoltage_;                      // 设置电压
        message.percentage = smoothedCharge_; // 设置电量百分比
        message.present = true;                        // 电池是否存在

        // 发布消息
        battery_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published BatteryState: Voltage=%.2fV, Percentage=%.2f%%", 
                    message.voltage, message.percentage);
    }
};
