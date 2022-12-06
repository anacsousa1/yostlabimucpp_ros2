#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "threespace_api_export.h"

#pragma comment(lib, "Setupapi.lib")
#pragma comment(lib, "BluetoothAPIs.lib")

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

int connectDongle(TSS_ComPort port, tss_device_id dongle_id){
    TSS_ERROR error = TSS_NO_ERROR;

    port.port_name = new char[64];
    tss_findSensorPorts(TSS_DONGLE);

    error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);

	if (error == TSS_NO_ERROR){
		error = tss_createDongle(port.port_name, &dongle_id);
        if (error){
            tss_deinitAPI();
            return 0;
        }
	}else{
		tss_deinitAPI();
		return 0;
	}
    return 1;
}

int connectSensor(tss_device_id dongle_id, tss_device_id device_id, U8 logical_id, U32 serial_number){
    TSS_ERROR error = TSS_NO_ERROR;

    error = tss_dongle_setSerialNumberAtLogicalID(dongle_id, logical_id, serial_number, NULL);
	
    if (error == TSS_NO_ERROR){
		error = tss_dongle_getWirelessSensor(dongle_id, logical_id, &device_id);
        if (error){
            tss_deinitAPI();
            return 0;
        }
	}else{ // failed to set serial number at logical ID
		tss_deinitAPI();
		return 0;
	}
    return 1;
}

class YostLabsIMU : public rclcpp::Node{
    public: YostLabsIMU() : Node("yost_labs_imu"), count_(0), countBattery_(0){
        int frequency{0}, frequencyBattery{0};

        this->declare_parameter("frequency", 200);
        this->declare_parameter("frequencyBattery", 1);

        this->get_parameter("frequency", frequency);
        this->get_parameter("frequencyBattery", frequencyBattery);

        auto sampleTime = std::chrono::milliseconds(static_cast<int>(1000 / frequency));
        auto sampleTimeBattery = std::chrono::milliseconds(static_cast<int>(1000 / frequencyBattery));

        if(connectDongle(port_,dongle_id_)){ // Connect the dongle
            if(connectSensor(dongle_id_, device_id_, logical_id_, serial_number_)){ // Connect one IMU
                // Create publishers, subscribers and timers
                anglePub_ = this->create_publisher<std_msgs::msg::String>("/data/incomplete/bike_feedback", 10);
                batteryPub_ = this->create_publisher<std_msgs::msg::String>("/data/yostlabsIMU/battery", 10);

                timerCrankAngle_ = this->create_wall_timer(sampleTime, std::bind(&YostLabsIMU::pubCrankAngle, this));
                timerBattery_ = this->create_wall_timer(sampleTimeBattery, std::bind(&YostLabsIMU::pubBattery, this));
            }else{ // err sensor
                if(debug_) RCLCPP_INFO(this->get_logger(), "Failed to create TSS Sensor on port %s, logical ID %d, and serial number %d", port_.port_name, logical_id_, serial_number_);
            }
        }else{ // err dongle
            if(debug_) RCLCPP_INFO(this->get_logger(), "Failed to create TSS Dongle on port %s", port_.port_name);
        }
    }

private:
    void pubCrankAngle(){
        TSS_ERROR error = TSS_NO_ERROR;
        float quat[4];

        // get quat
        error = tss_sensor_getTaredOrientationAsQuaternion(device_id_, quat, NULL);
        // if (!error){
        //     if(debug_) RCLCPP_INFO(this->get_logger(), "Quaternion: %f, %f, %f, %f ", quat[0], quat[1], quat[2], quat[3]);
        // }else{
        //     if(debug_) RCLCPP_INFO(this->get_logger(), "TSS_Error: %s", tss_error_string[error]);
        // }

        // get corrected component sensor data
        // float gyro[3], accel[3], compass[3];
        // error = tss_sensor_getCorrectedSensorData(device_id_, gyro, accel, compass, NULL);
        // if (!error){
        //     if(debug_) RCLCPP_INFO(this->get_logger(), "Gyro: %f, %f, %f | Accel: %f, %f, %f | Comp:  %f, %f, %f", gyro[0], gyro[1], gyro[2], quat[3], accel[0], accel[1], accel[2], compass[0], compass[1], compass[2]);
        // }else{
        //     if(debug_) RCLCPP_INFO(this->get_logger(), "TSS_Error: %s", tss_error_string[error]);
        // }


        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);

        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        anglePub_->publish(message);
    }
    
    void pubBattery(){
        TSS_ERROR error = TSS_NO_ERROR;
        U8 battery_percent;

        error = tss_sensor_getBatteryPercentRemaining(device_id_,&battery_percent,NULL);
        if (!error){
            if(debug_) RCLCPP_INFO(this->get_logger(), "Battery: %d%%", battery_percent);
        }else{
            if(debug_) RCLCPP_INFO(this->get_logger(), "TSS_Error: %s", tss_error_string[error]);
        }			


        auto msgBattery = std_msgs::msg::String();
        msgBattery.data = "Battery!" + std::to_string(countBattery_++);

        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msgBattery.data.c_str());
        batteryPub_->publish(msgBattery);
    }


	TSS_ComPort port_;	
	tss_device_id dongle_id_;
	tss_device_id device_id_;
	U8 logical_id_{0};
	U32 serial_number_{301993563};

    rclcpp::TimerBase::SharedPtr timerCrankAngle_, timerBattery_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anglePub_,batteryPub_;
    size_t count_,countBattery_;
    bool debug_{true};
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YostLabsIMU>());
    rclcpp::shutdown();
    return 0;
}