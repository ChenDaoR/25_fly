/*********************************************************************
 *  serial_bridge.cpp
 *  通过 rosserial 接收串口指令并调用 mavros / FlightCtrl 服务
 *********************************************************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <flight_ctrl/SetFlightTask.h>

class SerialBridge
{
public:
    SerialBridge()
    {
        /* 串口参数 */
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        sp_.setPort("/dev/ttyUSB0");
        sp_.setBaudrate(1152000);
        sp_.setTimeout(to);

        try
        {
            sp_.open();
            ROS_INFO("串口 /dev/ttyUSB0 打开成功 @ 1152000");
        }
        catch (const serial::IOException& e)
        {
            ROS_ERROR("无法打开串口: %s", e.what());
            ros::shutdown();
        }

        /* 客户端 */
        set_mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        arming_client_     = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_task_client_   = nh_.serviceClient<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask");

        timer_ = nh_.createTimer(ros::Duration(0.02),
                                 &SerialBridge::serialPollCb, this);
    }

    ~SerialBridge()
    {
        if (sp_.isOpen()) sp_.close();
    }

private:
    /* 轮询串口 */
    void serialPollCb(const ros::TimerEvent&)
    {
        if (!sp_.available()) return;

        std::string buf;
        sp_.read(buf, 1);          // 只读 1 字节
        char c = buf[0];

        switch (c)
        {
            case 'o':
                ROS_INFO("[串口] 收到 'o' -> 切换 OFFBOARD");
                set_mode("OFFBOARD");
                break;

            case 'a':
                ROS_INFO("[串口] 收到 'a' -> ARM");
                set_arm(true);
                break;

            case 'd':
                ROS_INFO("[串口] 收到 'd' -> DISARM");
                set_arm(false);
                break;

            case 'S':
                ROS_INFO("[串口] 收到 'S' -> Standby");
                set_flight_task("Standby");
                break;

            case 'T':
                ROS_INFO("[串口] 收到 'T' -> Takeoff");
                set_flight_task("Takeoff");
                break;

            case 'M':
                ROS_INFO("[串口] 收到 'M' -> Mission");
                set_flight_task("Mission");
                break;

            case 'L':
                ROS_INFO("[串口] 收到 'L' -> Land");
                set_flight_task("Land");
                break;

            case 'q':
                ROS_INFO("[串口] 收到 'q' -> 请求退出");
                ros::shutdown();
                break;

            default:
                ROS_WARN("[串口] 未定义字符: 0x%02X", c);
                break;
        }
    }

    /* 下面 3 个函数与 FlightPanel.cpp 中同名函数完全一致 */
    void set_mode(const std::string& mode)
    {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = mode;
        if (set_mode_client_.call(srv) && srv.response.mode_sent)
            ROS_INFO("Mode set to %s", mode.c_str());
        else
            ROS_WARN("Failed to set mode %s", mode.c_str());
    }

    void set_arm(bool arm)
    {
        mavros_msgs::CommandBool srv;
        srv.request.value = arm;
        if (arming_client_.call(srv) && srv.response.success)
            ROS_INFO("Arming: %s", arm ? "ARMED" : "DISARMED");
        else
            ROS_WARN("Arming call failed");
    }

    void set_flight_task(const std::string& task)
    {
        flight_ctrl::SetFlightTask srv;
        srv.request.value = task;
        if (set_task_client_.call(srv) && srv.response.success)
            ROS_INFO("Flight task set to %s", task.c_str());
        else
            ROS_WARN("Failed to set flight task %s", task.c_str());
    }

    /* 成员 */
    ros::NodeHandle nh_;
    serial::Serial sp_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_task_client_;
    ros::Timer timer_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_bridge");
    SerialBridge sb;
    ros::spin();
    return 0;
}