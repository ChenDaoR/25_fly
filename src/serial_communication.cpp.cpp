#include <ros/ros.h>
#include <serial/serial.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <flight_ctrl/SetFlightTask.h>


    ros::NodeHandle n;


    ros::ServiceClient set_mode_client_   = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient arming_client_     = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_task_client_   = n.serviceClient<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask");
 
 
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);

    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen()) ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    else    return -1;
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, 1);
        }

        switch(n)
        {
            case 'o':set_mode("OFFBOARD");  break;
            case 'a':set_arm(true); break;
            case 'd':set_arm(false); break;
            case 'S':set_flight_task("Standby");    break;
            case 'T':set_flight_task("Takeoff");    break;
            case 'M':set_flight_task("Mission");    break;
            case 'L':set_flight_task("Land");   break;
            case 'q':ros::shutdown();   break;
        }

        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}
