#include "FlightCtrl.hpp"

FlightCtrl::FlightCtrl(const ros::NodeHandle& nh_,const ros::NodeHandle& nh_private_)
    : nh(nh_),nh_private(nh_private_),CmdRate(150),StatusRate(1)
{
    ROS_INFO("[FlightCtrl] Constructor called with default rates");  // 新增
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",50,&FlightCtrl::state_Callback,this,ros::TransportHints().tcpNoDelay());
    reference_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",50,&FlightCtrl::reference_position_Callback,this,ros::TransportHints().tcpNoDelay());
    set_FlightTask_sub = nh.subscribe<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask",10,&FlightCtrl::set_FlightTask_Callback,this,ros::TransportHints().tcpNoDelay());

    target_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    target_velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    arming_request_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_change_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/setmode");

    cmdloop_timer = nh.createTimer(ros::Duration(1.00/150.0),&FlightCtrl::cmdloop_Callback,this);
    statusloop_timer = nh.createTimer(ros::Duration(1.00/1.0),&FlightCtrl::statusloop_Callback,this);

    last_request = ros::Time::now();
    last_task = ros::Time::now();

    state = STATE_UNINIT;
    FT = Standby;
    cache_position.pose.position.x = 0;
    cache_position.pose.position.y = 0;
    cache_position.pose.position.z = 0;
    ROS_INFO("[FlightCtrl] Initialization complete, current state: UNINIT, FlightTask: Standby");  // 新增
}

FlightCtrl::FlightCtrl(const ros::NodeHandle& nh_,const ros::NodeHandle& nh_private_,double CmdRate_,double StatusRate_)
    : nh(nh_),nh_private(nh_private_),CmdRate(CmdRate_),StatusRate(StatusRate_)
{
    ROS_INFO("[FlightCtrl] Constructor called with custom rates: CmdRate=%d, StatusRate=%d", CmdRate_, StatusRate_);  // 新增
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",50,&FlightCtrl::state_Callback,this,ros::TransportHints().tcpNoDelay());
    reference_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",50,&FlightCtrl::reference_position_Callback,this,ros::TransportHints().tcpNoDelay());
    set_FlightTask_sub = nh.subscribe<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask",10,&FlightCtrl::set_FlightTask_Callback,this,ros::TransportHints().tcpNoDelay());

    target_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    target_velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    arming_request_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_change_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/setmode");

    cmdloop_timer = nh.createTimer(ros::Duration(1.0/CmdRate),&FlightCtrl::cmdloop_Callback,this);
    statusloop_timer = nh.createTimer(ros::Duration(1.0/StatusRate),&FlightCtrl::statusloop_Callback,this);

    last_request = ros::Time::now();
    last_task = ros::Time::now();

    state = STATE_UNINIT;
    FT = Standby;
    cache_position.pose.position.x = 0;
    cache_position.pose.position.y = 0;
    cache_position.pose.position.z = 0;
    ROS_INFO("[FlightCtrl] Initialization complete with custom rates, current state: UNINIT, FlightTask: Standby");  // 新增
}

void FlightCtrl::state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    ROS_INFO("[StateCallback] mavros/state updated: mode=%s, armed=%d", msg->mode.c_str(), msg->armed);  // 新增
    state_cb = *msg;
}

void FlightCtrl::reference_position_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "[ReferenceCallback] local_position/pose updated: x=%.2f, y=%.2f, z=%.2f", 
                      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);  // 新增（限制频率）
    reference_position_cb = *msg;
}

void FlightCtrl::set_FlightTask_Callback(const flight_ctrl::SetFlightTask::ConstPtr& msg)
{
    ROS_INFO("[SetFlightTaskCallback] Received new FlightTask: %s", msg->FlightTask.c_str());  // 新增
    s = *msg;
    if(s.FlightTask == "Standby")
    {
        FT = Standby;
        ROS_INFO("[SetFlightTaskCallback] FlightTask changed to Standby");  // 新增
    }
    else if(s.FlightTask == "Takeoff")
    {
        FT = Takeoff;
        ROS_INFO("[SetFlightTaskCallback] FlightTask changed to Takeoff");  // 新增
    }
    else if(s.FlightTask == "Mission")
    {
        FT = Mission;
        ROS_INFO("[SetFlightTaskCallback] FlightTask changed to Mission");  // 新增
    }
    else if(s.FlightTask == "Land")
    {
        FT = Land;
        ROS_INFO("[SetFlightTaskCallback] FlightTask changed to Land");  // 新增
    }
}

void FlightCtrl::cmdloop_Callback(const ros::TimerEvent& event)
{
    ROS_INFO_THROTTLE(5.0, "[CmdLoop] Running cmdloop at %.1f Hz", CmdRate);  // 新增（限制频率）

    switch(FT)
    {
        case Standby: 
            ROS_INFO_ONCE("[CmdLoop] Switching to Standby task");  // 新增（仅首次触发）
            cmd_Task_Standby(); 
            break;
        case Takeoff: 
            ROS_INFO_ONCE("[CmdLoop] Switching to Takeoff task");  // 新增
            cmd_Task_Takeoff(1.5); 
            break;
        case Mission: 
            ROS_INFO_ONCE("[CmdLoop] Switching to Mission task");  // 新增
            cmd_Task_Mission(); 
            break;
        case Land: 
            ROS_INFO_ONCE("[CmdLoop] Switching to Land task");  // 新增
            cmd_Task_Land(); 
            break;
    }
}

void FlightCtrl::statusloop_Callback(const ros::TimerEvent& event)
{
    ROS_INFO_THROTTLE(5.0, "[StatusLoop] Running statusloop at %.1f Hz", StatusRate);  // 新增

    mavros_msgs::SetMode mode_change_msg;
    mode_change_msg.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arming_request_msgs;
    arming_request_msgs.request.value = true;

    if((state_cb.mode != "OFFBOARD") && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
        if(mode_change_client.call(mode_change_msg) && mode_change_msg.response.mode_sent)
        {
            ROS_INFO("[StatusLoop] Offboard Enabled");  // 新增
        }
        last_request = ros::Time::now();
    }
    else
    {
        if(!state_cb.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if(arming_request_client.call(arming_request_msgs) && arming_request_msgs.response.success)
            {
                ROS_INFO("[StatusLoop] Armed");  // 新增
            }
            last_request = ros::Time::now();
        }
    }
}

void FlightCtrl::cmd_Task_Standby()
{
    cache_position.pose.position.x = reference_position_cb.pose.position.x;
    cache_position.pose.position.y = reference_position_cb.pose.position.y;
    cache_position.pose.position.z = reference_position_cb.pose.position.z;
    ROS_INFO_THROTTLE(1.0, "[Standby] Publishing target position: x=%.2f, y=%.2f, z=%.2f", 
                      cache_position.pose.position.x, 
                      cache_position.pose.position.y, 
                      cache_position.pose.position.z);  // 新增
    target_position_pub.publish(cache_position);
}

void FlightCtrl::cmd_Task_Land()
{
    cache_position.pose.position.z += 1;
    ROS_INFO("[Land] Landing target updated: z=%.2f", cache_position.pose.position.z);  // 新增
    target_position_pub.publish(cache_position);
    ROS_INFO("[Land] Stopping cmdloop_timer");  // 新增
    cmdloop_timer.stop();
}

void FlightCtrl::cmd_Task_Mission()
{
    ROS_INFO_THROTTLE(2.0, "[Mission] Mission task placeholder (no implementation yet)");  // 新增
}

void FlightCtrl::cmd_Task_Takeoff(double h)
{
    cache_position.pose.position.x = 0;
    cache_position.pose.position.y = 0;
    cache_position.pose.position.z = h;
    ROS_INFO("[Takeoff] Publishing takeoff target: x=%.2f, y=%.2f, z=%.2f", 
             cache_position.pose.position.x, 
             cache_position.pose.position.y, 
             cache_position.pose.position.z);  // 新增
    target_position_pub.publish(cache_position);
}