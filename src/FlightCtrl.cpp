#include "FlightCtrl.hpp"

#define MISSION_1_NUM 6    //任务点数目
#define TAKEOFF_HEIGHT 1.00   //起飞高度
#define POS_ERR 0.05   //位置误差
#define YAW_ERR 0.5  //旋转误差
#define CTRLER 1   //控制器


FlightCtrl::FlightCtrl(const ros::NodeHandle& nh_,const ros::NodeHandle& nh_private_)
    : nh(nh_),nh_private(nh_private_),CmdRate(150),StatusRate(1),rate_count(0),rate(150),land_switch(false),land_switch_(false),initial_flag(false)
{
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",50,&FlightCtrl::state_Callback,this,ros::TransportHints().tcpNoDelay());
    reference_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",50,&FlightCtrl::reference_position_Callback,this,ros::TransportHints().tcpNoDelay());
    set_FlightTask_Topic_sub = nh.subscribe<flight_ctrl::SetFlightTask_Topic>("Flight/SetFlightTask_Topic",10,&FlightCtrl::set_FlightTask_Topic_Callback,this,ros::TransportHints().tcpNoDelay());
    vins_position_sub = nh.subscribe<nav_msgs::Odometry>("vins_estimator/odometry",10,&FlightCtrl::vins_position_Callback,this,ros::TransportHints().tcpNoDelay());
    
    target_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    target_velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    set_FlightTask_Topic_pub = nh.advertise<flight_ctrl::SetFlightTask_Topic>("Flight/SetFlightTask_Topic",10);
    
    arming_request_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_change_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    set_FlightTask_server = nh.advertiseService("FlightCtrl/SetFlightTask",&FlightCtrl::set_FlightTask_Callback,this);
    set_FlightTask_client = nh.serviceClient<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask");


    // cmdloop_timer = nh.createTimer(ros::Duration(1.0/CmdRate),&FlightCtrl::cmdloop_Callback,this);
    // statusloop_timer = nh.createTimer(ros::Duration(1.0/StatusRate),&FlightCtrl::statusloop_Callback,this);
    //statusloop_timer = nh.createTimer(ros::Duration(0.1),&FlightCtrl::trigger_Callback,this);

    mode_change_msg.request.custom_mode = "OFFBOARD";
    arming_request_msgs.request.value = true;

    state = STATE_UNINIT;
    FT = Standby;
    cache_position.pose.position.x = 0;
    cache_position.pose.position.y = 0;
    cache_position.pose.position.z = 0;

    last_request = ros::Time::now();
    last_mission = ros::Time::now();

    ctrler = CTRLER;
    mission_num = MISSION_1_NUM;
    pos_err = POS_ERR;
    yaw_err = YAW_ERR;

    cx.setDt(1.0/CmdRate);
    cy.setDt(1.0/CmdRate);
    cz.setDt(1.0/CmdRate);
    c_yaw.setDt(1.0/CmdRate);

    cx.setGains(1, 0, 0);
    cy.setGains(1, 0, 0);
    cz.setGains(1.35, 0, 0);
    c_yaw.setGains(0.7, 0, 0);

    cx.setOutputLimit(0.3,0);
    cy.setOutputLimit(0.3,0);
    cz.setOutputLimit(1.2,0);
    c_yaw.setOutputLimit(1.2,0);

}


FlightCtrl::~FlightCtrl()
{
    delete ro_matrix;
    // cmdloop_timer.stop();
    // statusloop_timer.stop();
}

void FlightCtrl::state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    state_cb = *msg;
}

void FlightCtrl::reference_position_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    reference_position_cb = *msg;

    if(!initial_flag)
    {
        initial_pose = reference_position_cb;
        initial_yaw = tf2::getYaw(initial_pose.pose.orientation);
        ro_matrix = new Eigen::Rotation2Dd(initial_yaw);
        initial_flag = true;
    }
} 

void FlightCtrl::vins_position_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    vins_position_cb = *msg;

        Eigen::Vector2d ref_pos(
        reference_position_cb.pose.position.x,
        reference_position_cb.pose.position.y
    );

    Eigen::Vector2d vins_pos(
        vins_position_cb.pose.pose.position.x,
        vins_position_cb.pose.pose.position.y
    );
    vins_err = (ref_pos - vins_pos).norm();
} 

bool FlightCtrl::set_FlightTask_Callback(flight_ctrl::SetFlightTask::Request& req,flight_ctrl::SetFlightTask::Response& res)
{
    s = req;
    if(s.value == "Standby")
    {
        ROS_INFO("now flight_task is Standby");
        FT = Standby;
        land_switch_ = false;
        res.success = true;
        cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
        cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
        cache_position.pose.position.z = round_(reference_position_cb.pose.position.z);
        cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
        cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
        cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
        cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
        
    }
    else if(s.value == "Takeoff")
    {
        ROS_INFO("now flight_task is Takeoff");
        FT = Takeoff;
        res.success = true;
        land_switch_ = false;
    }
    else if(s.value == "Mission")
    {
        ROS_INFO("now flight_task is Mission");
        FT = Mission;
        res.success = true;
        land_switch_ = false;
        last_mission = ros::Time::now();
    }
    else if(s.value == "Land")
    {
        ROS_INFO("now flight_task is Land");
        FT = Land;
        res.success = true;
        land_switch = false;
    }
    else
    {
        res.success = false;
    }

    return true;
}

void FlightCtrl::set_FlightTask_Topic_Callback(const flight_ctrl::SetFlightTask_Topic::ConstPtr& msg)
{
    s_Topic = *msg;
    if(s.value == "Standby")
    {
        ROS_INFO("now flight_task is Standby");
        FT = Standby;
        land_switch_ = false;
        
    }
    else if(s.value == "Takeoff")
    {
        ROS_INFO("now flight_task is Takeoff");
        FT = Takeoff;
        land_switch_ = false;
    }
    else if(s.value == "Mission")
    {
        ROS_INFO("now flight_task is Mission");
        FT = Mission;
        land_switch_ = false;
        last_mission = ros::Time::now();
    }
    else if(s.value == "Land")
    {
        ROS_INFO("now flight_task is Land");
        FT = Land;
        land_switch_ = false;
    }
}

void FlightCtrl::main_loop()
{
    ROS_INFO("waiting for imu");
    while(ros::ok() && !state_cb.connected)
    {
        ros::spinOnce();
    }
    ROS_INFO("fcu has connected");

    while(ros::ok())
    {
        ros::spinOnce();
        if(!land_switch_)    cmdloop_Callback();
        if(rate_count == 150)
        {
            rate_count = 0;
            statusloop_Callback();
        } 
        rate_count++;
        rate.sleep();
    }
}

void FlightCtrl::cmdloop_Callback()
{

    switch(FT)
    {
        case Standby: cmd_Task_Standby(); break;
        case Takeoff: cmd_Task_Takeoff(TAKEOFF_HEIGHT); break;
        case Mission: cmd_Task_Mission(); break;
        case Land: cmd_Task_Land(); break;
    }
    
}

double FlightCtrl::round_(double x)
{
    return std::round(x * 10.0) / 10.0;
}

void FlightCtrl::statusloop_Callback()
{

    if(state_cb.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(3.0))
    {
        ROS_INFO("trying to switch to OFFBOARD");
        if(mode_change_client.call(mode_change_msg) && mode_change_msg.response.mode_sent)
        {
            ROS_INFO("Offboard Enabled");
        }
        else ROS_INFO("Offboard Failed : %d",mode_change_msg.response.mode_sent);
        last_request = ros::Time::now();
    }
    else if(!state_cb.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
        ROS_INFO("trying to switch to Arm");
        if(arming_request_client.call(arming_request_msgs) && arming_request_msgs.response.success)
        {
            ROS_INFO("Armed");
        }
        else ROS_INFO("Arm Failed");
        last_request = ros::Time::now();
    }

    // ROS_INFO("Now Position: x: %.3f , y: %.3f , z: %.3f \n"
    //             ,reference_position_cb.pose.position.x,reference_position_cb.pose.position.y,reference_position_cb.pose.position.z);
    // 当前ENU坐标
    Eigen::Vector2d enu_pos(
    reference_position_cb.pose.position.x,
    reference_position_cb.pose.position.y
    );

    Eigen::Vector2d initial_pose_(
    initial_pose.pose.position.x,
    initial_pose.pose.position.y
    );

    // 旋转矩阵（ENU -> 任务坐标系）
    Eigen::Rotation2Dd rot(-initial_yaw);
    Eigen::Vector2d task_xy = rot * Eigen::Vector2d(enu_pos.x(), enu_pos.y()) - rot * Eigen::Vector2d(initial_pose_.x(), initial_pose_.y());

    // 输出任务坐标系下的坐标
    ROS_INFO("Task Frame Position: x: %.3f , y: %.3f , z: %.3f , yaw: %.3f \n Vins Frame Position: x: %.3f , y: %.3f , z: %.3f , yaw: %.3f \n err: %.3f",
            task_xy.x(), task_xy.y(), reference_position_cb.pose.position.z,(tf2::getYaw(reference_position_cb.pose.orientation) - initial_yaw) * 180 / M_PI,
            vins_position_cb.pose.pose.position.x,vins_position_cb.pose.pose.position.y,vins_position_cb.pose.pose.position.z,tf2::getYaw(vins_position_cb.pose.pose.orientation) * 180 / M_PI,
            vins_err
        );
    
}

void FlightCtrl::trigger_Callback(const ros::TimerEvent& event)
{

}

void FlightCtrl::fly_to(double x,double y,double z,double yaw,double t,bool is_err)
{
    Eigen::Vector3d target_(x, y, z);
    Eigen::Vector3d curr(reference_position_cb.pose.position.x,reference_position_cb.pose.position.y,reference_position_cb.pose.position.z);

    bool pos_ok = (target_ - curr).norm() < POS_ERR && std::abs(angles::shortest_angular_distance(tf2::getYaw(reference_position_cb.pose.orientation), yaw * M_PI / 180.0)) < YAW_ERR * M_PI / 180.0;
    bool time_ok = ros::Time::now() - last_mission >= ros::Duration(t);

    if((is_err && pos_ok) || time_ok)
    {
        
        if(mission_num-1 <= 0)
        {
            ROS_INFO("Misiion Finished");
            FT = Standby;
            cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
            cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
            cache_position.pose.position.z = round_(reference_position_cb.pose.position.z);
            cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
            cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
            cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
            cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
            mission_num = MISSION_1_NUM;
            return;

        }
        else    mission_num--;
    last_mission = ros::Time::now();
    }

    if(ctrler == 0)
    {
        cache_position.pose.position.x = x;
        cache_position.pose.position.y = y;
        cache_position.pose.position.z = z;
        cache_position.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), (yaw * M_PI / 180.0)).normalized());
        target_position_pub.publish(cache_position);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    ;
    }
    else if(ctrler == 1)
    {

        cx.setTarget(x);
        cy.setTarget(y);
        cz.setTarget(z);

        cache_vel.linear.x = cx.compute(reference_position_cb.pose.position.x);
        cache_vel.linear.y = cy.compute(reference_position_cb.pose.position.y);
        cache_vel.linear.z = cz.compute(reference_position_cb.pose.position.z);
        cache_vel.angular.z = c_yaw.compute(angles::shortest_angular_distance(tf2::getYaw(reference_position_cb.pose.orientation), (yaw * M_PI / 180.0)));
        target_velocity_pub.publish(cache_vel);
    }
}

void FlightCtrl::fly_to_(Eigen::Vector4d target,double t,bool is_err)
{
    Eigen::Vector2d xy = *ro_matrix * Eigen::Vector2d(initial_pose.pose.position.x + target(0),initial_pose.pose.position.y + target(1));

    Eigen::Vector3d target_(xy(0), xy(1), target(2));
    Eigen::Vector3d curr(reference_position_cb.pose.position.x,reference_position_cb.pose.position.y,reference_position_cb.pose.position.z);

    bool pos_ok = (target_ - curr).norm() < POS_ERR && std::abs(angles::shortest_angular_distance(tf2::getYaw(reference_position_cb.pose.orientation), initial_yaw + target(3) * M_PI / 180.0)) < YAW_ERR * M_PI / 180.0;
    bool time_ok = ros::Time::now() - last_mission >= ros::Duration(t);

    if((is_err && pos_ok) || time_ok)
    {
        
        if(mission_num-1 <= 0)
        {
            ROS_INFO("Misiion Finished");
            FT = Standby;
            cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
            cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
            cache_position.pose.position.z = round_(reference_position_cb.pose.position.z);
            cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
            cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
            cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
            cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
            mission_num = MISSION_1_NUM;
            return;

        }
        else    mission_num--;
    last_mission = ros::Time::now();
    }

    if(ctrler == 0)
    {
        cache_position.pose.position.x = xy(0);
        cache_position.pose.position.y = xy(1);
        cache_position.pose.position.z = target(2);
        cache_position.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), (initial_yaw + target(3) * M_PI / 180.0)).normalized());
        target_position_pub.publish(cache_position);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    ;
    }
    else if(ctrler == 1)
    {

        cx.setTarget(xy(0));
        cy.setTarget(xy(1));
        cz.setTarget(target(2));

        cache_vel.linear.x = cx.compute(reference_position_cb.pose.position.x);
        cache_vel.linear.y = cy.compute(reference_position_cb.pose.position.y);
        cache_vel.linear.z = cz.compute(reference_position_cb.pose.position.z);
        cache_vel.angular.z = c_yaw.compute(angles::shortest_angular_distance(tf2::getYaw(reference_position_cb.pose.orientation), (initial_yaw + target(3) * M_PI / 180.0)));
        target_velocity_pub.publish(cache_vel);
    }

}

void FlightCtrl::cmd_Task_Standby()
{
    cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
    cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
    cache_position.pose.position.z = round_(reference_position_cb.pose.position.z);
    cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
    cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
    cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
    cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
    target_position_pub.publish(cache_position);
}

void FlightCtrl::cmd_Task_Land()
{
    if(ctrler == 0)
    {
        cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
        cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
        cache_position.pose.position.z = reference_position_cb.pose.position.z - 0.15;
        // if (reference_position_cb.pose.position.z <= 0.16) 
        // {
        //     cache_position.pose.position.z = reference_position_cb.pose.position.z;
        //     if(!land_switch)    land_time = ros::Time::now();
        //     land_switch = true;
        // }
        cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
        cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
        cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
        cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
        target_position_pub.publish(cache_position);
    }
    else if(ctrler == 1)
    {
        cx.setTarget(round_(reference_position_cb.pose.position.x));
        cy.setTarget(round_(reference_position_cb.pose.position.y));

        cache_vel.linear.x = cx.compute(reference_position_cb.pose.position.x);
        cache_vel.linear.y = cy.compute(reference_position_cb.pose.position.y);
        cache_vel.linear.z = -0.15;
        // if (reference_position_cb.pose.position.z <= 0.16) 
        // {
        //     cache_vel.linear.z = 0;
        //     if(!land_switch)    land_time = ros::Time::now();
        //     land_switch = true;
        // }

        cache_vel.angular.z = 0;

        target_velocity_pub.publish(cache_vel);
    }


    // if(ros::Time::now() - land_time >= ros::Duration(1))
    // {

    //     land_switch_ = true;

    //     arming_request_msgs.request.value = false;
    //     arming_request_client.call(arming_request_msgs);

    //     FT = Standby;
    //     cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
    //     cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
    //     cache_position.pose.position.z = round_(reference_position_cb.pose.position.z);
    //     cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
    //     cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
    //     cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
    //     cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
    //     mission_num = MISSION_1_NUM;


    // }

    if (reference_position_cb.pose.position.z <= 0.15 && !land_switch_)
    {
        land_time = ros::Time::now();
        mode_change_msg.request.custom_mode = "POSITION";
        mode_change_client.call(mode_change_msg);

        // mavros_msgs::CommandBool arming_request_msgs_;
        // arming_request_msgs_.request.value = false;
        // arming_request_client.call(arming_request_msgs_);

        // mode_change_msg.request.custom_mode = "AUTO.LAND";
        // mode_change_client.call(mode_change_msg);

        // arming_request_msgs.request.value = false;
        // arming_request_client.call(arming_request_msgs);

        land_switch_ = true;
        FT = Standby;
        cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
        cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
        cache_position.pose.position.z = round_(reference_position_cb.pose.position.z);
        cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
        cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
        cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
        cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
        mission_num = MISSION_1_NUM;
    }

    if(ros::Time::now() - land_time >= ros::Duration(0))
    {
        mavros_msgs::CommandBool arming_request_msgs_;
        arming_request_msgs_.request.value = false;
        arming_request_client.call(arming_request_msgs_);
    }
}

void FlightCtrl::cmd_Task_Takeoff(double h)
{

    if(ctrler == 0)
    {
        cache_position.pose.position.x = round_(reference_position_cb.pose.position.x);
        cache_position.pose.position.y = round_(reference_position_cb.pose.position.y);
        cache_position.pose.position.z = h;
        cache_position.pose.orientation.x = reference_position_cb.pose.orientation.x;
        cache_position.pose.orientation.y = reference_position_cb.pose.orientation.y;
        cache_position.pose.orientation.z = reference_position_cb.pose.orientation.z;
        cache_position.pose.orientation.w = reference_position_cb.pose.orientation.w;
        target_position_pub.publish(cache_position);
    }
    else if(ctrler == 1)
    {

        cx.setTarget(round_(reference_position_cb.pose.position.x));
        cy.setTarget(round_(reference_position_cb.pose.position.y));
        cz.setTarget(h);

        cache_vel.linear.x = cx.compute(reference_position_cb.pose.position.x);
        cache_vel.linear.y = cy.compute(reference_position_cb.pose.position.y);
        cache_vel.linear.z = cz.compute(reference_position_cb.pose.position.z);
        cache_vel.angular.z = 0;

        target_velocity_pub.publish(cache_vel);
    }
}

void FlightCtrl::cmd_Task_Mission()
{  
    // switch(mission_num)
    // {
    //     case 4: fly_to(0.5,0,0.5,0,5); break;
    //     case 3: fly_to(0.5,0.5,0.5,0,5); break;
    //     case 2: fly_to(0,0.5,0.5,0,5); break;
    //     case 1: fly_to(0,0,0.5,0,5); break;
    // }

    switch(mission_num)
    {
        case 6: fly_to_(Eigen::Vector4d(1,0,0.8,0),5,true); break;
        case 5: fly_to_(Eigen::Vector4d(1,1,0.8,0),5,true); break;
        case 4: fly_to_(Eigen::Vector4d(-1,1,0.8,0),5,true); break;
        case 3: fly_to_(Eigen::Vector4d(-1,0,0.8,0),5,true); break;
        case 2: fly_to_(Eigen::Vector4d(0,0,0.8,0),5,true); break;
        case 1: fly_to_(Eigen::Vector4d(0,0,0.8,0),5,true); break;

        // case 4: fly_to_(Eigen::Vector4d(1,0,1,0),5); break;
        // case 3: fly_to_(Eigen::Vector4d(1,1,1,0),5); break;
        // case 2: fly_to_(Eigen::Vector4d(0,1,1,0),5); break;
        // case 1: fly_to_(Eigen::Vector4d(0,0,0.5,0),5); break;
    }
}