#include "FlightCtrl.hpp"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "FlightCtrl");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    FlightCtrl FlightCtrler(nh,nh_private);
    FlightCtrler.main_loop();
    
    return 0;
}