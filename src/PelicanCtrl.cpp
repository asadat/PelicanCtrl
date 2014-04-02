
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include "PelicanCtrl/start_log.h"
#include "PelicanCtrl.h"
#include "TooN/TooN.h"
#include "tf/tf.h"

using namespace TooN;

PelicanPosCtrl* PelicanPosCtrl::instance = NULL;

PelicanPosCtrl::PelicanPosCtrl(int argc, char **argv)
{

    initYaw = 0;
    gpsPos_sub = nh.subscribe("/fcu/gps_position", 100, &PelicanPosCtrl::gpsPositionCallback, this);
    gpsPose_sub = nh.subscribe("/fcu/gps_pose", 100, &PelicanPosCtrl::gpsPoseCallback, this);

    gotoService = nh.advertiseService("gotoPos", &PelicanPosCtrl::GoToPosServiceCall, this);

}

bool PelicanPosCtrl::GoToPosServiceCall(PelicanCtrl::gotoPosRequest &req, PelicanCtrl::gotoPosResponse &res)
{
    Vector<4> p;
    p[0] = req.x;
    p[1] = req.y;
    p[2] = req.z;
    p[3] = req.yaw;

    SetCurGoal(p);

    return true;
}

void PelicanPosCtrl::gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
    static bool firstYaw = true;
    Vector<3> p;
    p[0] = msg->pose.pose.position.x;
    p[1] = msg->pose.pose.position.y;
    p[2] = msg->pose.pose.position.z;

    //if(p_pos.size() > 100)
    //    p_pos.pop_back();

    p_pos.push_back(p);

    Vector<4> att;
    att[0] = msg->pose.pose.orientation.x;
    att[1] = msg->pose.pose.orientation.y;
    att[2] = msg->pose.pose.orientation.z;
    att[3] = msg->pose.pose.orientation.w;

    //if(p_att.size() > 100)
    //    p_att.pop_back();

    p_att.push_back(att);

    tf::Quaternion qu(att[0], att[1], att[2], att[3]);
//    tf::Matrix3x3 m(qu);
//    Matrix<3> rot;
//    rot[0][0] = m[0][0]; rot[1][0] = m[1][0]; rot[2][0] = m[2][0];
//    rot[0][1] = m[0][1]; rot[1][1] = m[1][1]; rot[2][1] = m[2][1];
//    rot[0][2] = m[0][2]; rot[1][2] = m[1][2]; rot[2][2] = m[2][2];
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    if(firstYaw)
    {
        initYaw = yaw;
        firstYaw = false;
    }
    ROS_INFO_THROTTLE(1,"Yaw:\t%f\n", (yaw-initYaw)*180/3.14);


}

void PelicanPosCtrl::gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg)
{
    Vector<3> p;
    p[0] = msg->position.x;
    p[1] = msg->position.y;
    p[2] = msg->position.z;

    positions.push_back(p);

}

void PelicanPosCtrl::SetCurGoal(TooN::Vector<4> p)
{
    curGoal = p;
}

void PelicanPosCtrl::Update()
{

}

bool start_log(PelicanCtrl::start_logRequest &req, PelicanCtrl::start_logResponse &res)
{
    ROS_INFO("Service Called ................");
    system("~/log.sh");
   // system("source /media/startlog.sh");
    return true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "PelicanCtrl");

  PelicanPosCtrl::Instance(argc, argv);

  ros::Rate r(15);

  while(ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    PelicanPosCtrl::Instance()->Update();
  }

  return 0;
}

//  if (argc == 1)
//  {
//    ROS_ERROR("Wrong number of arguments!!!");
//    usage();
//    return -1;
//  }

//  std::string command = std::string(argv[1]);

//  if (command == "motors")
//  {
//    if (argc != 3)
//    {
//      ROS_ERROR("Wrong number of arguments!!!");
//      usage();
//      return -1;
//    }

//    asctec_hl_comm::mav_ctrl_motors::Request req;
//    asctec_hl_comm::mav_ctrl_motors::Response res;
//    req.startMotors = atoi(argv[2]);
//    ros::service::call("fcu/motor_control", req, res);
//    std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
//  }
//  else if (command == "ctrl")
//   {
//    if (argc != 7)
//    {
//      ROS_ERROR("Wrong number of arguments!");
//      usage();
//      return -1;
//    }

//      asctec_hl_comm::mav_ctrl msg;
//    msg.x = 0.01;
//    msg.y = 0;
//    msg.z = 0;
//    msg.yaw = 0;
//    msg.v_max_xy = -1; // use max velocity from config
//    msg.v_max_z = -1;

//    std::string type("vel");
//    if (type == "acc")
//      msg.type = asctec_hl_comm::mav_ctrl::acceleration;
//    else if (type == "vel")
//      msg.type = asctec_hl_comm::mav_ctrl::velocity;
//    else if (type == "pos")
//      msg.type = asctec_hl_comm::mav_ctrl::position;
//    else
//    {
//      ROS_ERROR("Command type not recognized");
//      //usage();
//      return -1;
//    }

//    pub = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);
//    ros::Rate r(15); // ~15 Hz

//    while(ros::ok())
//    {

//        //if(startPeriodic)
//        {
//            for (int i = 0; i < 15; i++)
//            {
//              pub.publish(msg);
//              if (!ros::ok())
//                return 0;
//              r.sleep();
//            }
//        }
//            // reset
//        if (type != "pos")
//        {
//          msg.x = 0;
//          msg.y = 0;
//          msg.z = 0;
//          msg.yaw = 0;
//        }

//        for(int i=0; i<5; i++){
//          pub.publish(msg);
//          r.sleep();
//        }


//        ros::spinOnce();
//    }

//  }

