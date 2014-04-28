
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include "PelicanCtrl/start_log.h"
#include "PelicanPosCtrl.h"
//#include "TooN/TooN.h"
#include "tf/tf.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


#define CUTOFF(a,b,c) (a<b)?b:(a>c?c:a)
#define DEADZONE(a,b) ((fabs(a)<b)?0:a)

using namespace TooN;

PelicanPosCtrl* PelicanPosCtrl::instance = NULL;

PelicanPosCtrl::PelicanPosCtrl(int argc, char **argv):nh("PelicanCtrl")
{

    hasHoverPos = false;
    hover = false;
    orig = makeVector(0,0,0);
    gpsPose_sub = nh.subscribe("/fcu/gps_pose", 100, &PelicanPosCtrl::gpsPoseCallback, this);
    mag_sub = nh.subscribe("/fcu/mag", 10, &PelicanPosCtrl::magCallback, this);

    gotoService = nh.advertiseService("gotoPos", &PelicanPosCtrl::GoToPosServiceCall, this);
    hoverService = nh.advertiseService("hover", &PelicanPosCtrl::HoverServiceCall, this);

    velPub = nh.advertise<asctec_hl_comm::mav_ctrl>("/fcu/control", 1);
    atGoalPub = nh.advertise<std_msgs::Bool>("at_goal", 10);

    fixedPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("fixedPose", 100);

    curCtrl = makeVector(0,0,0,0);

    curPos = makeVector(0,0,1);
    curYaw = 0;
    curGoal = makeVector(curPos[0], curPos[1], curPos[2], curYaw);

    double pidx[3];
    double pidy[3];

    double pid_yaw[3];

    nh.param<double>("px",pidx[0], 0.01);
    nh.param<double>("dx",pidx[1], 0.0);
    nh.param<double>("ix",pidx[2], 0.0);

    nh.param<double>("py",pidy[0], 0.01);
    nh.param<double>("dy",pidy[1], 0.0);
    nh.param<double>("iy",pidy[2], 0.0);

    nh.param<double>("pz_a",pidz_a[0], 0.01);
    nh.param<double>("dz_a",pidz_a[1], 0.0);
    nh.param<double>("iz_a",pidz_a[2], 0.0);

    nh.param<double>("pz_d",pidz_d[0], 0.01);
    nh.param<double>("dz_d",pidz_d[1], 0.0);
    nh.param<double>("iz_d",pidz_d[2], 0.0);

    nh.param<double>("p_yaw",pid_yaw[0], 0.01);
    nh.param<double>("d_yaw",pid_yaw[1], 0.0);
    nh.param<double>("i_yaw",pid_yaw[2], 0.0);

    //ROS_INFO("ctrl: %f %f %f %f", pidx[0], pidy[0], pidz[0], pid_yaw[0]);

    pid[X].initPid(pidx[0],pidx[2], pidx[1], 0, -0);
    pid[Y].initPid(pidy[0],pidy[2], pidy[1], 0, -0);
    pid[Z].initPid(pidz_d[0],pidz_d[2], pidz_d[1], 0, -0);
    pid[YAW].initPid(pid_yaw[0],pid_yaw[2], pid_yaw[1], 0, -0);

    nh.param<double>("max_vx",ctrlCutoff[X],0.5);
    nh.param<double>("max_vy",ctrlCutoff[Y],0.5);
    nh.param<double>("max_vz",ctrlCutoff[Z],0.5);

    //ctrlCutoff[X] = 0.5;
    //ctrlCutoff[Y] = 0.5;
    //ctrlCutoff[Z] = 0.05;
    ctrlCutoff[YAW] = 0.1;

    nh.param<double>("goal_thr_x",goalThr[X], 0);
    nh.param<double>("goal_thr_y",goalThr[Y], 0);
    nh.param<double>("goal_thr_z",goalThr[Z], 0);
    nh.param<double>("goal_thr_yaw",goalThr[YAW], 0);
    ROS_INFO("goal reached thresholds: %f %f %f %f", goalThr[X],goalThr[Y],goalThr[Z],goalThr[YAW]);
    //goalThr[X] = 1.5;
    //goalThr[Y] = 1.5;
    //goalThr[Z] = 0.0;
    //goalThr[YAW] = 0.2;

    //SetCurGoal(orig);
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

bool PelicanPosCtrl::HoverServiceCall(PelicanCtrl::hoverRequest &req, PelicanCtrl::hoverResponse &res)
{
    if(!hasHoverPos)
        return false;

    curGoal = makeVector(curPos[0],curPos[1],curPos[2],curYaw);
    return true;
}


void PelicanPosCtrl::gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
    static bool firstgpPose = true;
    Vector<3> p;
    p[0] = msg->pose.pose.position.x;
    p[1] = msg->pose.pose.position.y;
    p[2] = msg->pose.pose.position.z;

    if(p_pos.size() > 100)
        p_pos.erase(p_pos.begin());

    p_pos.push_back(p);

//    Vector<4> att;
//    att[0] = msg->pose.pose.orientation.x;
//    att[1] = msg->pose.pose.orientation.y;
//    att[2] = msg->pose.pose.orientation.z;
//    att[3] = msg->pose.pose.orientation.w;

//    if(p_att.size() > 100)
//        p_att.erase(p_att.begin());

//    p_att.push_back(att);

    //double yaw = tf::getYaw(msg->pose.pose.orientation);

    if(firstgpPose)
    {
        orig = p;
        firstgpPose = false;
    }

    curPos = p-orig;

    geometry_msgs::PoseWithCovarianceStamped fixepose;
    fixepose.pose.pose.position.x = curPos[0];
    fixepose.pose.pose.position.y = curPos[1];
    fixepose.pose.pose.position.z = curPos[2];

    tf::Quaternion q = tf::Quaternion(curYaw,0,0);

    fixepose.pose.pose.orientation.x = q.x();
    fixepose.pose.pose.orientation.y = q.y();
    fixepose.pose.pose.orientation.z = q.z();
    fixepose.pose.pose.orientation.w = q.w();

    fixepose.header.stamp = msg->header.stamp;
    fixedPosePub.publish(fixepose);


    ROS_INFO_THROTTLE(4,"POSE: %f\t%f\t%f\t%f ", curPos[0], curPos[1], curPos[2], (curYaw)*180/3.14);
}

void PelicanPosCtrl::magCallback(const geometry_msgs::Vector3Stamped::Ptr &msg)
{
    curYaw = atan2(msg->vector.y, msg->vector.x) + ((45.0)*3.14/180.0);
    if(curYaw > 3.14)
        curYaw -= 2*3.14;

    yaws.push_back(curYaw);
    if(yaws.size()>50)
    {
        yaws.erase(yaws.begin());
    }

    curYaw = 0;
    for(int i=0; i<yaws.size(); i++)
    {
        curYaw += yaws[i];
    }

    curYaw /= yaws.size();
   
    /* geometry_msgs::PoseWithCovarianceStamped fixepose;
    fixepose.pose.pose.position.x = curPos[0];
    fixepose.pose.pose.position.y = curPos[1];
    fixepose.pose.pose.position.z = curYaw;

    tf::Quaternion q = tf::Quaternion(curYaw,0,0);

    fixepose.pose.pose.orientation.x = q.x();
    fixepose.pose.pose.orientation.y = q.y();
    fixepose.pose.pose.orientation.z = q.z();
    fixepose.pose.pose.orientation.w = q.w();

    fixepose.header.stamp = msg->header.stamp;
    fixedPosePub.publish(fixepose);
*/
  //  ROS_INFO("**************** YAW: %f", curYaw);
}


void PelicanPosCtrl::SetCurGoal(TooN::Vector<4> p)
{
    hasHoverPos = true;
    hover = false;
    curGoal = p;
}
void PelicanPosCtrl::OnReachedGoal()
{
   std_msgs::Bool atgoal;
   atgoal.data = true;
   atGoalPub.publish(atgoal);
   hover = true;
}

void PelicanPosCtrl::Update()
{
    static bool ascending = false;

    ROS_INFO_THROTTLE(5,"Hover:%d",hover);
    if(!hasHoverPos)
        return;

    ROS_INFO_THROTTLE(5,"goal:%f %f %f",curGoal[0],curGoal[1],curGoal[2]);

    static ros::Time lastTime = ros::Time::now();
    ros::Time curTime = ros::Time::now();
    ros::Duration dt = lastTime-curTime;
    lastTime = curTime;

    bool zeroCtrl = true;

    Vector<4> err_4D;
    for(int i=0; i<=YAW; i++)
    {
        if(i<YAW)
        {
            err_4D[i] = curPos[i]-curGoal[i];
        }
        else
        {
            err_4D[i] = curYaw-curGoal[i];
        }

        err_4D[i] = DEADZONE(err_4D[i], goalThr[i]);


        zeroCtrl = zeroCtrl && (fabs(err_4D[i]) < 0.001);

        if(i == Z) // set the correct PID gains for ascending/descending
        {
            ROS_INFO("Asc:%d ERR:%f",ascending,err_4D[i] );

            if(ascending && err_4D[i] > 0)
            {
                ascending = false;
                pid[i].setGains(pidz_d[0],pidz_d[2],pidz_d[1],0,-0);
                ROS_INFO("Descending: %f %f %f",pidz_d[0],pidz_d[1],pidz_d[2]);
            }
            else if(!ascending && err_4D[i] < 0)
            {
                ascending = true;
                pid[i].setGains(pidz_a[0],pidz_a[2],pidz_a[1],0,-0);
                ROS_INFO("Ascending: %f %f %f",pidz_a[0],pidz_a[1],pidz_a[2]);
            }
        }

        curCtrl[i] = pid[i].updatePid(err_4D[i], dt);
        curCtrl[i] = CUTOFF(curCtrl[i], -ctrlCutoff[i], ctrlCutoff[i]);
    }

    if(zeroCtrl && !hover)
    {
        OnReachedGoal();
    }
    else
    {

        ROS_INFO_THROTTLE(5, "CTRL-W: %f\t%f\t%f\t%f dt: %f", curCtrl[0], curCtrl[1], curCtrl[2], curCtrl[3], dt.toSec());
        TransformFromGlobal2Pelican(curCtrl);
        ROS_INFO_THROTTLE(5, "ERR: %f\t%f\t%f dt: %f", err_4D[0], err_4D[1], err_4D[2], dt.toSec());
        ROS_INFO_THROTTLE(5, "CTRL-P: %f\t%f\t%f\t%f dt: %f", curCtrl[0], curCtrl[1], curCtrl[2], curCtrl[3], dt.toSec());

//        geometry_msgs::Twist velmsg;
//        velmsg.linear.x = curCtrl[0];
//        velmsg.linear.y = curCtrl[1];
//        velmsg.linear.z = curCtrl[2];




        asctec_hl_comm::mav_ctrl velmsg;

        if(hover)
        {
            velmsg.yaw = curCtrl[3];
        }

        velmsg.x = curCtrl[0];
        velmsg.y = curCtrl[1];
        velmsg.z = curCtrl[2];
        velmsg.type = velmsg.velocity;
        velmsg.v_max_xy = -1;
        velmsg.v_max_z   = -1;
        velPub.publish(velmsg);
    }

}

void PelicanPosCtrl::TransformFromGlobal2Pelican(Vector<4> &ctrl_cur)
{
    Vector<2> vxvy = makeVector(ctrl_cur[0], ctrl_cur[1]);
    Matrix<2, 2, double> trans = Data(-cos(curYaw), -sin(curYaw),
                                 -sin(curYaw), cos(curYaw));

    vxvy = trans*vxvy;

    ctrl_cur[0] = vxvy[0];
    ctrl_cur[1] = vxvy[1];
}

bool start_log(PelicanCtrl::start_logRequest &req, PelicanCtrl::start_logResponse &res)
{
    ROS_INFO("Service Called ................");
    //system("~/log.sh");
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

