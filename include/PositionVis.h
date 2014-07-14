#include "ros/ros.h"
#include "asctec_hl_comm/PositionWithCovarianceStamped.h"
#include "asctec_hl_comm/mav_ctrl.h"
#include "TooN/TooN.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

class PositionVis
{
public:

    ~PositionVis();

    static PositionVis * Instance(int argc=0, char **argv=NULL)
    {
        if(instance == NULL)
        {
            instance = new PositionVis(argc, argv);
        }

        return instance;
    }

    void idle();
    void mainLoop();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);
    void glDraw();
    void gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg);
    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    void fixedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    void imuCallback(const sensor_msgs::Imu::Ptr &msg);

    void velCallback(const asctec_hl_comm::mav_ctrl::Ptr &msg);

private:

    PositionVis(int argc, char **argv);
    static PositionVis* instance;

    ros::NodeHandle nh;
    ros::Subscriber gpsPos_sub;
    ros::Subscriber gpsPose_sub;

    ros::Subscriber velcmd_sub;
    ros::Publisher velcmd_pub;

    ros::Subscriber fixedPose_sub;
    ros::Publisher  fixedPose_pub;
    ros::Publisher  position_pub;

    ros::Subscriber imu_sub;

    std::vector<TooN::Vector<3> > positions;
    std::vector<TooN::Vector<3> > p_pos;
    std::vector<TooN::Vector<4> > p_att;
    TooN::Vector<4> orig;
    TooN::Vector<3> p_orig;
    std::vector<double> rpy[3];

    double fixedYaw;
    TooN::Vector<3> vel;

};
