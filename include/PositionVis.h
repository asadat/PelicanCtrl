#include "ros/ros.h"
#include "asctec_hl_comm/PositionWithCovarianceStamped.h"
#include "asctec_hl_comm/mav_ctrl.h"
#include "TooN/TooN.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"

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

    std::vector<TooN::Vector<3> > positions;
    std::vector<TooN::Vector<3> > p_pos;
    std::vector<TooN::Vector<4> > p_att;
    TooN::Vector<4> orig;

    double fixedYaw;
    TooN::Vector<3> vel;

};
