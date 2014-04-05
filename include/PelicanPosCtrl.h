#include "ros/ros.h"
#include "asctec_hl_comm/PositionWithCovarianceStamped.h"
#include "TooN/TooN.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "PelicanCtrl/gotoPos.h"
#include "control_toolbox/pid.h"

class PelicanPosCtrl
{
public:
    enum DOF {X=0, Y, Z, YAW, NUM};

    ~PelicanPosCtrl(){};

    static PelicanPosCtrl * Instance(int argc=0, char **argv=NULL)
    {
        if(instance == NULL)
        {
            instance = new PelicanPosCtrl(argc, argv);
        }

        return instance;
    }

    void Update();
    void gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg);
    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    bool GoToPosServiceCall(PelicanCtrl::gotoPosRequest &req, PelicanCtrl::gotoPosResponse &res);

private:

    PelicanPosCtrl(int argc, char **argv);
    static PelicanPosCtrl* instance;

    void SetCurGoal(TooN::Vector<4> p);

    ros::NodeHandle nh;
    ros::Subscriber gpsPos_sub;
    ros::Subscriber gpsPose_sub;
    ros::ServiceServer gotoService;
    ros::Publisher  velPub;

    std::vector<TooN::Vector<3> > positions;
    std::vector<TooN::Vector<3> > p_pos;
    std::vector<TooN::Vector<4> > p_att;

    TooN::Vector<4> curGoal;
    TooN::Vector<4> curPos;
    TooN::Vector<4> curCtrl;

    double initYaw;

    double ctrlCutoff[NUM];
    double goalThr[NUM];

    control_toolbox::Pid pid[NUM];
};
