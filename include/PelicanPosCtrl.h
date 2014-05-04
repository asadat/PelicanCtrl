#include "ros/ros.h"
#include "asctec_hl_comm/PositionWithCovarianceStamped.h"
#include "TooN/TooN.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "PelicanCtrl/gotoPos.h"
#include "control_toolbox/pid.h"
#include "PelicanCtrl/hover.h"

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
    void magCallback(const geometry_msgs::Vector3Stamped::Ptr &msg);
    void gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg);
    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    bool GoToPosServiceCall(PelicanCtrl::gotoPosRequest &req, PelicanCtrl::gotoPosResponse &res);
    bool HoverServiceCall(PelicanCtrl::hoverRequest &req, PelicanCtrl::hoverResponse &res);

private:

    PelicanPosCtrl(int argc, char **argv);
    static PelicanPosCtrl* instance;

    void SetCurGoal(TooN::Vector<4> p);
    void OnReachedGoal();

    void TransformFromGlobal2Pelican(TooN::Vector<4> &curCtrl);

    ros::NodeHandle nh;
    ros::Subscriber gpsPose_sub;
    ros::Subscriber mag_sub;

    ros::ServiceServer gotoService;
    ros::ServiceServer hoverService;

    ros::Publisher  velPub;
    ros::Publisher atGoalPub;
    ros::Publisher fixedPosePub;

    std::vector<TooN::Vector<3> > p_pos;
    std::vector<TooN::Vector<4> > p_att;
    std::vector<double> yaws;

    TooN::Vector<4> curGoal;
    TooN::Vector<3> curPos;
    double curYaw;
    TooN::Vector<4> curCtrl;
    double angularVelDir;

    double pidz_a[3]; //PID gains used when ascending
    double pidz_d[3]; //PID gains used when descending

    TooN::Vector<3> orig;

    double max_xy_v;
    double small_xyz_v;
    double ctrlCutoff[NUM];
    double goalThr[NUM];
    bool hover;
    bool hasHoverPos;
    bool origIsSet;

    control_toolbox::Pid pid[NUM];
};
