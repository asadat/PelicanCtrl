#include "ros/ros.h"
#include "asctec_hl_comm/PositionWithCovarianceStamped.h"
#include "asctec_hl_comm/Wgs84ToEnu.h"
#include "TooN/TooN.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "pelican_ctrl/gotoPos.h"
#include "control_toolbox/pid.h"
#include "pelican_ctrl/hover.h"
#include "pelican_ctrl/gotoPos_body.h"
#include "pelican_ctrl/gotoPosGPS.h"

class PelicanPosCtrl
{
public:
    enum DOF {X=0, Y, Z, YAW, NUM};

    ~PelicanPosCtrl();

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
    bool GoToPosServiceCall(pelican_ctrl::gotoPosRequest &req, pelican_ctrl::gotoPosResponse &res);
    bool GoToPosGPSServiceCall(pelican_ctrl::gotoPosGPSRequest &req, pelican_ctrl::gotoPosGPSResponse &res);
    bool GoToPos_bodyServiceCall(pelican_ctrl::gotoPos_bodyRequest &req, pelican_ctrl::gotoPos_bodyResponse &res);
    bool HoverServiceCall(pelican_ctrl::hoverRequest &req, pelican_ctrl::hoverResponse &res);

private:

    PelicanPosCtrl(int argc, char **argv);
    static PelicanPosCtrl* instance;

    void SetCurGoal_body(TooN::Vector<4> p);
    void SetCurGoal(TooN::Vector<4> p);
    void OnReachedGoal();

    void TransformFromGlobal2Pelican(TooN::Vector<4> &curCtrl);

    ros::NodeHandle nh;
    ros::Subscriber gpsPose_sub;
    ros::Subscriber mag_sub;

    ros::ServiceClient wsg84Toxyz;
    ros::ServiceServer gotoBodyService;
    ros::ServiceServer gotoService;
    ros::ServiceServer gotoGPSService;
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
