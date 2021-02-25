#include <ros/ros.h>
#include <mutex>
#include "sam_slam_msgs/OdomEdgeStamped.h"
#include "sam_slam_msgs/LandmarkEdgeStamped.h"
#include "sam_slam_msgs/MeanCovariance.h"
#include "sam_slam_back_end/sam_system.h"

class SamRosWrapper
{
    // TODO templater ceci par dessus SAM::SanSystem un jour (pour que ca scalez)
public:
    SamRosWrapper(const Pose2D &posini, Eigen::Matrix3d &covini, const double &back_end_frequency) : 
                nh_(ros::NodeHandle("~"))
                , sub_odom_(nh_.subscribe("/output_front_end_odom", 10, &SamRosWrapper::callback_odom, this))
                , sub_tag_(nh_.subscribe("/output_front_end_tag", 10, &SamRosWrapper::callback_tag, this))
                , pub_(nh_.advertise<sam_slam_msgs::MeanCovariance>("/sam_slam_output", 5))
                , timer_(nh_.createTimer(ros::Duration(1.f / back_end_frequency), &SamRosWrapper::callback_timer, this))
                , sam_system_(posini, covini)
    {
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_tag_;
    ros::Timer timer_;
    SAM::SamSystem<Factor::FactorOdom, Factor::FactorRangeBearing> sam_system_;
    std::mutex sam_mutex_; // callbacks and timer are concurrent: so protect ressource with a mutex

    void callback_odom(const sam_slam_msgs::OdomEdgeStampedConstPtr &msg)
    {
        std::lock_guard<std::mutex> l(sam_mutex_); // protecting access to sam_system

        Factor::MesVelOdom_t mesvelodom = {msg->E.v, msg->E.w, 0} ;
        Factor::CovMesVelOdom_t cov_mesVelOdom = Eigen::Map<const Factor::CovMesVelOdom_t>(msg->E.covariance.data());

        Factor::VelOdomVarsStr_t tmp;  // TODO : splitter le string qui vient du msg en std array (1 var / idx)
        Factor::FactorOdom factor_odom(tmp, mesvelodom, cov_mesVelOdom,msg->dt);
        
        sam_system_.register_new_factor(factor_odom);
    }

    void callback_tag(const sam_slam_msgs::LandmarkEdgeStampedConstPtr &msg)
    {
        std::lock_guard<std::mutex> l(sam_mutex_); // protecting access to sam_system

        Factor::MesRangeBearing_t mesrb = {msg->r, msg->phi };
        Factor::CovMesRangeBearing_t covrb = Eigen::Map<const Factor::CovMesRangeBearing_t>(msg->covariance.data());
        Factor::RangeBearingVarsStr_t tmp; // TODO : splitter le string qui vient du msg en std array (1 var / idx)
        Factor::FactorRangeBearing factor_rb(tmp, mesrb, covrb);

        sam_system_.register_new_factor(factor_rb);
    }

    void callback_timer(const ros::TimerEvent &e)
    {
        std::lock_guard<std::mutex> l(sam_mutex_); // protecting access to sam_system

        // where the magic happens
        sam_system_.smooth_and_map();

        sam_slam_msgs::MeanCovariance mean_cov_msg;
        Eigen::VectorXd mean = sam_system_.mean_;
        Eigen::VectorXd covariance = sam_system_.covariance_;

        mean_cov_msg.mean = std::vector<double>(
            mean.data(), mean.data() + mean.cols() * mean.rows()); // TODO: voir si methode meilleure avec Eigen::Map<> ?

        mean_cov_msg.covariance =
            std::vector<double>(covariance.data(), covariance.data() + covariance.cols() * covariance.rows());

        // ROS publish le mean et la cov    
        pub_.publish(mean_cov_msg);
    }
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "sam_slam_back_end");

    // TODO: get initial positions & cov from ROS parameter server and timer frequency
    int back_end_frequency = 5;

    Pose2D poseini(0, 0, 0);
    Eigen::Matrix3d covariance_ini;

    covariance_ini << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.003;

    // calling the ROS wrapper, containing callbacksm timers and the calls to the SAM system API
    SamRosWrapper(poseini, covariance_ini, back_end_frequency);

    ros::spin();

    return 0;
}
