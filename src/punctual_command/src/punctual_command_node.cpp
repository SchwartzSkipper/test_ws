#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <dynamic_reconfigure/server.h>
#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>
#include <punctual_command/PunctualCommandConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

class PunctualCommand{
    public:
        PunctualCommand(std::string traj);
        ~PunctualCommand();
        void NavCtrlStatusCallback(const yocs_msgs::NavigationControlStatus::ConstPtr& nav_ctrl_status);
        void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped initial_pose);
        void loopTalker();
        //void PublishStatus();
        // void reconfigureCB(PunctualCommandConfig &config, uint32_t level);   
    

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber nav_ctrl_status_sub;
        ros::Publisher  nav_ctrl_pub; 
        ros::Time time_,mark_;
        ros::Duration period_;
        double period_time_;
        ros::Duration initial_time_;
        std::string waypoint_name_;
        bool initial_command_;
        double mark_secs,time_secs;
        yocs_msgs::NavigationControl nav_ctrl_;
        yocs_msgs::NavigationControlStatus nav_status_;

        dynamic_reconfigure::Server<punctual_command::PunctualCommandConfig> *dsrv_;
            
};

int main(int argc,char** argv){
    ros::init(argc,argv,"punctual_command_node");
    ros::NodeHandle pnh("~");
    double period;
    std::string goal_name;
    pnh.param("period",period,30.0);
    pnh.param("goal_name",goal_name,std::string("punctual_traj"));
    ros::Rate loop_rate(1/period);
    PunctualCommand pc(goal_name);
    while(ros::ok){
        pc.loopTalker();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

PunctualCommand::PunctualCommand(std::string traj):nh_(""),pnh_("~"){
    nav_ctrl_status_sub = nh_.subscribe("nav_ctrl_status",1,&PunctualCommand::NavCtrlStatusCallback,this);
    // initial_pose_sub = nh_.subscribe("initialpose",1,&PunctualCommand::InitialPoseCallback,this);
    nav_ctrl_pub = nh_.advertise<yocs_msgs::NavigationControl>("nav_ctrl", 1, true);
    period_ = ros::Duration(period_time_); 
    initial_time_ = ros::Duration(30.0);
    mark_ = ros::Time::now();
    initial_command_  = true;
    nav_ctrl_.control = yocs_msgs::NavigationControl::START;
    nav_ctrl_.goal_name = traj;
}


PunctualCommand::~PunctualCommand()
{}

void PunctualCommand::NavCtrlStatusCallback(const yocs_msgs::NavigationControlStatus::ConstPtr& nav_ctrl_status){
    nav_status_.status = nav_ctrl_status->status;
}

void PunctualCommand::loopTalker()
{   
    if(initial_command_){
        sleep(30);
        nav_ctrl_pub.publish(nav_ctrl_);
        initial_command_ = false;
    }
    else if(nav_status_.status == yocs_msgs::NavigationControlStatus::IDLING){
        
        nav_ctrl_pub.publish(nav_ctrl_);
    }
}
