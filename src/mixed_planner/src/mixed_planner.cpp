#include <pluginlib/class_list_macros.h>
#include <global_planner/mixed_planner.h>
#include <pcl_conversions/pcl_conversions.h>
#include <navfn/navfn_ros.h>
#include <global_planner/fixed_planner_core.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::MixedPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {
    MixedPlanner::MixedPlanner()
        :costmap_ros_(NULL), navfn_planner_(), fixed_planner_(), planner_is_navfn_(false), initialized_(false){

    }
    MixedPlanner::MixedPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::string frame_id)
        :costmap_ros_(NULL), navfn_planner_(), fixed_planner_(), planner_is_navfn_(false), initialized_(false)
    {
        initialize(name, costmap_ros, costmap_ros->getGlobalFrameID());
    }

    MixedPlanner::~MixedPlanner(){}

    void MixedPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros, costmap_ros->getGlobalFrameID());
    }
    
    void MixedPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::string frame_id){
        if(!initialized_){
           //default planner: fixed global planner
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros->getCostmap();
            frame_id_ = frame_id;
            navfn_planner_ = boost::shared_ptr<navfn::NavfnROS>(new navfn::NavfnROS(name, costmap_ros));
            //TEST
            ROS_INFO("navfn_initialized:%d",navfn_planner_->initialized_);
            fixed_planner_ = boost::shared_ptr<FixedGlobalPlanner>(new FixedGlobalPlanner(name, costmap_ros->getCostmap(), frame_id));
            //TEST
            ROS_INFO("fixed_initialized:%d",fixed_planner_->initialized_);
            
            dsrv_m_ = new dynamic_reconfigure::Server<::mixed_planner::MixedPlannerConfig>(ros::NodeHandle("~/" + name + "/navfn_planner"));
            dsrv_f_ = new dynamic_reconfigure::Server<::mixed_planner::FixedPlannerConfig>(ros::NodeHandle("~/" + name + "/fixed_planner"));

            dynamic_reconfigure::Server<::mixed_planner::MixedPlannerConfig>::CallbackType cb_m = boost::bind(&MixedPlanner::reconfigureCB_m, this, _1, _2);
            dynamic_reconfigure::Server<::mixed_planner::FixedPlannerConfig>::CallbackType cb_f = boost::bind(&MixedPlanner::reconfigureCB_f, this, _1, _2);
            dsrv_m_->setCallback(cb_m);
            dsrv_f_->setCallback(cb_f);

            initialized_ = true;            
        }
        else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }

    bool MixedPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
            if(!planner_is_navfn_){
                plan.clear();
                return fixed_planner_->makePlan(start, goal, fixed_planner_->endpoint_tolerance_, plan);
            }
            else{
                plan.clear();
                return navfn_planner_->makePlan(start, goal, navfn_planner_->default_tolerance_, plan);
            }
        }

    bool MixedPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
            if(!planner_is_navfn_){
                plan.clear();
                return fixed_planner_->makePlan(start, goal, tolerance, plan);
            }
            else{
                plan.clear();
                return navfn_planner_->makePlan(start, goal, tolerance, plan);  
            }                  
    }

    void MixedPlanner::pathCallback(scheduling_msgs::PathStampWithID path){
        if(!planner_is_navfn_)
            fixed_planner_->pathCallback(path);
    }

    void MixedPlanner::reconfigureCB_m(mixed_planner::MixedPlannerConfig& config, uint32_t level){
        planner_is_navfn_ = config.planner_is_navfn;
    }

    void MixedPlanner::reconfigureCB_f(mixed_planner::FixedPlannerConfig& config, uint32_t level){       
        if(!planner_is_navfn_){ 
            fixed_planner_->allow_unknown_ = config.allow_unknown;
            fixed_planner_->endpoint_tolerance_= config.endpoint_tolerance;
            fixed_planner_->retrace_path_ = config.retrace_path;
            fixed_planner_->use_goal_direction_=config.use_goal_direction;
            if(fixed_planner_->sub_path_topic_!=config.subscribe_path_topic){
                boost::mutex::scoped_lock lock(fixed_planner_->mutex_);
                fixed_planner_->sub_path_topic_=config.subscribe_path_topic;
                ROS_WARN("Switch to subscribe new path topic: %s",fixed_planner_->sub_path_topic_.c_str());
                fixed_planner_->path_sub_.shutdown();
                fixed_planner_->path_sub_ = fixed_planner_->nh_.subscribe(fixed_planner_->sub_path_topic_,2,&MixedPlanner::pathCallback,this);
            }
        }
    }

    void MixedPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
        if(!planner_is_navfn_)
            fixed_planner_->publishPlan(path);
        else
            navfn_planner_->publishPlan(path, r, g, b, a);
    }
}; //end namespce global_planner





    


