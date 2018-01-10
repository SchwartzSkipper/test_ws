#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H

#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <scheduling_msgs/PathWithID.h>
#include <scheduling_msgs/PathStampWithID.h>
#include <scheduling_msgs/ClearPlannerPath.h>
#include <scheduling_msgs/SetPlannerPath.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <fixed_global_planner/FixedGlobalPlannerConfig.h>


namespace global_planner {

class Expander;
class GridPath;

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class FixedGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        FixedGlobalPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        FixedGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~FixedGlobalPlanner();

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        double poseDist(const geometry_msgs::PoseStamped pose0, const geometry_msgs::PoseStamped pose1);
        
        friend class MixedPlanner;
        boost::mutex mutex_;
protected:
        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;

        bool initialized_, allow_unknown_;
    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);

        //Added by Pan
        void pathCallback(scheduling_msgs::PathStampWithID path);
        bool setPlannerPathServiceCallback(scheduling_msgs::SetPlannerPath::Request &req, scheduling_msgs::SetPlannerPath::Response &resp);
        bool clearPlannerPathServiceCallback(scheduling_msgs::ClearPlannerPath::Request &req, scheduling_msgs::ClearPlannerPath::Response &resp);

        ros::NodeHandle nh_;
        ros::Subscriber path_sub_;
        //nav_msgs::Path specified_path_;
        scheduling_msgs::PathStampWithID specified_path_;
        std::string sub_path_topic_;
        std::string set_planner_path_service_name_;
        double error_retry_time_;
        double endpoint_tolerance_;
        bool retrace_path_;
        bool include_start_pose_;
        bool include_goal_pose_;//I think for consistance, we should not activate this option
        bool use_goal_direction_;
        bool dynamic_reconfigure_subscribed_path_topic_;
        bool always_update_path_;

        double planner_window_x_, planner_window_y_;

        std::string tf_prefix_;
        
        ros::ServiceServer make_plan_srv_;
        ros::ServiceServer set_plan_service_;
        ros::ServiceServer set_planner_path_service_;
        ros::ServiceServer clear_planner_path_service_;

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
        unsigned char* cost_array_;

        bool old_navfn_behavior_;
        float convert_offset_; //for conversion between maps of map and world

        dynamic_reconfigure::Server<::fixed_global_planner::FixedGlobalPlannerConfig> *dsrv_;
        void reconfigureCB(::fixed_global_planner::FixedGlobalPlannerConfig &config, uint32_t level);
};

} //end namespace global_planner

#endif
