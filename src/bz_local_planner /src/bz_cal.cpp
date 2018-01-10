void BZPlannerROS::calcBezierVel(geometry_msgs::Twist& cmd_vel, std::vector<geometry_msgs::Pose2D>& points)
{
    geometry_msgs::PoseStamped pose, global_pose;
    tf::poseStampedTFToMsg(current_pose_, pose);
    try
    {
        tf_->waitForTransform("/map", costmap_ros_->getGlobalFrameID(), ros::Time(0), ros::Duration(5.0));
        tf_->transformPose("/map", pose, global_pose); 
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        exit(1);
    }
    struct{
    geometry_msgs::Pose2D ps,pt,pu,pv;
    }bez_points;
    bez_points.ps.x = 
    bez_points.ps.y = 
    bez_points.pt.x = 





    geometry_msgs::Pose2D pr, ps, pt, pu, pv;
    ps.x = global_pose.pose.position.x;
    ps.y = global_pose.pose.position.y;
    ps.theta = tf::getYaw(global_pose.pose.orientation);
    double ps_tan = tan(ps.theta);
    pt.x = goal_.pose.position.x;
    pt.y = goal_.pose.position.y;
    pt.theta = tf::getYaw(goal_.pose.orientation);
    double pt_tan = tan(pt.theta);
    pr.x = (ps_tan * ps.x - pt_tan * pt.x - ps.y + pt.y) / (ps_tan - pt_tan);
    pr.y = ps_tan * (pr.x - ps.x) + ps.y;
    pr.theta = atan2(ps.y - pt.y, ps.x - pt.x);
    double pr_len = sqrt(pow(ps.x - pt.x, 2) + pow(ps.y - pt.y, 2));
    double px_len = pr_len * cos(pr.theta -pt.theta);
    // for isGoalReached use
    current_tolerance_ = px_len;
    double px_offset = (pr_len > (x_offset_pos_ + x_offset_neg_)) ? x_offset_pos_ : (pr_len - x_offset_neg_);
    double pr_dir = (fabs(pt.theta - pr.theta) > M_PI) ? (2*M_PI - fabs(pt.theta - pr.theta)) : (fabs(pt.theta - pr.theta));
    if (pr_dir < M_PI_2) 
    {
        pr_len = - pr_len;
        px_offset = - px_offset;
    }
    pt.x = pt.x - px_offset * cos(pt.theta);
    pt.y = pt.y - px_offset * sin(pt.theta);
    double ps_offset = (pr_len - px_offset) * source_u_;
    pu.x = ps.x + ps_offset * cos(ps.theta);
    pu.y = ps.y + ps_offset * sin(ps.theta);
    double pt_offset = (pr_len - px_offset) * target_v_;
    pv.x = pt.x - pt_offset * cos(pt.theta);
    pv.y = pt.y - pt_offset * sin(pt.theta);
    double dx_d = ps.x;
    double dx_c = 3.0*(pu.x-ps.x);
    double dx_b = 3.0*(pv.x-pu.x)-dx_c;
    double dx_a = pt.x-ps.x-dx_c-dx_b;
    double dy_d = ps.y;
    double dy_c = 3.0*(pu.y-ps.y);
    double dy_b = 3.0*(pv.y-pu.y)-dy_c;
    double dy_a = pt.y-ps.y-dy_c-dy_b;
    double t = 0.1;
    double tSquared = pow(t,2);
    double tCubed = tSquared*t;
    double dx = dx_a*tCubed + dx_b*tSquared + dx_c*t + dx_d;
    double dy = dy_a*tCubed + dy_b*tSquared + dy_c*t + dy_d; 
    double dxy = sqrt(pow(dx - ps.x, 2) + pow(dy - ps.y, 2));
    double dyaw_temp = ps.theta - (pr_len < 0 ? M_PI : 0) * (ps.theta > 0 ? 1 : -1);
    double dyaw2 = atan2(dy-ps.y, dx-ps.x);
    double dyaw = fmod(dyaw2 - dyaw_temp, M_PI * 2);
    if (dyaw > M_PI) 
        dyaw -= M_PI*2;
    if (dyaw < -M_PI) 
        dyaw += M_PI*2;
    cmd_vel.linear.x = (pr_len > 0 ? 1 : -1);
    cmd_vel.angular.z = fabs(cmd_vel.linear.x / dxy) * dyaw * 3;
    double cmd_vel_factor = (1 / sqrt(pow(cmd_vel.linear.x, 2) + pow(cmd_vel.angular.z, 2)));
    cmd_vel_factor *= fmin(0.1 * px_len + 0.15, 0.3);
    cmd_vel.linear.x *= cmd_vel_factor;
    cmd_vel.angular.z *= cmd_vel_factor;
    points.push_back(ps);
    points.push_back(pu);
    points.push_back(pv);
    points.push_back(pt);
}
