#include "global_path_planner/following_planner.hpp"
#include "global_path_planner/highway_planner_core.hpp"

#include <ros/ros.h>

#include <katri_msgs/Objects.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class HighwayPlannerNode
{
public:
    HighwayPlannerNode()
        : nh_(),
          pnh_("~"),
          following_planner_(loadFollowingConfig()),
          speed_filter_(loadSpeedFilterConfig())
    {
        loadParams();

        global_path_sub_ = nh_.subscribe(
            global_path_topic_, 1, &HighwayPlannerNode::globalPathCallback, this);
        local_path_sub_ = nh_.subscribe(
            local_path_topic_, 1, &HighwayPlannerNode::localPathCallback, this);
        //차량 위치와 heading
        odom_sub_ = nh_.subscribe(
            odom_topic_, 10, &HighwayPlannerNode::odomCallback, this);
        // 현재 속도
        speed_sub_ = nh_.subscribe(
            speed_topic_, 10, &HighwayPlannerNode::speedCallback, this);
        // 브릿지에서 차량 상태가 정상적으로 들어오는지
        status_valid_sub_ = nh_.subscribe(
            status_valid_topic_, 1, &HighwayPlannerNode::statusValidCallback, this);
        current_path_sub_ = nh_.subscribe(
            current_path_topic_, 1, &HighwayPlannerNode::currentPathCallback, this);
        //선행차량까지의 거리와 상대속도를 계산
        objects_sub_ = nh_.subscribe(
            objects_topic_, 10, &HighwayPlannerNode::objectsCallback, this);

        target_path_pub_ = nh_.advertise<nav_msgs::Path>(target_path_topic_, 1);
        target_speed_pub_ = nh_.advertise<std_msgs::Float64>(target_speed_topic_, 1);
        hard_speed_limit_pub_ =
            nh_.advertise<std_msgs::Float64>(hard_speed_limit_topic_, 1);
        zone_state_pub_ = nh_.advertise<std_msgs::String>(zone_state_topic_, 1);
        behavior_state_pub_ = nh_.advertise<std_msgs::String>(behavior_state_topic_, 1);
        emergency_pub_ = nh_.advertise<std_msgs::Bool>(emergency_topic_, 1);
        current_s_pub_ = nh_.advertise<std_msgs::Float64>(current_s_topic_, 1);
        ttc_pub_ = nh_.advertise<std_msgs::Float64>(ttc_topic_, 1);

        ROS_INFO(
            "[highway_planner] V1 ready: route progress, zone/curve speed, tollgate braking, ACC/TTC");
    }

    void spin()
    {
        ros::Rate rate(rate_hz_);
        ros::Time previous_cycle = ros::Time::now();

        while (ros::ok()) {
            ros::spinOnce();

            const ros::Time now = ros::Time::now();
            const double dt = std::max(0.0, (now - previous_cycle).toSec());
            previous_cycle = now;

            if (!baseInputsReady() ||
                (require_vehicle_status_ && !status_received_)) {
                ROS_INFO_THROTTLE(
                    2.0,
                    "[highway_planner] waiting for paths, current lane, odom, speed and valid vehicle status");
                publishStop(now, "INPUT_NOT_READY");
                rate.sleep();
                continue;
            }

            if (inputsTimedOut(now)) {
                publishStop(now, "STALE_INPUT");
                rate.sleep();
                continue;
            }

            const highway_planner::Projection projection = route_.project(
                //현재 차량 위치 (ego_x_, ego_y_)와 yaw를 사용해 전역 경로에서 가장 가까운 지점 찾기
                ego_x_,
                ego_y_,
                ego_yaw_,
                route_index_,
                projection_search_radius_,
                relocalization_distance_,
                projection_heading_weight_);
            if (!projection.valid || //기본 허용 거리
                projection.distance > maximum_route_distance_) {
                publishStop(now, "ROUTE_LOST");
                rate.sleep();
                continue;
            }
            route_index_ = projection.index;

            const double curve_speed = route_.curveSpeedLimit(
                route_index_,
                curve_lookahead_distance_,
                curve_planned_deceleration_);
            const highway_planner::ZoneResult zone =
                zone_planner_.evaluate(projection.s, ego_speed_, route_);

            const double free_flow_speed = std::min(
                curve_speed,
                std::min(zone.zone_speed, zone.tollgate_speed));
            const highway_planner::FollowingResult following =
                following_planner_.plan(
                    current_path_,
                    ego_speed_,
                    free_flow_speed,
                    now.toSec());

            const bool objects_stale =
                require_objects_ &&
                (last_objects_stamp_.isZero() ||
                 (now - last_objects_stamp_).toSec() > objects_timeout_);
            const bool emergency = following.emergency || objects_stale;

            double target_speed = 0.0;
            double hard_speed_limit = 0.0;
            std::string behavior = "STOP";
            if (!emergency) {
                hard_speed_limit = following.target_speed;
                target_speed = speed_filter_.update(
                    zone.zone_speed,
                    hard_speed_limit,
                    dt);
                behavior = following.following ? "FOLLOW" : "CRUISE";
            } else {
                speed_filter_.reset(0.0);
            }

            publish(
                now,
                projection.s,
                zone,
                behavior,
                target_speed,
                hard_speed_limit,
                following.ttc,
                emergency);
            rate.sleep();
        }
    }

private:
    highway_planner::FollowingConfig loadFollowingConfig()
    {
        highway_planner::FollowingConfig config;
        pnh_.param(
            "leader/maximum_distance",
            config.maximum_distance,
            config.maximum_distance);
        pnh_.param("leader/hold_time", config.hold_time, config.hold_time);
        pnh_.param(
            "leader/switch_distance",
            config.switch_distance,
            config.switch_distance);
        pnh_.param(
            "leader/relative_speed_alpha",
            config.relative_speed_alpha,
            config.relative_speed_alpha);
        pnh_.param(
            "leader/maximum_relative_speed",
            config.maximum_relative_speed,
            config.maximum_relative_speed);

        pnh_.param(
            "acc/standstill_gap",
            config.acc.standstill_gap,
            config.acc.standstill_gap);
        pnh_.param(
            "acc/time_headway",
            config.acc.time_headway,
            config.acc.time_headway);
        pnh_.param("acc/gap_gain", config.acc.gap_gain, config.acc.gap_gain);
        pnh_.param(
            "acc/relative_speed_gain",
            config.acc.relative_speed_gain,
            config.acc.relative_speed_gain);
        pnh_.param(
            "acc/response_time",
            config.acc.response_time,
            config.acc.response_time);
        pnh_.param(
            "acc/maximum_acceleration",
            config.acc.maximum_acceleration,
            config.acc.maximum_acceleration);
        pnh_.param(
            "acc/comfortable_deceleration",
            config.acc.comfortable_deceleration,
            config.acc.comfortable_deceleration);
        pnh_.param(
            "acc/emergency_deceleration",
            config.acc.emergency_deceleration,
            config.acc.emergency_deceleration);
        pnh_.param(
            "acc/minimum_gap",
            config.acc.minimum_gap,
            config.acc.minimum_gap);
        pnh_.param(
            "acc/emergency_ttc",
            config.acc.emergency_ttc,
            config.acc.emergency_ttc);
        return config;
    }

    highway_planner::SpeedFilterConfig loadSpeedFilterConfig()
    {
        highway_planner::SpeedFilterConfig config;
        pnh_.param(
            "speed_filter/maximum_acceleration",
            config.maximum_acceleration,
            config.maximum_acceleration);
        pnh_.param(
            "speed_filter/maximum_deceleration",
            config.maximum_deceleration,
            config.maximum_deceleration);
        pnh_.param(
            "speed_filter/maximum_jerk",
            config.maximum_jerk,
            config.maximum_jerk);
        pnh_.param(
            "speed_filter/response_time",
            config.response_time,
            config.response_time);
        return config;
    }

    void loadParams()
    {
        pnh_.param("rate_hz", rate_hz_, 20.0);
        pnh_.param("global_path_topic", global_path_topic_, std::string("/global_path1"));
        pnh_.param("local_path_topic", local_path_topic_, std::string("/local_path1"));
        pnh_.param("odom_topic", odom_topic_, std::string("/gps_utm_odom"));
        pnh_.param("speed_topic", speed_topic_, std::string("/current_speed"));
        pnh_.param(
            "current_path_topic",
            current_path_topic_,
            std::string("/current_path"));
        pnh_.param(
            "status_valid_topic",
            status_valid_topic_,
            std::string("/vehicle_status_valid"));
        pnh_.param(
            "objects_topic",
            objects_topic_,
            std::string("/obstacle_path_info"));

        pnh_.param(
            "target_path_topic",
            target_path_topic_,
            std::string("/highway/target_path"));
        pnh_.param(
            "target_speed_topic",
            target_speed_topic_,
            std::string("/highway/target_speed"));
        pnh_.param(
            "hard_speed_limit_topic",
            hard_speed_limit_topic_,
            std::string("/highway/hard_speed_limit"));
        pnh_.param(
            "zone_state_topic",
            zone_state_topic_,
            std::string("/highway/zone_state"));
        pnh_.param(
            "behavior_state_topic",
            behavior_state_topic_,
            std::string("/highway/behavior_state"));
        pnh_.param(
            "emergency_topic",
            emergency_topic_,
            std::string("/highway/emergency_stop"));
        pnh_.param(
            "current_s_topic",
            current_s_topic_,
            std::string("/highway/current_s"));
        pnh_.param("ttc_topic", ttc_topic_, std::string("/highway/ttc"));

        pnh_.param("require_vehicle_status", require_vehicle_status_, true);
        pnh_.param("require_objects", require_objects_, false);
        pnh_.param("input_timeout", input_timeout_, 0.5);
        pnh_.param("local_path_timeout", local_path_timeout_, 0.5);
        pnh_.param("objects_timeout", objects_timeout_, 0.5);
        pnh_.param(
            "expected_local_path_frame",
            expected_local_path_frame_,
            std::string("base_link"));

        pnh_.param("route/circular", route_circular_, true);
        pnh_.param("route/curvature_half_window", curvature_half_window_, 10);
        pnh_.param("route/max_lateral_accel", max_lateral_accel_, 2.5);
        pnh_.param("route/maximum_speed", route_maximum_speed_, 25.0);
        pnh_.param(
            "route/maximum_allowed_curvature",
            maximum_allowed_curvature_,
            0.2);
        pnh_.param("route/projection_search_radius", projection_search_radius_, 400);
        pnh_.param("route/relocalization_distance", relocalization_distance_, 8.0);
        pnh_.param("route/maximum_route_distance", maximum_route_distance_, 10.0);
        pnh_.param("route/projection_heading_weight", projection_heading_weight_, 5.0);
        pnh_.param("route/curve_lookahead_distance", curve_lookahead_distance_, 100.0);
        pnh_.param(
            "route/curve_planned_deceleration",
            curve_planned_deceleration_,
            2.5);

        pnh_.param("zone/enabled", zone_config_.enabled, false);
        pnh_.param("zone/entry_s", zone_config_.entry_s, 0.0);
        pnh_.param("zone/high_speed_start_s", zone_config_.high_speed_start_s, 0.0);
        pnh_.param("zone/brake_start_s", zone_config_.brake_start_s, 0.0);
        pnh_.param("zone/limit_start_s", zone_config_.limit_start_s, 0.0);
        pnh_.param("zone/approach_distance", zone_config_.approach_distance, 30.0);
        pnh_.param("zone/limited_speed", zone_config_.limited_speed, 16.0);
        pnh_.param("zone/highway_speed", zone_config_.highway_speed, 25.0);
        pnh_.param(
            "zone/planned_deceleration",
            zone_config_.planned_deceleration,
            2.5);
        pnh_.param("zone/limit_margin", zone_config_.limit_margin, 10.0);
        pnh_.param("zone/system_delay", zone_config_.system_delay, 0.3);

        rate_hz_ = std::max(1.0, rate_hz_);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        std::vector<highway_planner::Point2D> points;
        points.reserve(msg->poses.size());
        for (const auto& pose : msg->poses) {
            points.push_back(
                {pose.pose.position.x, pose.pose.position.y});
        }

        if (!route_.setPath(
                points,
                route_circular_,
                curvature_half_window_,
                max_lateral_accel_,
                route_maximum_speed_)) {
            ROS_ERROR("[highway_planner] invalid global path");
            route_ready_ = false;
            return;
        }

        double maximum_curvature = 0.0;
        int maximum_curvature_index = -1;
        for (int index = 0;
             index < static_cast<int>(route_.points().size());
             ++index) {
            const double curvature =
                std::fabs(route_.points()[index].curvature);
            if (curvature > maximum_curvature) {
                maximum_curvature = curvature;
                maximum_curvature_index = index;
            }
        }
        if (maximum_curvature > maximum_allowed_curvature_) {
            ROS_ERROR(
                "[highway_planner] route rejected: curvature %.4f 1/m at index=%d s=%.2f exceeds %.4f 1/m",
                maximum_curvature,
                maximum_curvature_index,
                route_.points()[maximum_curvature_index].s,
                maximum_allowed_curvature_);
            route_ready_ = false;
            return;
        }

        std::string zone_error;
        if (!zone_planner_.configure(zone_config_, route_, zone_error)) {
            ROS_ERROR(
                "[highway_planner] invalid zone configuration: %s",
                zone_error.c_str());
            route_ready_ = false;
            return;
        }

        route_index_ = -1;
        route_ready_ = true;
        ROS_INFO(
            "[highway_planner] route loaded: points=%zu length=%.2f m zone=%s",
            points.size(),
            route_.length(),
            zone_config_.enabled ? "enabled" : "disabled (limited speed)");
    }

    void localPathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (msg->poses.size() < 2) {
            local_path_ready_ = false;
            ROS_ERROR_THROTTLE(1.0, "[highway_planner] local path has fewer than two points");
            return;
        }
        if (msg->header.frame_id != expected_local_path_frame_) {
            local_path_ready_ = false;
            ROS_ERROR_THROTTLE(
                1.0,
                "[highway_planner] expected local path frame '%s', got '%s'",
                expected_local_path_frame_.c_str(),
                msg->header.frame_id.c_str());
            return;
        }

        local_path_ = *msg;
        last_local_path_stamp_ = ros::Time::now();
        local_path_ready_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ego_x_ = msg->pose.pose.position.x;
        ego_y_ = msg->pose.pose.position.y;
        ego_yaw_ = tf::getYaw(msg->pose.pose.orientation);
        last_odom_stamp_ = ros::Time::now();
        odom_ready_ = true;
    }

    void speedCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        ego_speed_ = std::max(0.0, msg->data);
        last_speed_stamp_ = ros::Time::now();
        if (!speed_ready_) {
            speed_filter_.reset(ego_speed_);
        }
        speed_ready_ = true;
    }

    void statusValidCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        vehicle_status_valid_ = msg->data;
        status_received_ = true;
    }

    void currentPathCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        if (msg->data != 1 && msg->data != 2) {
            current_path_ready_ = false;
            following_planner_.reset();
            ROS_WARN_THROTTLE(
                1.0,
                "[highway_planner] current path must be 1 or 2, got %d",
                msg->data);
            return;
        }

        if (current_path_ready_ && current_path_ != msg->data) {
            following_planner_.reset();
        }
        current_path_ = msg->data;
        current_path_ready_ = true;
    }

    void objectsCallback(const katri_msgs::Objects::ConstPtr& msg)
    {
        const ros::Time now = ros::Time::now();
        last_objects_stamp_ = now;

        std::vector<highway_planner::ObjectObservation> observations;
        observations.reserve(msg->objects.size());
        for (const auto& object : msg->objects) {
            observations.push_back(
                {object.path_number, object.distance});
        }

        if (current_path_ready_) {
            following_planner_.observe(
                observations,
                current_path_,
                now.toSec());
        }
    }

    bool baseInputsReady() const
    {
        return route_ready_ &&
               local_path_ready_ &&
               odom_ready_ &&
               speed_ready_ &&
               current_path_ready_;
    }

    bool inputsTimedOut(const ros::Time& now) const
    {
        return (now - last_odom_stamp_).toSec() > input_timeout_ ||
               (now - last_speed_stamp_).toSec() > input_timeout_ ||
               (now - last_local_path_stamp_).toSec() > local_path_timeout_ ||
               (require_vehicle_status_ && !vehicle_status_valid_);
    }

    void publish(const ros::Time& now,
                 const double current_s,
                 const highway_planner::ZoneResult& zone,
                 const std::string& behavior,
                 const double target_speed,
                 const double hard_speed_limit,
                 const double ttc,
                 const bool emergency)
    {
        nav_msgs::Path target_path = local_path_;
        target_path.header.stamp = now;
        target_path_pub_.publish(target_path);

        std_msgs::Float64 speed_msg;
        speed_msg.data = target_speed;
        target_speed_pub_.publish(speed_msg);

        std_msgs::Float64 hard_speed_limit_msg;
        hard_speed_limit_msg.data = hard_speed_limit;
        hard_speed_limit_pub_.publish(hard_speed_limit_msg);

        std_msgs::String zone_msg;
        zone_msg.data = highway_planner::toString(zone.state);
        zone_state_pub_.publish(zone_msg);

        std_msgs::String behavior_msg;
        behavior_msg.data = behavior;
        behavior_state_pub_.publish(behavior_msg);

        std_msgs::Bool emergency_msg;
        emergency_msg.data = emergency;
        emergency_pub_.publish(emergency_msg);

        std_msgs::Float64 current_s_msg;
        current_s_msg.data = current_s;
        current_s_pub_.publish(current_s_msg);

        std_msgs::Float64 ttc_msg;
        ttc_msg.data = ttc;
        ttc_pub_.publish(ttc_msg);

        ROS_INFO_THROTTLE(
            1.0,
            "[highway_planner] s=%.1f zone=%s behavior=%s speed=%.2f limit=%.2f m/s ttc=%.2f emergency=%d",
            current_s,
            zone_msg.data.c_str(),
            behavior.c_str(),
            target_speed,
            hard_speed_limit,
            ttc,
            emergency ? 1 : 0);
    }

    void publishStop(const ros::Time& now, const std::string& reason)
    {
        speed_filter_.reset(0.0);
        highway_planner::ZoneResult zone;
        publish(
            now,
            route_index_ >= 0 ? route_.points()[route_index_].s : 0.0,
            zone,
            "STOP_" + reason,
            0.0,
            0.0,
            0.0,
            true);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber local_path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber speed_sub_;
    ros::Subscriber status_valid_sub_;
    ros::Subscriber current_path_sub_;
    ros::Subscriber objects_sub_;
    ros::Publisher target_path_pub_;
    ros::Publisher target_speed_pub_;
    ros::Publisher hard_speed_limit_pub_;
    ros::Publisher zone_state_pub_;
    ros::Publisher behavior_state_pub_;
    ros::Publisher emergency_pub_;
    ros::Publisher current_s_pub_;
    ros::Publisher ttc_pub_;

    std::string global_path_topic_;
    std::string local_path_topic_;
    std::string odom_topic_;
    std::string speed_topic_;
    std::string current_path_topic_;
    std::string status_valid_topic_;
    std::string objects_topic_;
    std::string target_path_topic_;
    std::string target_speed_topic_;
    std::string hard_speed_limit_topic_;
    std::string zone_state_topic_;
    std::string behavior_state_topic_;
    std::string emergency_topic_;
    std::string current_s_topic_;
    std::string ttc_topic_;

    highway_planner::RouteModel route_;
    highway_planner::ZoneConfig zone_config_;
    highway_planner::ZonePlanner zone_planner_;
    highway_planner::FollowingPlanner following_planner_;
    highway_planner::SpeedCommandFilter speed_filter_;
    nav_msgs::Path local_path_;

    double rate_hz_ = 20.0;
    bool require_vehicle_status_ = true;
    bool require_objects_ = false;
    double input_timeout_ = 0.5;
    double local_path_timeout_ = 0.5;
    double objects_timeout_ = 0.5;
    std::string expected_local_path_frame_ = "base_link";

    bool route_circular_ = true;
    int curvature_half_window_ = 10;
    double max_lateral_accel_ = 2.5;
    double route_maximum_speed_ = 25.0;
    double maximum_allowed_curvature_ = 0.2;
    int projection_search_radius_ = 400;
    double relocalization_distance_ = 8.0;
    double maximum_route_distance_ = 10.0;
    double projection_heading_weight_ = 5.0;
    double curve_lookahead_distance_ = 100.0;
    double curve_planned_deceleration_ = 2.5;

    bool route_ready_ = false;
    bool local_path_ready_ = false;
    bool odom_ready_ = false;
    bool speed_ready_ = false;
    bool current_path_ready_ = false;
    bool status_received_ = false;
    bool vehicle_status_valid_ = false;

    double ego_x_ = 0.0;
    double ego_y_ = 0.0;
    double ego_yaw_ = 0.0;
    double ego_speed_ = 0.0;
    int current_path_ = 0;
    int route_index_ = -1;
    ros::Time last_odom_stamp_;
    ros::Time last_speed_stamp_;
    ros::Time last_local_path_stamp_;
    ros::Time last_objects_stamp_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "highway_planner");
    HighwayPlannerNode node;
    node.spin();
    return 0;
}
