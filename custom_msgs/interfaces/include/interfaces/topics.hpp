#pragma once
#include <string>
using string = std::string;

namespace topics {

struct perception {
    inline static const string BASE                  = "/perception";
    inline static const string CONES                 = BASE + "/cones";
    inline static const string UNDISTORTED_CLOUD     = BASE + "/undistorted_cloud";
    inline static const string GROUND_FILTERED_CLOUD = BASE + "/ground_filtered_cloud";
    inline static const string GROUND_CLOUD          = BASE + "/ground_cloud";
    inline static const string CLUSTERED_CLOUD       = BASE + "/clustered_cloud";
    inline static const string CONE_COUNT            = BASE + "/cone_count";
};

struct slam {
    inline static const string BASE                      = "/slam";
    inline static const string MAP                       = BASE + "/map";
    inline static const string VEHICLE_STATE             = BASE + "/vehicle_state";
    inline static const string LAP_COUNT                 = BASE + "/lap_count";
    inline static const string CONE_COUNT                = BASE + "/cone_count";
    inline static const string DRIVEN_PATH               = BASE + "/driven_path";
    inline static const string OPTIMIZE                  = BASE + "/optimize";
    inline static const string PRELOADED_MAP             = BASE + "/preloaded_map";
    inline static const string MAP_LOADED                = BASE + "/map_loaded";
    inline static const string LANDMARK_COV_VIZ          = BASE + "/landmark_cov_viz";
    inline static const string DATA_ASSOCIATION_AREA_VIZ = BASE + "/data_association_area_viz";
    inline static const string DATA_ASSOCIATION_VIZ      = BASE + "/data_association_viz";
};

struct estimation {
    inline static const string BASE                      = "/estimation";
    inline static const string VELOCITY                  = BASE + "/velocity";
    inline static const string DEBUG                     = "/debug";
    inline static const string DEBUG_BUTTER_GSS          = BASE + DEBUG + "/butter_gss";
    inline static const string DEBUG_BUTTER_IMU          = BASE + DEBUG + "/butter_imu";
    inline static const string DEBUG_EKF_VELOCITY        = BASE + DEBUG + "/ekf_velocity";
    inline static const string DEBUG_SLIP_RATIO          = BASE + DEBUG + "/slip_ratio";
    inline static const string DEBUG_ADAPTED_WHEEL_SPEED = BASE + DEBUG + "/adapted_wheel_speed";
    inline static const string DEBUG_WHEEL_SPEED_COG     = BASE + DEBUG + "/wheel_speed_cog";
};

struct planning {
    inline static const string BASE             = "/planning";
    inline static const string TRIANGLES        = BASE + "/triangles";
    inline static const string MIDDLE_LINE      = BASE + "/middle_line";
    inline static const string TRACK_BOUNDARIES = BASE + "/track_boundaries";
    inline static const string RACING_LINE      = BASE + "/racing_line";
    inline static const string TRAJECTORY       = BASE + "/trajectory";

    // Velocity Planning
    inline static const string FRICTION_MAP       = BASE + "/friction_map";
    inline static const string MU_SCALING         = BASE + "/mu_scaling";
    inline static const string REQUEST_MU_SCALING = BASE + "/request_mu_scaling";
    inline static const string FINAL_MU_SCALING   = BASE + "/final_mu_scaling";
    inline static const string STABILITY_CRITERIA = BASE + "/stability_criteria";
};

struct controller {
    inline static const string BASE                               = "/controller";
    inline static const string TORQUE_REQUEST                     = BASE + "/torque_request";
    inline static const string STEERING_REQUEST                   = BASE + "/steering_request";
    inline static const string TARGET_WHEEL_SPEEDS                = BASE + "/target_wheel_speeds";
    inline static const string PATH_ERROR                         = BASE + "/path_error";
    inline static const string ORIENTATION_ERROR                  = BASE + "/orientation_error";
    inline static const string YAW_RATE_ERROR                     = BASE + "/yaw_rate_error";
    inline static const string VELOCITY_ERROR                     = BASE + "/velocity_error";
    inline static const string WHEEL_SLIP_ANGLES                  = BASE + "/wheel_slip_angles";
    inline static const string WHEEL_LOADS                        = BASE + "/wheel_loads";
    inline static const string S_VEHICLE_ON_PATH                  = BASE + "/s_vehicle_on_path";
    inline static const string DEBUG                              = BASE + "/debug";
    inline static const string DEBUG_TARGET_VELOCITY              = DEBUG + "/target_velocity";
    inline static const string DEBUG_TARGET_ACCELERATION          = DEBUG + "/target_acceleration";
    inline static const string DEBUG_TARGET_ACCELERATION_UNSCALED = DEBUG + "/target_acceleration_unscaled";
    inline static const string DEBUG_ACCELERATION_SCALE_FACTOR    = DEBUG + "/acceleration_scale_factor";
    inline static const string DEBUG_TARGET_SLIP_RATIO            = DEBUG + "/target_slip_ratio";
    inline static const string DEBUG_LONG_CONTROL_KP              = DEBUG + "/long_control_kp";
    inline static const string DEBUG_LONG_CONTROL_KI              = DEBUG + "/long_control_ki";
    inline static const string DEBUG_FEED_FORWARD_ACCELERATION    = DEBUG + "/feed_forward_acceleration";
    inline static const string DEBUG_STEERING_RATE                = DEBUG + "/steering_rate";
    inline static const string DEBUG_MPC_PREDICTION               = DEBUG + "/mpc_prediction";
    inline static const string DEBUG_REFERENCE_YAW_RATE           = DEBUG + "/reference_yaw_rate";
    inline static const string DEBUG_TARGET_ORIENTATION           = DEBUG + "/target_orientation";
    inline static const string DEBUG_TARGET_YAW_MOMENT            = DEBUG + "/target_yaw_moment";
    inline static const string DEBUG_OCA_ACCEL_DIRECT             = DEBUG + "/oca_accel_direct";
    inline static const string DEBUG_OCA_YAW_DIRECT               = DEBUG + "/oca_yaw_direct";
    inline static const string DEBUG_OCA_POWER_DIRECT             = DEBUG + "/oca_power_direct";
    inline static const string DEBUG_OCA_MOTOR_POWER_DIRECT       = DEBUG + "/oca_motor_power_direct";
    inline static const string DEBUG_MAX_TORQUE_TC                = DEBUG + "/max_torque_TC";
    inline static const string DEBUG_MAX_TORQUE_GRIP              = DEBUG + "/max_torque_grip";
    inline static const string DEBUG_MAX_TORQUE_MOTOR_POWER       = DEBUG + "/max_torque_motor_power";
    inline static const string DEBUG_MAX_TORQUE_MAX_TORQUE_PARAM  = DEBUG + "/max_torque_max_torque_param";
    inline static const string DEBUG_POINT_ON_PATH                = DEBUG + "/point_on_path";
};

struct sensing {
    inline static const string BASE             = "/sensing";
    inline static const string LIDAR            = BASE + "/lidar_points";
    inline static const string GSS              = BASE + "/gss";
    inline static const string IMU              = BASE + "/imu";
    inline static const string GPS              = BASE + "/gps";
    inline static const string WHEEL_ENCODER    = BASE + "/wheel_encoder";
    inline static const string ACTUAL_TORQUE    = BASE + "/actual_torque";
    inline static const string MAX_TORQUE       = BASE + "/max_torque";
    inline static const string STEERING_ANGLE   = BASE + "/steering_angle";
    inline static const string POWER_DRAW       = BASE + "/power_draw";
    inline static const string TIRE_TEMPERATURE = BASE + "/tire_temperature";
    inline static const string SPRING_TRAVEL    = BASE + "/spring_travel";
};

struct system {
    inline static const string BASE              = "/system";
    inline static const string PARAMETERS        = BASE + "/parameters";
    inline static const string MISSION           = BASE + "/mission";
    inline static const string ACTUATION_ALLOWED = BASE + "/actuation_allowed";
    inline static const string SUPERVISOR_STOP   = BASE + "/supervisor_stop";
    inline static const string HEARTBEAT         = BASE + "/heartbeat";
    inline static const string AS_STATUS         = BASE + "/as_status";
    inline static const string AS_STATUS_REQUEST = BASE + "/as_status_request";
    inline static const string MCU_HEARTBEAT     = BASE + "/mcu_heartbeat";
    inline static const string HARDWARE_STATUS   = BASE + "/hardware_status";
};

struct simulator {
    inline static const string BASE                  = "/simulation";
    inline static const string GROUND_TRUTH          = BASE + "/ground_truth";
    inline static const string CONES_HIT_COUNT       = BASE + "/cones_hit_count";
    inline static const string LAP_COUNT             = BASE + "/lap_count";
    inline static const string LAP_TIME              = BASE + "/lap_time";
    inline static const string TRACK_VISUALIZATION   = BASE + "/track_visualization";
    inline static const string VEHICLE_VISUALIZATION = BASE + "/vehicle_visualization";
};

struct rviz_plugin {
    inline static const string BASE            = "/rviz_plugin";
    inline static const string MISSION_REQUEST = BASE + "/mission_request";
    inline static const string RESET_REQUEST   = BASE + "/reset_request";
    inline static const string TRACK_REQUEST   = BASE + "/track_request";
};

struct visualization {
    inline static const string BASE               = "/visualization";
    inline static const string TRIANGLES          = BASE + "/triangles";
    inline static const string ACTIVE_TRIANGLES   = BASE + "/active_triangles";
    inline static const string DRIVEN_PATH        = BASE + "/driven_path";
    inline static const string MIDDLE_LINE        = BASE + "/middle_line";
    inline static const string TRACK_BOUNDARIES   = BASE + "/track_boundaries";
    inline static const string RACING_LINE        = BASE + "/racing_line";
    inline static const string APEXES             = BASE + "/apexes";
    inline static const string MIN_TIME_DEBUG     = BASE + "/min_time_debug";
    inline static const string TRAJECTORY         = BASE + "/trajectory";
    inline static const string VEL_PROFILE_LIMITS = BASE + "/vel_profile_limits";
    inline static const string FRICTION_MAP       = BASE + "/friction_map";
    inline static const string CLIPPING_SHAPES    = BASE + "/clipping_shapes";
    inline static const string VEHICLE_STATE_HIST = BASE + "/vehicle_state_hist";
    inline static const string DFS                = BASE + "/dfs";
};

struct tf {
    inline static const string WORLD            = "world";
    inline static const string SLAM_VEHICLE_COG = "slam/vehicle_cog";
    inline static const string SIM_VEHICLE_COG  = "simulator/vehicle_cog";
    inline static const string SIM_LIDAR        = "simulator/lidar";
    inline static const string LIDAR            = "pandar64";
};

}  // namespace topics
