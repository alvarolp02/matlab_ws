#pragma once
#include "interfaces/msg/middle_line.hpp"
#include "interfaces/msg/racing_line.hpp"
#include "interfaces/msg/trajectory.hpp"
#include "interfaces/msg/vehicle_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "shared_interfaces/msg/wheel_data.hpp"

#include <Eigen/Dense>
#include <vector>

namespace interfaces {

struct VehicleState {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
    double vx  = 0.0;
    double vy  = 0.0;
    double r   = 0.0;

    VehicleState() = default;

    VehicleState(const interfaces::msg::VehicleState::SharedPtr msg) {
        x   = msg->x;
        y   = msg->y;
        yaw = msg->yaw;
        vx  = msg->vx;
        vy  = msg->vy;
        r   = msg->r;
    }

    interfaces::msg::VehicleState to_msg() const {
        interfaces::msg::VehicleState msg;
        msg.x   = x;
        msg.y   = y;
        msg.yaw = yaw;
        msg.vx  = vx;
        msg.vy  = vy;
        msg.r   = r;
        return msg;
    }
};

struct IMUData {
    double ax         = 0.0;
    double ay         = 0.0;
    double az         = 0.0;
    double roll_rate  = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate   = 0.0;

    IMUData() = default;

    IMUData(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ax         = msg->linear_acceleration.x;
        ay         = msg->linear_acceleration.y;
        az         = msg->linear_acceleration.z;
        roll_rate  = msg->angular_velocity.x;
        pitch_rate = msg->angular_velocity.y;
        yaw_rate   = msg->angular_velocity.z;
    }

    sensor_msgs::msg::Imu to_msg() const {
        sensor_msgs::msg::Imu msg;
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        msg.angular_velocity.x    = roll_rate;
        msg.angular_velocity.y    = pitch_rate;
        msg.angular_velocity.z    = yaw_rate;
        return msg;
    }
};

struct WheelData {
    double fl = 0.0;
    double fr = 0.0;
    double rl = 0.0;
    double rr = 0.0;

    WheelData() = default;

    WheelData(double fl, double fr, double rl, double rr) : fl(fl), fr(fr), rl(rl), rr(rr) {}

    WheelData(const shared_interfaces::msg::WheelData::SharedPtr msg) {
        fl = msg->front_left;
        fr = msg->front_right;
        rl = msg->rear_left;
        rr = msg->rear_right;
    }

    Eigen::Array4d to_vector() const { return Eigen::Array4d(fl, fr, rl, rr); }

    shared_interfaces::msg::WheelData to_msg() const {
        shared_interfaces::msg::WheelData msg;
        msg.front_left  = fl;
        msg.front_right = fr;
        msg.rear_left   = rl;
        msg.rear_right  = rr;
        return msg;
    }

    double sum() const { return fl + fr + rl + rr; }
    double mean() const { return (fl + fr + rl + rr) / 4.0; }
};

struct MiddleLine {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> left_boundary_x;
    std::vector<double> left_boundary_y;
    std::vector<double> right_boundary_x;
    std::vector<double> right_boundary_y;
    double              length    = 0.0;
    bool                is_closed = false;

    MiddleLine() = default;

    MiddleLine(const interfaces::msg::MiddleLine::SharedPtr msg) {
        x                = msg->x;
        y                = msg->y;
        left_boundary_x  = msg->left_boundary_x;
        left_boundary_y  = msg->left_boundary_y;
        right_boundary_x = msg->right_boundary_x;
        right_boundary_y = msg->right_boundary_y;
        length           = msg->length;
        is_closed        = msg->is_closed;
    }

    interfaces::msg::MiddleLine to_msg() const {
        interfaces::msg::MiddleLine msg;
        msg.x                = x;
        msg.y                = y;
        msg.left_boundary_x  = left_boundary_x;
        msg.left_boundary_y  = left_boundary_y;
        msg.right_boundary_x = right_boundary_x;
        msg.right_boundary_y = right_boundary_y;
        msg.length           = length;
        msg.is_closed        = is_closed;
        return msg;
    }
};

struct RacingLine {
    // Unique identifier
    int id;

    // Points making up the racing line
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> psi;
    std::vector<double> s;
    std::vector<double> curvature;

    // Indices of apexes
    std::vector<int> apex_indices;

    // Flags
    bool loop_closed;
    bool has_transition_spline;

    // Additional data
    double interpolation_distance;
    bool   significant_change;
    bool   is_final = false;

    RacingLine() = default;

    RacingLine(const interfaces::msg::RacingLine::SharedPtr msg) {
        id        = static_cast<int>(msg->racing_line_id);
        x         = msg->x;
        y         = msg->y;
        psi       = msg->psi;
        s         = msg->s;
        curvature = msg->curvature;
        apex_indices.clear();
        apex_indices.reserve(msg->apex_indices.size());
        for (uint32_t idx : msg->apex_indices) {
            apex_indices.push_back(static_cast<int>(idx));
        }
        loop_closed            = msg->loop_closed;
        has_transition_spline  = msg->has_transition_spline;
        interpolation_distance = msg->interpolation_distance;
        significant_change     = msg->significant_change;
        is_final               = msg->is_final;
    }

    interfaces::msg::RacingLine to_msg() const {
        interfaces::msg::RacingLine msg;
        msg.racing_line_id = static_cast<uint32_t>(id);
        msg.x              = x;
        msg.y              = y;
        msg.psi            = psi;
        msg.s              = s;
        msg.curvature      = curvature;
        msg.apex_indices.clear();
        msg.apex_indices.reserve(apex_indices.size());
        for (int idx : apex_indices) {
            msg.apex_indices.push_back(static_cast<uint32_t>(idx));
        }
        msg.loop_closed            = loop_closed;
        msg.has_transition_spline  = has_transition_spline;
        msg.interpolation_distance = interpolation_distance;
        msg.significant_change     = significant_change;
        msg.is_final               = is_final;
        return msg;
    }
};

struct Trajectory {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> psi;
    std::vector<double> s;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> curvature;

    Trajectory() = default;

    Trajectory(const interfaces::msg::Trajectory::SharedPtr msg) {
        x            = msg->x;
        y            = msg->y;
        psi          = msg->psi;
        s            = msg->s;
        velocity     = msg->velocity;
        acceleration = msg->acceleration;
        curvature    = msg->curvature;
    }

    interfaces::msg::Trajectory to_msg() const {
        interfaces::msg::Trajectory msg;
        msg.x            = x;
        msg.y            = y;
        msg.psi          = psi;
        msg.s            = s;
        msg.velocity     = velocity;
        msg.acceleration = acceleration;
        msg.curvature    = curvature;
        return msg;
    }

    inline size_t size() const { return x.size(); }

    void resize(size_t n) {
        x.resize(n);
        y.resize(n);
        psi.resize(n);
        s.resize(n);
        velocity.resize(n);
        acceleration.resize(n);
        curvature.resize(n);
    }

    void clear() {
        x.clear();
        y.clear();
        psi.clear();
        s.clear();
        velocity.clear();
        acceleration.clear();
        curvature.clear();
    }
};

}  // namespace interfaces
