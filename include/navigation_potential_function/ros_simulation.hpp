#ifndef ROS_SIMULATION_HPP
#define ROS_SIMULATION_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

#include "navigation_potential_function.hpp"

using namespace std;


class PIDController {
  public:
    /**
     * Constructor of PID controller with no setting parameter
     */
    PIDController(void);

    /**
     * Constructor of PID controller with setting parameter
     * 
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    PIDController(double Kp, double Ki, double Kd);

    /**
     * Deconstructor of PID controller
     */
    ~PIDController(void);

    /**
     * Sets PID controller
     * 
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    void set_PID(double Kp, double Ki, double Kd);

    /**
     * Calculates PID control input
     * 
     * @param error Error of control signal to target value
     * @param dt Step size (iteriation time)
     */
    double get_PID_control_input(double error, double dt=0.1);

  private:
    double Kp;  //!< Proportional gain
    double Ki;  //!< Integral gain
    double Kd;  //!< Derivative gain
    double P, I, D;  //!< Proportional, integral and derivative calculation variables
    double prev_error;  //!< Previous error for integral
};


class RosSimulation : public rclcpp::Node {
  public:
    /**
     * Constructor of ROS simulation
     */
    RosSimulation();

    /**
     * Deconstructor of ROS simulation
     */
    ~RosSimulation();

  private:
    /**
     * Control function for multi-robot
     */
    void robot_controller();

    /**
     * Pose callback function for first (robotID=0) robot
     */
    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose);

    /**
     * Finds robots in sensing range
     * 
     * @param robot_ID Current robot (Robot ID)
     * @param num_of_robots Number of robots in the workspace
     * @param sense_dist Distanse of sensing
     * @param robot Position vectors of robots in the workspace
     */
    void find_robots_in_range(int robot_ID, int num_of_robots, double sense_dist, vector<vector<double>> robot);

    double robot_heading;  //!< Heading of robot

    PIDController v_pid;  //!< PID controller for linear velocity
    PIDController w_pid;  //!< PID controller for angular velocity

    NavigationPotentialFunction robot_navigation;  //!< Potential function navigation object

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
};


#endif /* ROS_SIMULATION_HPP */