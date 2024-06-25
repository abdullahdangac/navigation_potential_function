#include "ros_simulation.hpp"


PIDController::PIDController(void):
Kp(0), Ki(0), Kd(0), P(0), I(0), D(0), prev_error(0)
{
    std::cout << "PID controller has been created." <<  std::endl;
}

PIDController::PIDController(double Kp, double Ki, double Kd):
Kp(Kp), Ki(Ki), Kd(Kd), P(0), I(0), D(0), prev_error(0)
{
    std::cout << "PID controller has been created." <<  std::endl;
}

PIDController::~PIDController(void)
{
    std::cout << "PID controller has been deleted." <<  std::endl;
}

void PIDController::set_PID(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    std::cout << "PID controller has been set." <<  std::endl;
}

double PIDController::get_PID_control_input(double error, double dt)
{
    P = Kp * error;
    I = I + Ki * error * dt;
    D = Kd * (error - prev_error) / dt;

    return P + I + D;
}


RosSimulation::RosSimulation() : Node("navigation_potential_function_simulation")
{
    declare_parameter("robot_ID", 0);
    declare_parameter("num_of_robots", 1);
    declare_parameter("num_of_obstacles", 0);
    declare_parameter("obstacle_dist", 0.0);
    declare_parameter("start_position_of_robot", vector<double>{0.0, 0.0});
    declare_parameter("goal_position_of_robot", vector<double>{0.0, 0.0});
    declare_parameter("radius_of_robots", 1.0);
    declare_parameter("radius_of_boundry", 15.0);
    declare_parameter("k", 1.0);

    int robot_ID = this->get_parameter("robot_ID").as_int();
    int num_of_robots = this->get_parameter("num_of_robots").as_int();
    int num_of_obstacles = this->get_parameter("num_of_obstacles").as_int();
    double obstacle_dist = this->get_parameter("obstacle_dist").as_double();
    vector<double> start_position_of_robot = this->get_parameter("start_position_of_robot").as_double_array();
    vector<double> goal_position_of_robot = this->get_parameter("goal_position_of_robot").as_double_array();
    double r_robot = this->get_parameter("radius_of_robots").as_double();
    double r_boundry = this->get_parameter("radius_of_boundry").as_double();
    double k = this->get_parameter("k").as_double();

    // vector<vector<double>> robot(num_of_robots, vector<double>(3, 0.0));
    // vector<vector<double>> goal(num_of_robots, vector<double>(2, 0.0));
    vector<vector<double>> obstacle(num_of_obstacles, vector<double>(3, 0.0));

    // start_position_of_robot.push_back(r_robot);
    // robot.push_back(start_position_of_robot);
    // goal.push_back(goal_position_of_robot);

    // cout << start_position_of_robot[0] << " " << start_position_of_robot[1] << " " << start_position_of_robot[2] << endl;
    // cout << robot[0][0] << " " << robot[0][1] << " " << robot[0][2] << endl;

    vector<vector<double>> robot({{5.5, 5.5, 1.0}});
    vector<vector<double>> goal({{10, 10}});

    robot_navigation.set_navigation(robot_ID, num_of_robots, num_of_obstacles, obstacle_dist, r_boundry, robot, goal, obstacle, k);

    v_pid.set_PID(0.5, 0.0, 0.0);
    w_pid.set_PID(1.0, 0.1, 0.0);

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle" + to_string(robot_ID) +  "/cmd_vel", 10);
    pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle0/pose", 10, std::bind(&RosSimulation::pose_callback, this, placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&RosSimulation::robot_controller, this));
}


RosSimulation::~RosSimulation()
{
    std::cout << "ROS Simulation has been deleted." <<  std::endl;
}


void RosSimulation::pose_callback(const turtlesim::msg::Pose::SharedPtr pose) 
{
    //RCLCPP_INFO(this->get_logger(), "x: %f  y: %f", pose->x, pose->y);
    robot_navigation.robot[0][0] = pose->x;
    robot_navigation.robot[0][1] = pose->y;

    this->robot_heading = pose->theta;
}


void RosSimulation::find_robots_in_range(int robot_ID, int num_of_robots, double sense_dist, vector<vector<double>> robot)
{
    int main_robot_ID = robot_ID;

    for(int robot_ID = 0; robot_ID < num_of_robots; ++robot_ID)
    {
        double dist = sqrt(pow((robot[main_robot_ID][0] - robot[robot_ID][0]), 2) + pow((robot[main_robot_ID][1] - robot[robot_ID][1]), 2));

        if (dist > sense_dist)
        {
            robot[robot_ID][2] = 0;
        }
    }
}


void RosSimulation::robot_controller()
{
    auto start_time = this->now();
    robot_navigation.robot_controller();

    //RCLCPP_INFO(this->get_logger(), "x: %f  y: %f", robot_navigation.robot[0][0], robot_navigation.robot[0][1]);

    // Calculate linear velocity
    double error_dist = sqrt(pow(robot_navigation.robot[robot_navigation.robot_ID][0] - robot_navigation.goal[robot_navigation.robot_ID][0], 2) 
                           + pow(robot_navigation.robot[robot_navigation.robot_ID][1] - robot_navigation.goal[robot_navigation.robot_ID][1], 2));
    
    double linear_velocity = v_pid.get_PID_control_input(error_dist);

    if (linear_velocity > 5.0)
        linear_velocity = 5.0;

    if (error_dist < 0.2)
        linear_velocity = 0.0;

    std::cout << "v: " << linear_velocity << std::endl;

    // Calculate angular velocity
    double desired_angle = atan2(robot_navigation.v_robot[1], robot_navigation.v_robot[0]);
    std::cout << "desired angle: " << desired_angle << std::endl;

    double error_heading = desired_angle - this->robot_heading;

    auto end_time = this->now();
    auto duration = end_time - start_time;
    double dt = duration.seconds();

    double angular_velocity = w_pid.get_PID_control_input(error_heading, dt);

    if (angular_velocity > 2.0)
        angular_velocity = 2.0;

    if (error_heading < 0.1)
        angular_velocity = 0.0;

    std::cout << "w: " << angular_velocity << std::endl << std::endl;

    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;

    cmd_vel_publisher_->publish(cmd_vel);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RosSimulation>());
    return 0;
}