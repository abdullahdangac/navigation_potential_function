#ifndef NAVIGATION_POTENTIAL_FUNCTION
#define NAVIGATION_POTENTIAL_FUNCTION

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <cfloat>
#include <vector>


class NavigationController
{
  public:
    NavigationController();

    //static void fcn_A_B(int numOfRobots, int numOfParts, double partDist, mpfr_t A, mpfr_t B, std::vector<std::vector<double>> b_, std::vector<std::vector<double> > bt_,  std::vector<std::vector<double> > brs_, std::vector<std::vector<double> > bp, double ro,int rID);
    static void calc_gamma_beta(int numOfRobots, int numOfParts, double partDist, double A, double B, std::vector<std::vector<double>> b_, std::vector<std::vector<double>> bt_, std::vector<std::vector<double>> bp, double ro, int rID);
    
    /**
     * Calculates gamma function that encodes Euclidean distance from goal point
     * 
     * @param num_of_robots Number of robots in the workspace
     * @param Y Value of gamma
     * @param robot Pose vectors of robots in the workspace
     * @param goal Position vectors of goals of robots in the workspace
     */
    static void calc_Y(int num_of_robots, double &Y, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal);

    /**
     * Calculates beta function that encodes distance from free space boundry and obstacles
     * 
     * @param num_of_robots Number of robots in the workspace
     * @param num_of_obstacles Number of obstacles in the workspace
     * @param obstacle_dist
     * @param B Value of beta
     * @param robot Pose vectors of robots in the workspace
     * @param obstacle Position vectors of goals of robots in the workspace
     * @param r_boundry Radius of the workspace
     * @param robot_ID Current robot (Robot ID)
     */
    static void calc_B(int num_of_robots, int num_of_obstacles, double obstacle_dist, double &B, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> obstacle, double r_boundry, int robot_ID);

    /**
     * Calculates the derivative of Y (gamma) with respect to x
     * 
     * @param dY_dx
     * @param robot
     * @param goal
     * @param robot_ID
     */
    static void calc_dY_dx(double &dY_dx, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, int robot_ID);

    /**
     * Calculates the derivative of Y (gamma) with respect to y
     * 
     * @param dY_dy
     * @param robot
     * @param goal
     * @param robot_ID
     */
    static void calc_dY_dy(double &dY_dy, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, int robot_ID);

    /**
     * Calculates the derivative of B (beta) with respect to x
     * 
     * @param num_of_robots
     * @param num_of_obstacles
     * @param obstacle_dist
     * @param dB_dx
     * @param B
     * @param robot
     * @param obstacle
     * @param r_boundry
     * @param robot_ID
     */
    static void calc_dB_dx(int num_of_robots, int num_of_obstacles, double obstacle_dist, double &dB_dx, double B, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> obstacle, double r_boundry, int robot_ID);

    /**
     * Calculates the derivative of B (beta) with respect to y
     * 
     * @param num_of_robots
     * @param num_of_obstacles
     * @param obstacle_dist
     * @param dB_dy
     * @param B
     * @param robot
     * @param obstacle
     * @param r_boundry
     * @param robot_ID
     */
    static void calc_dB_dy(int num_of_robots, int num_of_obstacles, double obstacle_dist, double &dB_dy, double B, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> obstacle, double r_boundry, int robot_ID);

    /**
     * Calculates the derivative of F (phi) with respect to x
     * 
     * @param dF_dx
     * @param dQ_dx
     * @param Y
     * @param B
     * @param kk
     * @param kq
     * @param dY_dx
     * @param dB_dx
     */
    static void calc_dF_dx(double &dF_dx, double &dQ_dx, double Y, double B, int kk, int kq, double dY_dx, double dB_dx);

    /**
     * Calculates the derivative of F (phi) with respect to y
     * 
     * @param dF_dy
     * @param dQ_dy
     * @param Y
     * @param B
     * @param kk
     * @param kq
     * @param dY_dy
     * @param dB_dy
     */
    static void calc_dF_dy(double &dF_dy, double &dQ_dy, double Y, double B, int kk, int kq, double dY_dy, double dB_dy);

    //static void robotContoller(double bout[],double* boutLengthOfVel, int numOfRobots, int numOfParts, double partDist, std::vector<std::vector<double>> bin_, std::vector<std::vector<double>> bt_, std::vector<std::vector<double>> bp, double ro, double kkLimits[], int rID);
    static void robot_controller(double bout[], double* bout_length_of_vel, int num_of_robots, int num_of_obstacles, double obstacle_dist, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, std::vector<std::vector<double>> obstacle, double r_boundry, double kk_limits[], int robot_ID);

};


/*

Y  -> gamma
B  -> beta
F  -> phi
Q  -> phi-hat

robot
    robot[...][0] -> x of robot
    robot[...][1] -> y of robot
    robot[...][2] -> radius of robot

goal
    goal[...][0] -> x of goal
    goal[...][1] -> y of goal

obstacle
    obstacle[...][0] -> x of obstacle
    obstacle[...][1] -> y of obstacle


========================================
    Paper Notation
========================================
b -> state vector of each robot [x y]
g -> state vector of each goal point [x y]

b.T     -> [b_x    b_y  ]
  b_i.T -> [b_i_x  b_i_y]
  b_j.T -> [b_j_x  b_j_y]
  b_k.T -> [b_k_x  b_k_y]

p -> radius
  p_i -> radius of robot_i
  p_o -> radius of boundry

b_i -> robot_i -> robot[robot_ID] - main robot
b_j -> robot_j -> other robots
b_k -> robot_k -> other robots for another robot (for beta calculation)

*/

#endif /* NAVIGATION_POTENTIAL_FUNCTION */
