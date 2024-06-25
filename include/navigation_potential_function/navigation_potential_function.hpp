#ifndef NAVIGATION_POTENTIAL_FUNCTION_HPP
#define NAVIGATION_POTENTIAL_FUNCTION_HPP

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <cfloat>
#include <vector>


class NavigationPotentialFunction
{
  public:
    /**
     * Constructor with no parameter
     */
    NavigationPotentialFunction(void);

    /**
     * Constructor with parameters
     * 
     * @param robot_ID Current robot (Robot ID)
     * @param num_of_robots Number of robots in the workspace
     * @param num_of_obstacles Number of obstacles in the workspace
     * @param obstacle_dist
     * @param r_boundry Radius of the workspace
     * @param robot Position vectors of robots in the workspace
     * @param goal Position vectors of goals of robots in the workspace
     * @param obstacle Pose vectors of obstacles in the workspace
     * @param k Tune parameter of navigation function
     */
    NavigationPotentialFunction(int robot_ID, int num_of_robots, int num_of_obstacles, double obstacle_dist, double r_boundry, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, std::vector<std::vector<double>> obstacle, double k);

    /**
     * Sets parameters of potential function navigation
     * 
     * @param robot_ID Current robot (Robot ID)
     * @param num_of_robots Number of robots in the workspace
     * @param num_of_obstacles Number of obstacles in the workspace
     * @param obstacle_dist
     * @param r_boundry Radius of the workspace
     * @param robot Position vectors of robots in the workspace
     * @param goal Position vectors of goals of robots in the workspace
     * @param obstacle Pose vectors of obstacles in the workspace
     * @param k Tune parameter of navigation function
     */
    void set_navigation(int robot_ID, int num_of_robots, int num_of_obstacles, double obstacle_dist, double r_boundry, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, std::vector<std::vector<double>> obstacle, double k);

    /**
     * Control robots with potential function navigaton
     */
    void robot_controller(void);

    /**
     * Calculates gamma function that encodes Euclidean distance from goal point
     */
    void calc_Y(void);

    /**
     * Calculates beta function that encodes distance from free space boundry and obstacles
     */
    void calc_B(void);

    /**
     * Calculates the derivative of Y (gamma) with respect to x
     */
    void calc_dY_dx(void);

    /**
     * Calculates the derivative of Y (gamma) with respect to y
     */
    void calc_dY_dy(void);

    /**
     * Calculates the derivative of B (beta) with respect to x
     */
    void calc_dB_dx(void);

    /**
     * Calculates the derivative of B (beta) with respect to y
     */
    void calc_dB_dy(void);

    /**
     * Calculates Q (phi-hat) function
     */
    void calc_Q(void);

    /**
     * Calculates the derivative of Q (phi-hat) respect to Y (gamma)
     */
    void calc_dQ_dY(void);

    /**
     * Calculates the derivative of Q (phi-hat) respect to B (beta)
     */
    void calc_dQ_dB(void);

    /**
     * Calculates the derivative of F (phi) respect to Q (phi-hat)
     */
    void calc_dF_dQ(void);

    /**
     * Calculates the derivative of F (phi) with respect to x
     */
    void calc_dF_dx(void);

    /**
     * Calculates the derivative of F (phi) with respect to y
     */
    void calc_dF_dy(void);

    /**
     * @see Paper notation
     */
    double Y;  //!< gamma
    double B;  //!< beta
    //double F;  //!< phi
    double Q;  //!< phi-hat

    double dY_dx;  //!< derivative of gamma with respect to x
    double dY_dy;  //!< derivative of gamma with respect to y

    double dB_dx;  //!< derivative of beta with respect to x
    double dB_dy;  //!< derivative of beta with respect to y

    double dQ_dY;  //!< derivative of phi-hat with respect to gamma
    double dQ_dB;  //!< derivative of phi-hat with respect to beta

    double dF_dQ;  //!< derivative of phi with respect to phi-hat

    double dF_dx;  //!< derivative of phi with respect to x
    double dF_dy;  //!< derivative of phi with respect to y

    int robot_ID;  //!< Current (main) robot
    int num_of_robots;  //!< Number of robots in the workspace
    int num_of_obstacles;  //!< Number of obstacles in the workspace

    double obstacle_dist;  //!< 
    double r_boundry;  //!< Radius of the workspace boundry

    std::vector<std::vector<double>> robot;  //!< Pose vectors of robots in the workspace
    std::vector<std::vector<double>> goal;  //!< Position vectors of goals of robots in the workspace
    std::vector<std::vector<double>> obstacle;  //!< Pose vectors of obstacles in the workspace

    double k;  //!< Tune parameter of navigation function
    //double k_limits[2];  //!< Limits of k parameter (k_limits[0]: min limit, k_limits[1]: max limit)

    double v_robot[2];  //!< Linear velocities of the robot (v_robot[0]: x velocity, v_robot[1]: y velocity)
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
    obstacle[...][2] -> radius of obstacle


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

#endif /* NAVIGATION_POTENTIAL_FUNCTION_HPP */
