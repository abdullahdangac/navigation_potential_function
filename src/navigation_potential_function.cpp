#include "navigation_potential_function.hpp"


NavigationPotentialFunction::NavigationPotentialFunction()
{
    std::cout << "Navigation Potential Function has been created." <<  std::endl;
}


NavigationPotentialFunction::NavigationPotentialFunction(int robot_ID, int num_of_robots, int num_of_obstacles, double obstacle_dist, double r_boundry, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, std::vector<std::vector<double>> obstacle, double k):
robot_ID(robot_ID),
num_of_robots(num_of_robots),
num_of_obstacles(num_of_obstacles),
obstacle_dist(obstacle_dist),
r_boundry(r_boundry),
robot(robot),
goal(goal),
obstacle(obstacle),
k(k)
{
    std::cout << "Navigation Potential Function has been created." <<  std::endl;
}


void NavigationPotentialFunction::set_navigation(int robot_ID, int num_of_robots, int num_of_obstacles, double obstacle_dist, double r_boundry, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, std::vector<std::vector<double>> obstacle, double k)
{
    this->robot_ID = robot_ID;
    this->num_of_robots = num_of_robots;
    this->num_of_obstacles = num_of_obstacles;
    this->obstacle_dist = obstacle_dist;
    this->r_boundry = r_boundry;
    this->robot = robot;
    this->goal = goal;
    this->obstacle = obstacle;
    this->k = k;

    std::cout << "Navigation Potential Function has been set." <<  std::endl;
}


void NavigationPotentialFunction::calc_Y(void)
{
    Y = 0.0;

    // any robot ~ its goal
    for(int a = 0; a < num_of_robots; ++a)
    {
        if (robot[a][2] != 0)  // sensing range control
        {
            // gamma = (robot_x - goal_x)^2 + (robot_y - goal_y)^2
            double dist = pow((robot[a][0] - goal[a][0]), 2) + pow((robot[a][1] - goal[a][1]), 2);

            Y += dist;
        }
    }
}


void NavigationPotentialFunction::calc_B(void)
{
    B = 1.0;

    // robot_i ~ robot_j (other robots)
    for(int a = 0; a < num_of_robots; ++a)
    {
        if (a != robot_ID)
        {
            double dist = -1;

            if (robot[a][2] != 0 )  // sensing range control
            {
                // dist = (robot_i_x - robot_j_x)^2 + (robot_i_y - robot_j_y)^2 - (robot_i_radius + robot_j_radius)^2
                dist = abs(pow((robot[robot_ID][0] - robot[a][0]), 2) + pow((robot[robot_ID][1] - robot[a][1]), 2) - pow((robot[robot_ID][2] + robot[a][2]), 2));
            }

            if (dist > 0)
            {
                B *= dist;
            }
        }
    }
 
    // robot_j (another robot) ~ robot_k (other robots)
    for(int a = 0; a < num_of_robots; ++a)
    {
        for(int b = a + 1; b < num_of_robots; ++b)
        {
            if ((a != robot_ID) && (b != robot_ID))
            {
                double dist = -1;

                if ((robot[a][2] != 0) && (robot[b][2] != 0))  // sensing range control
                {
                    // dist = (robot_j_x - robot_k_x)^2 + (robot_j_y - robot_k_y)^2 - (robot_j_radius + robot_k_radius)^2
                    dist = abs(pow((robot[a][0] - robot[b][0]), 2) + pow((robot[a][1] - robot[b][1]), 2) - pow((robot[a][2] + robot[b][2]), 2));
                }

                if (dist > 0)
                {
                    B *= dist;
                }
            }
        }
    }

    // any robot ~ boundry
    for(int a = 0; a < num_of_robots; ++a)
    {
        double dist = -1;

        if (robot[a][2] != 0)  // sensing range control
        {
            // dist = (r_boundry - robot_radius)^2 - (robot_x^2 + robot_y^2)
            dist = abs(pow((r_boundry - robot[a][2]), 2) - pow(robot[a][0], 2) - pow(robot[a][1], 2));
        }

        if (dist > 0)
        {
            B *= dist;
        }
     }

    // robot_i ~ obstacles
    for(int a = 0; a < num_of_obstacles; ++a)
    {
        // dist = (robot_i_x - obstacle_x)^2 + (robot_i_y - obstacle_y)^2 - (robot_i_radius + obstacle_radius)^2
        double dist = abs(pow((robot[robot_ID][0] - obstacle[a][0]), 2) + pow((robot[robot_ID][1] - obstacle[a][1]), 2) - pow((robot[robot_ID][2] + obstacle[a][2]), 2));

        if (dist < obstacle_dist * obstacle_dist)
        {
            B *= dist;
        }
    }
}


void NavigationPotentialFunction::calc_dY_dx(void)
{
    //  dgamma 
    // -------- = 2 * (robot_x - goal_x)
    //    dx
    dY_dx = 2 * (robot[robot_ID][0] - goal[robot_ID][0]);
}


void NavigationPotentialFunction::calc_dY_dy(void)
{
    //  dgamma 
    // -------- = 2 * (robot_y - goal_y)
    //    dy
    dY_dy = 2 * (robot[robot_ID][1] - goal[robot_ID][1]);
}


void NavigationPotentialFunction::calc_dB_dx(void)
{
    dB_dx = 0.0;

    // robot_i ~ robot_j
    for(int a = 0; a < num_of_robots; ++a)
    {
        if (a != robot_ID)
        {
            if (robot[a][2] != 0)  // sensing range control
            {
                //                                 2 * (robot_i_x - robot_j_x)
                // ---------------------------------------------------------------------------------------------
                //  (robot_i_x - robot_j_x)^2 + (robot_i_y - robot_j_y)^2 - (robot_i_radius + robot_j_radius)^2
                double val = 2 * (robot[robot_ID][0] - robot[a][0]) / (pow((robot[robot_ID][0] - robot[a][0]), 2) + pow((robot[robot_ID][1] - robot[a][1]), 2) - pow((robot[robot_ID][2] + robot[a][2]), 2));
                
                dB_dx += val;
            }
        }
    }

    // robot_i ~ boundry
    //                    -2 * robot_i_x
    // -----------------------------------------------------------
    //  r_boundry - robot_i_radius)^2 - robot_i_x^2 - robot_i_y^2
    double val = (-2 * robot[robot_ID][0]) / (pow((r_boundry - robot[robot_ID][2]), 2) - pow(robot[robot_ID][0], 2) - pow(robot[robot_ID][1], 2)); // bounded

    dB_dx += val;
    
    // robot_i ~ obstacles
    for(int a = 0; a < num_of_obstacles; ++a)
    {
        // dist = (robot_i_x - obstacle_x)^2 + (robot_i_y - obstacle_y)^2 - (robot_i_radius + obstacle_radius)^2
        double dist = fabs(pow((robot[robot_ID][0] - obstacle[a][0]), 2) + pow((robot[robot_ID][1] - obstacle[a][1]), 2) - pow((robot[robot_ID][2] + obstacle[a][2]), 2));
        
        if (dist < obstacle_dist * obstacle_dist)
        {
            //                                 2 * (robot_i_x - obstacle_x)
            // ------------------------------------------------------------------------------------------------
            //  (robot_i_x - obstacle_x)^2 + (robot_i_y - obstacle_y)^2 - (robot_i_radius + obstacle_radius)^2
            double val = 2 * (robot[robot_ID][0] - obstacle[a][0]) / (pow((robot[robot_ID][0] - obstacle[a][0]), 2) + pow((robot[robot_ID][1] - obstacle[a][1]), 2) - pow((robot[robot_ID][2] + obstacle[a][2]), 2));
            
            dB_dx += val;
        }
    }

    dB_dx = dB_dx * B;
}


void NavigationPotentialFunction::calc_dB_dy(void)
{
    dB_dy = 0.0;

    // robot_i ~ robot_j
    for(int a = 0; a < num_of_robots; ++a)
    {
        if (a != robot_ID)
        {
            double val;
            if (robot[a][2] != 0)  // sensing range control
            {
                //                                 2 * (robot_i_y - robot_j_y)
                // ---------------------------------------------------------------------------------------------
                //  (robot_i_x - robot_j_x)^2 + (robot_i_y - robot_j_y)^2 - (robot_i_radius + robot_j_radius)^2
                val = 2 * (robot[robot_ID][1] - robot[a][1]) / (pow((robot[robot_ID][0] - robot[a][0]), 2) + pow((robot[robot_ID][1] - robot[a][1]), 2) - pow((robot[robot_ID][2] + robot[a][2]), 2));
                
                dB_dy += val;
            }
        }
    }

    // robot_i ~ boundry
    //                     -2 * robot_i_y
    // -----------------------------------------------------------
    //  r_boundry - robot_i_radius)^2 - robot_i_x^2 - robot_i_y^2
    double val = (-2 * robot[robot_ID][1]) / (pow((r_boundry - robot[robot_ID][2]), 2) - robot[robot_ID][0] * robot[robot_ID][0] - robot[robot_ID][1] * robot[robot_ID][1]); // bounded
    
    dB_dy += val;

    // robot_i ~ obstacles
    for(int a = 0; a < num_of_obstacles; ++a)
    {
        double dist = fabs(pow((robot[robot_ID][0] - obstacle[a][0]), 2) + pow((robot[robot_ID][1] - obstacle[a][1]), 2) - pow((robot[robot_ID][2] + obstacle[a][3]), 2));
        if (dist < obstacle_dist * obstacle_dist)
        {
            //                                 2 * (robot_i_y - obstacle_y)
            // ------------------------------------------------------------------------------------------------
            //  (robot_i_x - obstacle_x)^2 + (robot_i_y - obstacle_y)^2 - (robot_i_radius + obstacle_radius)^2
            double val = 2 * (robot[robot_ID][1] - obstacle[a][1]) / (pow((robot[robot_ID][0] - obstacle[a][0]), 2) + pow((robot[robot_ID][1] - obstacle[a][1]), 2) - pow((robot[robot_ID][2] + obstacle[a][2]), 2));
            
            dB_dy += val;
        }
    }

    dB_dy = dB_dy * B;
}


void NavigationPotentialFunction::calc_Q(void)
{
    //      gamma^k
    // q = ---------
    //        beta
    Q = pow(Y, k) / B;
}


void NavigationPotentialFunction::calc_dQ_dY(void)
{
    //    dq         gamma^(k-1)
    // -------- = k -------------
    //  dgamma           beta
    dQ_dY = k * pow(Y, k-1) / B;
}


void NavigationPotentialFunction::calc_dQ_dB(void)
{
    //   dq          gamma^k
    // ------- = -1 ---------
    //  dbeta         beta^2
    dQ_dB = -1.0 * pow(Y, k) / (B * B);
}


void NavigationPotentialFunction::calc_dF_dQ(void)
{
    //  dF     1     /   Q   \ (1 / k) - 1
    // ---- = --- . | ------- |            . (1 + Q)^-2
    //  dQ     k     \ 1 + Q /
    dF_dQ = (1 / k) * pow((Q / (Q + 1)), ((1 / k) - 1)) * pow((Q + 1), -2);
}


void NavigationPotentialFunction::calc_dF_dx(void)
{
    //  dF     dF       dQ       dgamma     dF      dQ       dbeta
    // ---- = ---- . -------- . -------- + ---- . ------- . -------
    //  dx     dQ     dgamma       dx       dQ     dbeta       dx
    dF_dx = dF_dQ * dQ_dY * dY_dx + dF_dQ * dQ_dB * dB_dx;
}


void NavigationPotentialFunction::calc_dF_dy(void)
{
    //  dF     dF       dQ       dgamma     dF      dQ       dbeta
    // ---- = ---- . -------- . -------- + ---- . ------- . -------
    //  dy     dQ     dgamma       dy       dQ     dbeta       dy
    dF_dy = dF_dQ * dQ_dY * dY_dy + dF_dQ * dQ_dB * dB_dy;
}


void NavigationPotentialFunction::robot_controller(void)
{
    calc_Y();
    calc_B();
    // std::cout << "Y: " << Y << std::endl;
    // std::cout << "B: " << B << std::endl;

    double log_Y = log10(Y);
    double log_B = log10(B);

    int k_calc = ceil(log_Y / log_B);
    k_calc = abs(k_calc + (k_calc % 2));
    k = k_calc;
    // std::cout << "k: " << k << std::endl;

    // if (k < k_limits[0]) {
    //     k = k_limits[0];
    // } else {
    //     if (k > k_limits[1])
    //         k = k_limits[1];
    // }

    calc_dY_dx();
    calc_dY_dy();
    // std::cout << "dY_dx: " << dY_dx << std::endl;
    // std::cout << "dY_dy: " << dY_dy << std::endl;

    calc_dB_dx();
    calc_dB_dy();
    // std::cout << "dB_dx: " << dB_dx << std::endl;
    // std::cout << "dB_dy: " << dB_dy << std::endl;

    calc_Q();
    // std::cout << "Q: " << Q << std::endl;

    calc_dQ_dY();
    calc_dQ_dB();
    // std::cout << "dQ_dY: " << dQ_dY << std::endl;
    // std::cout << "dQ_dB: " << dQ_dB << std::endl;

    calc_dF_dQ();
    // std::cout << "dF_dQ: " << dF_dQ << std::endl;

    calc_dF_dx();
    calc_dF_dy();
    // std::cout << "dF_dx: " << dF_dx << std::endl;
    // std::cout << "dF_dy: " << dF_dy << std::endl;

    double norm = sqrt(pow(dF_dx, 2) + pow(dF_dy, 2));

    dF_dx =  dF_dx / norm;
    dF_dy =  dF_dy / norm;

    if (!std::isnan(dF_dx))
        v_robot[0] = -1 * dF_dx;
    
    if (!std::isnan(dF_dy))
        v_robot[1] = -1 * dF_dy;
    // std::cout << "v_x: " << v_robot[0] << std::endl;
    // std::cout << "v_y: " << v_robot[1] << std::endl;
}