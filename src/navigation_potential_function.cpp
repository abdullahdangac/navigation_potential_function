#include "navigation_potential_function.hpp"


void NavigationController::calc_gamma_beta(int numOfRobots, int numOfParts, double partDist, double A, double B, std::vector<std::vector<double>> b_, std::vector<std::vector<double>> bt_, std::vector<std::vector<double> > bp, double ro, int rID)
{
    // GAMA - robot_i and goal_i
    for(int i = 0 ; i < numOfRobots; ++i)
    {
        if (b_[i][2]!=0)  // sensing range control
        {
            // (b_i_x - g_i_x)^2 + (b_i_y - g_i_y)^2
            double dist = pow((b_[i][0] - bt_[i][0]), 2) + pow((b_[i][1] - bt_[i][1]), 2);

            //mpfr_set_d(temp, dist, MPFR_RND);
            //mpfr_add(A, A, temp, MPFR_RND);
        }
    }


    // BETA - robot_i (main robot) and robot_j (other robots)
    for(int i = 0; i < numOfRobots; ++i)
    {
        if (i != rID)
        {
            double dist = -1;

            if (b_[i][2]!=0)  // sensing range control
            {
                // (b_j_x - b_i_x)^2 + (b_j_y - b_i_y)^2 - (b_j_radius + b_i_radius)^2
                dist = abs(pow((b_[rID][0] - b_[i][0]), 2) + pow((b_[rID][1] - b_[i][1]), 2) - pow((b_[rID][2] + b_[i][2]), 2));
            }

            if (dist > 0)
            {
                //mpfr_set_d(temp, dist, MPFR_RND);
                //mpfr_mul(B, B, temp, MPFR_RND);
            }
        }
    }


    // BETA - robot_j (another robot) and robot_k (other robots)
    for(int j = 0; j < numOfRobots; ++j)
    {
        for(int k = j + 1; k < numOfRobots; ++k)
        {
            if ( (j != rID) && (k != rID))
            {
                double dist = -1;

                if ((b_[j][2] != 0) && (b_[k][2] != 0))  // sensing range control
                {
                    // (b_j_x - b_k_x)^2 + (b_j_y - b_k_y)^2 - (b_j_radius + b_k_radius)^2
                    dist = abs(pow((b_[j][0] - b_[k][0]), 2) + pow((b_[j][1] - b_[k][1]), 2) - pow((b_[j][2] + b_[k][2]), 2));
                }

                if (dist > 0)
                {
                    //mpfr_set_d(temp, dist, MPFR_RND);
                    //mpfr_mul(B, B, temp, MPFR_RND);
                }
            }
        }
    }

    //parts
    for(int j = 0; j < numOfParts; ++j)
    {
        double dist = abs(pow((b_[rID][0] - bp[j][0]),2) + pow((b_[rID][1] - bp[j][1]),2) - pow((b_[rID][2]+bp[j][2]), 2));

        if (dist < partDist * partDist)
        {
            //mpfr_set_d(temp, dist, MPFR_RND);
            //mpfr_mul(B, B, temp, MPFR_RND);
        }
    }


    // BETA - robot_i and boundry
    for(int i = 0; i <= numOfRobots; ++i)
    {
        double dist = -1;

        if (b_[i][2] != 0)  // sensing range control
        {
            // (r_boundry - b_i_radius)^2 - (b_i_x^2 + b_i_y^2)
            dist = abs(pow((ro - b_[i][2]), 2) - pow(b_[i][0], 2) - pow(b_[i][1], 2));
        }

        if (dist > 0)
        {
            //mpfr_set_d(temp, dist, MPFR_RND);
            //mpfr_mul(B, B, temp, MPFR_RND);
        }
     }

     //mpfr_clear (temp);
}

void NavigationController::calc_Y(int num_of_robots, double &Y, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal)
{
    // any robot ~ its goal
    for(int a = 0; a < num_of_robots; ++a)
    {
        if (robot[a][2] != 0)  // sensing range control
        {
            // (robot_x - goal_x)^2 + (robot_y - goal_y)^2
            double dist = pow((robot[a][0] - goal[a][0]), 2) + pow((robot[a][1] - goal[a][1]), 2);

            Y += dist;
            //mpfr_set_d(temp, dist, MPFR_RND);
            //mpfr_add(A, A, temp, MPFR_RND);
        }
    }
}

void NavigationController::calc_B(int num_of_robots, int num_of_obstacles, double obstacle_dist, double &B, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> obstacle, double r_boundry, int robot_ID)
{
    // robot_i ~ robot_j (other robots)
    for(int a = 0; a < num_of_robots; ++a)
    {
        if (a != robot_ID)
        {
            double dist = -1;

            if (robot[a][2] !=0 )  // sensing range control
            {
                // (robot_i_x - robot_j_x)^2 + (robot_i_y - robot_j_y)^2 - (robot_i_radius + robot_j_radius)^2
                dist = abs(pow((robot[robot_ID][0] - robot[a][0]), 2) + pow((robot[robot_ID][1] - robot[a][1]), 2) - pow((robot[robot_ID][2] + robot[a][2]), 2));
            }

            if (dist > 0)
            {
                B *= dist;
                //mpfr_set_d(temp, dist, MPFR_RND);
                //mpfr_mul(B, B, temp, MPFR_RND);
            }
        }
    }


    // robot_j (another robot) ~ robot_k (other robots) (B_j_k)
    for(int a = 0; a < num_of_robots; ++a)
    {
        for(int b = a + 1; b < num_of_robots; ++b)
        {
            if ((a != robot_ID) && (b != robot_ID))
            {
                double dist = -1;

                if ((robot[a][2] != 0) && (robot[b][2] != 0))  // sensing range control
                {
                    // (robot_j_x - robot_k_x)^2 + (robot_j_y - robot_k_y)^2 - (robot_j_radius + robot_k_radius)^2
                    dist = abs(pow((robot[a][0] - robot[b][0]), 2) + pow((robot[a][1] - robot[b][1]), 2) - pow((robot[a][2] + robot[b][2]), 2));
                }

                if (dist > 0)
                {
                    B *= dist;
                    //mpfr_set_d(temp, dist, MPFR_RND);
                    //mpfr_mul(B, B, temp, MPFR_RND);
                }
            }
        }
    }


    // any robot ~ boundry (B_o_i)
    for(int a = 0; a <= num_of_robots; ++a)
    {
        double dist = -1;

        if (robot[a][2] != 0)  // sensing range control
        {
            // (r_boundry - robot_radius)^2 - (robot_x^2 + robot_y^2)
            dist = abs(pow((r_boundry - robot[a][2]), 2) - pow(robot[a][0], 2) - pow(robot[a][1], 2));
        }

        if (dist > 0)
        {
            B *= dist;
            //mpfr_set_d(temp, dist, MPFR_RND);
            //mpfr_mul(B, B, temp, MPFR_RND);
        }
     }


    // robot_i ~ obstacles
    for(int a = 0; a < num_of_obstacles; ++a)
    {
        // (robot_i_x - obstacle_x)^2 + (robot_i_y - obstacle_y)^2 - (robot_i_radius + obstacle_radius)^2
        double dist = abs(pow((robot[robot_ID][0] - obstacle[a][0]), 2) + pow((robot[robot_ID][1] - obstacle[a][1]), 2) - pow((robot[robot_ID][2] + obstacle[a][2]), 2));

        if (dist < obstacle_dist * obstacle_dist)
        {
            B *= dist;
            //mpfr_set_d(temp, dist, MPFR_RND);
            //mpfr_mul(B, B, temp, MPFR_RND);
        }
    }

     //mpfr_clear (temp);
}

void NavigationController::calc_dY_dx(double &dY_dx, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, int robot_ID)
{
    // gamma = (robot_x - goal_x)^2 + (robot_y - goal_y)^2
    // dgamma/dx = 2 * (robot_x - goal_x)
    dY_dx = 2 * (robot[robot_ID][0] - goal[robot_ID][0]);

    //mpfr_set_d(dAdx, 2*(b_[rID][1]-bt_[rID][1]), MPFR_RND);
}

void NavigationController::calc_dY_dy(double &dY_dy, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, int robot_ID)
{
    // gamma = (robot_x - goal_x)^2 + (robot_y - goal_y)^2
    // dgamma/dy = 2 * (robot_y - goal_y)
    dY_dy = 2 * (robot[robot_ID][1] - goal[robot_ID][1]);

    //mpfr_set_d(dAdy, 2*(b_[rID][2]-bt_[rID][2]), MPFR_RND);
}

void NavigationController::calc_dB_dx(int num_of_robots, int num_of_obstacles, double obstacle_dist, double &dB_dx, double B, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> obstacle, double r_boundry, int robot_ID)
{
    //mpfr_t temp;
    //mpfr_init (temp);

    //mpfr_set_d(dBdx, 0.0, MPFR_RND);

    // robot_i ~ robot_j
    for(int a = 0; a < num_of_robots; ++a)
    {
        if (a != robot_ID)
        {
            double val;
            if (robot[a][2] != 0)  // sensing range control
            {
                //                                 2 * (robot_i_x - robot_j_x)
                // ---------------------------------------------------------------------------------------------
                //  (robot_i_x - robot_j_x)^2 + (robot_i_y - robot_j_y)^2 - (robot_i_radius + robot_j_radius)^2
                val = 2 * (robot[robot_ID][0] - robot[a][0]) / (pow((robot[robot_ID][0] - robot[a][0]), 2) + pow((robot[robot_ID][1] - robot[a][1]), 2) - pow((robot[robot_ID][2] + robot[a][2]), 2));
                
                dB_dx += val;
                //mpfr_set_d(temp, val, MPFR_RND);
                //mpfr_add(dBdx, dBdx, temp, MPFR_RND);
            }
        }
    }

    // robot_i ~ boundry
    //                     -2 * robot_i_x
    // --------------------------------------------------------------
    //  (r_boundry - robot_i_radius)^2 + (robot_i_x^2 - robot_i_y^2)
    double val = (-2 * robot[robot_ID][0]) / (pow((r_boundry - robot[robot_ID][2]), 2) - robot[robot_ID][0] * robot[robot_ID][0] - robot[robot_ID][1] * robot[robot_ID][1]); // bounded
  
    dB_dx += val;
    //mpfr_set_d(temp, val, MPFR_RND);
    //mpfr_add(dBdx, dBdx, temp, MPFR_RND);

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
            //mpfr_set_d(temp, val, MPFR_RND);
            //mpfr_add(dBdx, dBdx, temp, MPFR_RND);
        }
    }

    dB_dx = dB_dx * B;
    //mpfr_mul(dBdx, dBdx, B, MPFR_RND);

    //mpfr_clear (temp);
}

void NavigationController::calc_dB_dy(int num_of_robots, int num_of_obstacles, double obstacle_dist, double &dB_dy, double B, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> obstacle, double r_boundry, int robot_ID)
{
    // mpfr_t temp;
    // mpfr_init (temp);

    // mpfr_set_d(dBdy, 0.0, MPFR_RND);

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
                // mpfr_set_d(temp, val, MPFR_RND);
                // mpfr_add(dBdy, dBdy, temp, MPFR_RND);
            }
        }
    }

    // robot_i ~ boundry
    //                     -2 * robot_i_y
    // --------------------------------------------------------------
    //  (r_boundry - robot_i_radius)^2 + (robot_i_x^2 - robot_i_y^2)
    double val = (-2 * robot[robot_ID][1]) / (pow((r_boundry - robot[robot_ID][2]), 2) - robot[robot_ID][0] * robot[robot_ID][0] - robot[robot_ID][1] * robot[robot_ID][1]); // bounded
    
    dB_dy += val;
    // mpfr_set_d(temp, val, MPFR_RND);
    // mpfr_add(dBdy, dBdy, temp, MPFR_RND);

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
            // mpfr_set_d(temp, val, MPFR_RND);
            // mpfr_add(dBdy, dBdy, temp, MPFR_RND);
        }
    }

    dB_dy = dB_dy * B;
    // mpfr_mul(dBdy, dBdy, B, MPFR_RND);

    // mpfr_clear (temp);
}

void NavigationController::calc_dF_dx(double &dF_dx, double &dQ_dx, double Y, double B, int kk, int kq, double dY_dx, double dB_dx)
{
    double gammak, gammakB, temp, temp1, temp2, temp3, temp4, one, two, kk_m, kq_m, minusOne;
    // mpfr_t gammak, gammakB, temp, temp1, temp2, temp3, temp4, one, two, kk_m, kq_m, minusOne;

    // mpfr_init (gammak);
    // mpfr_init (gammakB);
    // mpfr_init (temp);
    // mpfr_init (temp1);
    // mpfr_init (temp2);
    // mpfr_init (temp3);
    // mpfr_init (temp4);
    // mpfr_init (one);
    // mpfr_init (two);
    // mpfr_init (minusOne);
    // mpfr_init (kk_m);
    // mpfr_init (kq_m);

    one = 1.0;
    two = 2.0;
    minusOne = -1.0;
    kk_m = kk;
    kq_m = kq;
    // mpfr_set_d(one, 1.0, MPFR_RND);
    // mpfr_set_d(two, 2.0, MPFR_RND);
    // mpfr_set_d(minusOne, -1.0, MPFR_RND);
    // mpfr_set_d(kk_m, kk, MPFR_RND);
    // mpfr_set_d(kq_m, kq, MPFR_RND);

    /////  new  /////
    double Q, dQ_dY, dQ_dB;
    // mpfr_t Q, dQdA, dQdB;
    // mpfr_init (Q);
    // mpfr_init (dQdA);
    // mpfr_init (dQdB);


    // Q = math.pow(A, Ak) / B;
    //      gamma^k
    // q = ---------
    //        beta
    Q = pow(Y, kk_m) / B;
    // mpfr_pow(Q, A, kk_m, MPFR_RND);
    // mpfr_div(Q, Q, B, MPFR_RND);

    // dQdA = Ak*Math.pow(A,Ak-1)/B;
    //    dq         gamma^k-1
    // -------- = k -----------
    //  dgamma          beta
    dQ_dY = kk_m * pow(Y, kk_m-1) / B;
    // mpfr_set_d(temp, kk-1, MPFR_RND);
    // mpfr_pow(dQdA, A, temp, MPFR_RND);
    // mpfr_mul(dQdA, dQdA, kk_m, MPFR_RND);
    // mpfr_div(dQdA, dQdA, B, MPFR_RND);

    // dQdB = -1.0*Math.pow(A,Ak)/(B*B);
    //   dq          gamma^k
    // ------- = -1 ---------
    //  dbeta         beta^2
    dQ_dB = -1.0 * pow(Y, kk_m) / (B * B);
    // mpfr_div(dQdB, Q, B, MPFR_RND);
    // mpfr_mul(dQdB, dQdB, minusOne, MPFR_RND);

    // dQdx = dQdA*dAdx+dQdB*dBdx;
    //  dQ       dQ       dgamma      dQ       dbeta
    // ---- = -------- . -------- + ------- . -------
    //  dx     dgamma       dx       dbeta       dx
    dQ_dx = dQ_dY * dY_dx + dQ_dB * dB_dx;
    // mpfr_mul(temp1, dQdA, dAdx, MPFR_RND);
    // mpfr_mul(temp2, dQdB, dBdx, MPFR_RND);
    // mpfr_add(dQdx, temp1, temp2, MPFR_RND);

    // dFdx = (1/kq)*(Q/(Q+1))^((1/kq)-1)*(Q+1)^(-2)*dQdx;
    //  dF      1       /   Q   \  (1 / kq) - 1                dQ
    // ---- = ------ . | ------- |              . (1 + Q)^2 . ----
    //  dx     kq_m     \ 1 + Q /                              dx
    dF_dx = (1 / kq_m) * pow((Q / (Q + 1)), ((1 / kq) - 1)) * pow((Q + 1), (-2)) * dQ_dx;
    // mpfr_set_d(temp, ((1.0/kq)-1.0), MPFR_RND);
    // mpfr_add(temp1, Q, one, MPFR_RND);
    // mpfr_div(temp1, Q, temp1, MPFR_RND);
    // mpfr_pow(temp1, temp1, temp, MPFR_RND);

    // mpfr_set_d(temp, -2.0, MPFR_RND);
    // mpfr_add(temp2, Q, one, MPFR_RND);
    // mpfr_pow(temp2, temp2, temp, MPFR_RND);

    // mpfr_mul(dFdx, temp1, temp2, MPFR_RND);
    // mpfr_mul(dFdx, dFdx, dQdx, MPFR_RND);
    // mpfr_div(dFdx, dFdx, kq_m, MPFR_RND);

    // mpfr_clear (gammak);
    // mpfr_clear (gammakB);
    // mpfr_clear (temp);
    // mpfr_clear (temp1);
    // mpfr_clear (temp2);
    // mpfr_clear (temp3);
    // mpfr_clear (temp4);
    // mpfr_clear (one);
    // mpfr_clear (minusOne);
    // mpfr_clear (two);
    // mpfr_clear (kk_m);
    // mpfr_clear (kq_m);
    // mpfr_clear (Q);
    // mpfr_clear (dQdA);
    // mpfr_clear (dQdB);
}

void NavigationController::calc_dF_dy(double &dF_dy, double &dQ_dy, double Y, double B, int kk, int kq, double dY_dy, double dB_dy)
{
//dFdy = (1/kq)*(Q/(Q+1))^((1/kq)-1)*(Q+1)^(-2)*dQdy;

  //gammak = A^kk;
  //gammakB = (gammak+B)^(1/kk);
  //dFdy = (1/gammakB^2)*(gammakB*dAdy - (A/kk)*gammakB^(1-kk)*(kk*A^(kk-1)*dAdy+ dBdy));

    double gammak, gammakB, temp, temp1, temp2, temp3, temp4, one, two, kk_m, kq_m, minusOne;
    //mpfr_t gammak, gammakB, temp, temp1, temp2, temp3, temp4, one, two, kk_m, kq_m, minusOne;

    // mpfr_init (gammak);
    // mpfr_init (gammakB);
    // mpfr_init (temp);
    // mpfr_init (temp1);
    // mpfr_init (temp2);
    // mpfr_init (temp3);
    // mpfr_init (temp4);
    // mpfr_init (one);
    // mpfr_init (two);
    // mpfr_init (minusOne);
    // mpfr_init (kk_m);
    // mpfr_init (kq_m);

    // for mpfr
    one = 1.0;
    two = 2.0;
    minusOne = -1.0;
    kk_m = kk;
    kq_m = kq;
    // mpfr_set_d(one, 1.0, MPFR_RND);
    // mpfr_set_d(two, 2.0, MPFR_RND);
    // mpfr_set_d(minusOne, -1.0, MPFR_RND);
    // mpfr_set_d(kk_m, kk, MPFR_RND);
    // mpfr_set_d(kq_m, kq, MPFR_RND);


    /////  new  /////
    double Q, dQ_dY, dQ_dB;
    // mpfr_t Q, dQdA, dQdB;
    // mpfr_init (Q);
    // mpfr_init (dQdA);
    // mpfr_init (dQdB);


    // Q = Math.pow(A,Ak)/B;
    Q = pow(Y, kk_m) / B;
    // mpfr_pow(Q, A, kk_m, MPFR_RND);
    // mpfr_div(Q, Q, B, MPFR_RND);

    // dQdA = Ak*Math.pow(A,Ak-1)/B;
    dQ_dY = kk_m * pow(Y, kk_m - 1) / B;
    // mpfr_set_d(temp, kk-1, MPFR_RND);
    // mpfr_pow(dQdA, A, temp, MPFR_RND);
    // mpfr_mul(dQdA, dQdA, kk_m, MPFR_RND);
    // mpfr_div(dQdA, dQdA, B, MPFR_RND);

    // dQdB = -1.0*Math.pow(A,Ak)/(B*B);
    dQ_dB = -1.0 * pow(Y, kk_m) / (B * B);
    // mpfr_div(dQdB, Q, B, MPFR_RND);
    // mpfr_mul(dQdB, dQdB, minusOne, MPFR_RND);

    // dAdy = dAdx();
    // dBdy = dBdx();

    // dQdy = dQdA*dAdy+dQdB*dBdy;
    //  dQ       dQ       dgamma      dQ       dbeta
    // ---- = -------- . -------- + ------- . -------
    //  dy     dgamma       dy       dbeta       dy
    dQ_dy = dQ_dY * dY_dy + dQ_dB * dB_dy;
    // mpfr_mul(temp1, dQdA, dAdy, MPFR_RND);
    // mpfr_mul(temp2, dQdB, dBdy, MPFR_RND);
    // mpfr_add(dQdy, temp1, temp2, MPFR_RND);

    // dFdy = (1/kq)*(Q/(Q+1))^((1/kq)-1)*(Q+1)^(-2)*dQdy;
    //  dF      1       /   Q   \  (1 / kq) - 1                dQ
    // ---- = ------ . | ------- |              . (1 + Q)^2 . ----
    //  dx     kq_m     \ 1 + Q /                              dy
    dF_dy = (1 / kq_m) * pow((Q / (Q + 1)), ((1 / kq) - 1)) * pow((Q + 1), (-2)) * dQ_dy;
    // mpfr_set_d(temp, ((1.0/kq)-1.0), MPFR_RND);
    // mpfr_add(temp1, Q, one, MPFR_RND);
    // mpfr_div(temp1, Q, temp1, MPFR_RND);
    // mpfr_pow(temp1, temp1, temp, MPFR_RND);

    // mpfr_set_d(temp, -2.0, MPFR_RND);
    // mpfr_add(temp2, Q, one, MPFR_RND);
    // mpfr_pow(temp2, temp2, temp, MPFR_RND);

    // mpfr_mul(dFdy, temp1, temp2, MPFR_RND);
    // mpfr_mul(dFdy, dFdy, dQdy, MPFR_RND);
    // mpfr_div(dFdy, dFdy, kq_m, MPFR_RND);

    // mpfr_clear (gammak);
    // mpfr_clear (gammakB);
    // mpfr_clear (temp);
    // mpfr_clear (temp1);
    // mpfr_clear (temp2);
    // mpfr_clear (temp3);
    // mpfr_clear (temp4);
    // mpfr_clear (one);
    // mpfr_clear (minusOne);
    // mpfr_clear (two);
    // mpfr_clear (kk_m);
    // mpfr_clear (kq_m);
    // mpfr_clear (Q);
    // mpfr_clear (dQdA);
    // mpfr_clear (dQdB);
}

void NavigationController::robot_controller(double bout[], double *bout_length_of_vel, int num_of_robots, int num_of_obstacles, double obstacle_dist, std::vector<std::vector<double>> robot, std::vector<std::vector<double>> goal, std::vector<std::vector<double>> obstacle, double r_boundry, double kk_limits[], int robot_ID)
{
    double Y, B;
    // mpfr_t A, B, dBdx, dBdy, dAdx, dAdy, dFdx, dFdy, norm, temp1, temp2,minusOne, two;
    // mpfr_t logA_m, logB_m;// divlogBlogA;
    // mpfr_rnd_t MPFR_RND;
    // MPFR_RND = mpfr_get_default_rounding_mode();

    // int kk, kq;
    // double logA, logB;

    //mpfr_set_default_prec(53);

    // mpfr_init (A);
    // mpfr_init (B);
    // mpfr_init (logA_m);
    // mpfr_init (logB_m);
    // mpfr_init (dAdx);
    // mpfr_init (dAdy);
    // mpfr_init (dBdx);
    // mpfr_init (dBdy);
    // mpfr_init (dFdx);
    // mpfr_init (dFdy);
    // mpfr_init (norm);
    // mpfr_init (temp1);
    // mpfr_init (temp2);
    // mpfr_init (minusOne);
    // mpfr_init (two);


    calc_Y(num_of_robots, Y, robot, goal);
    calc_B(num_of_robots, num_of_obstacles, obstacle_dist, B, robot, obstacle, r_boundry, robot_ID);
    // fcn_A_B(numOfRobots, numOfParts, partDist, A, B, bin_, bt_, b_rs_, bp, ro, rID);


    /*
    printf("\n");
    mpfr_out_str (stdout, 10, 0, A, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, B, MPFR_RND);
    printf("\n");
*/
    int kk;
    double log_Y, log_B;

    log_Y = log10(Y);
    log_B = log10(B);
    kk = ceil(log_Y / log_B);
    kk = abs(kk + (kk % 2));
    // mpfr_log10(logA_m, A, MPFR_RND);
    // mpfr_log10(logB_m, B, MPFR_RND);
    // logA = mpfr_get_d(logA_m, MPFR_RND);
    // logB = mpfr_get_d(logB_m, MPFR_RND);
    // //mpfr_div(divlogBlogA, logB, logA, MPFR_RND);
    // //kk = ceil(mpfr_get_d(divlogBlogA, MPFR_RND));
    // kk = ceil(logB/logA);
    // kk = abs(kk + (kk%2));

    /*
    if (kk<0)
    {
        printf("\n");
        mpfr_out_str (stdout, 10, 0, A, MPFR_RND);
        printf("\n");
        mpfr_out_str (stdout, 10, 0, B, MPFR_RND);
        printf("\n");
        mpfr_out_str (stdout, 10, 0, logA_m, MPFR_RND);
        printf("\n");
        mpfr_out_str (stdout, 10, 0, logB_m, MPFR_RND);
        printf("\n");
    }
    */

    if (kk < kk_limits[0])
    {
        kk = kk_limits[0];
    }
    else
    {
        if (kk > kk_limits[1])
            kk = kk_limits[1];
    }

/*
    if ( (kk<(numOfRobots + numOfRobots*(numOfRobots-1)/2)))
    {
        int temp;
        temp = numOfRobots + numOfRobots*(numOfRobots-1)/2;
        kk = abs(temp + (temp%2));
    }

*/
    int kq = kk;
    double dY_dx, dY_dy, dB_dx, dB_dy;

    calc_dY_dx(dY_dx, robot, goal, robot_ID);
    calc_dY_dy(dY_dy, robot, goal, robot_ID);
    calc_dB_dx(num_of_robots, num_of_obstacles, obstacle_dist, dB_dx, B, robot, obstacle, r_boundry, robot_ID);
    calc_dB_dy(num_of_robots, num_of_obstacles, obstacle_dist, dB_dy, B, robot, obstacle, r_boundry, robot_ID);
    // fcn_dAdx(dAdx, MPFR_RND, bin_, bt_, rID);
    // fcn_dBdx(numOfRobots, numOfParts, partDist, dBdx, B, MPFR_RND, bin_, b_rs_, bp, ro, rID);
    // fcn_dAdy(dAdy, MPFR_RND, bin_, bt_, rID);
    // fcn_dBdy(numOfRobots, numOfParts, partDist, dBdy, B, MPFR_RND, bin_, b_rs_, bp, ro, rID);


    /*fcn_dFdx(dFdx, A, B, kk, kq, dAdx, dBdx, MPFR_RND);
    fcn_dFdy(dFdy, A, B, kk, kq, dAdy, dBdy, MPFR_RND);*/

    double dF_dx, dF_dy, dQ_dx, dQ_dy;
    // mpfr_t dQdx,dQdy;
    // mpfr_init(dQdx);
    // mpfr_init(dQdy);

    calc_dF_dx(dF_dx, dQ_dx, Y, B, kk, kq, dY_dx, dB_dx);
    calc_dF_dy(dF_dy, dQ_dy, Y, B, kk, kq, dY_dy, dB_dy);
    // fcn_dFdx(dFdx, dQdx, A, B, kk, kq, dAdx, dBdx, MPFR_RND);
    // fcn_dFdy(dFdy, dQdy, A, B, kk, kq, dAdy, dBdy, MPFR_RND);

    double norm;

    norm = log10(sqrt(pow(dQ_dx, 2) + pow(dQ_dy, 2)));
    *bout_length_of_vel = norm;
    // mpfr_set_d(two, 2.0, MPFR_RND);
    // mpfr_pow(temp1, dQdx, two, MPFR_RND);
    // mpfr_pow(temp2, dQdy, two, MPFR_RND);
    // mpfr_add(norm, temp1, temp2, MPFR_RND);
    // mpfr_sqrt(norm, norm, MPFR_RND);
    // mpfr_log10(norm, norm, MPFR_RND);
    // *boutLengthOfVel  = mpfr_get_d(norm, MPFR_RND);
/*
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdx, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdy, MPFR_RND);
    printf("\n");
*/

    norm = sqrt(pow(dF_dx, 2) + pow(dF_dy, 2));
    // mpfr_set_d(two, 2.0, MPFR_RND);
    // mpfr_pow(temp1, dFdx, two, MPFR_RND);
    // mpfr_pow(temp2, dFdy, two, MPFR_RND);
    // mpfr_add(norm, temp1, temp2, MPFR_RND);
    // mpfr_sqrt(norm, norm, MPFR_RND);

    dF_dx =  dF_dx / norm;
    dF_dy =  dF_dy / norm;
    // mpfr_div(dFdx, dFdx, norm, MPFR_RND);
    // mpfr_div(dFdy, dFdy, norm, MPFR_RND);

    bout[0] = -1 * dF_dx;
    bout[1] = -1 * dF_dy;
    // bout[0] = -1.0*mpfr_get_d(dFdx, MPFR_RND);
    // bout[1] = -1.0*mpfr_get_d(dFdy, MPFR_RND);

    // mpfr_clear (dQdx);
    // mpfr_clear (dQdy);
    // mpfr_clear (A);
    // mpfr_clear (B);
    // mpfr_clear (logA_m);
    // mpfr_clear (logB_m);
    // mpfr_clear (dAdx);
    // mpfr_clear (dAdy);
    // mpfr_clear (dBdx);
    // mpfr_clear (dBdy);
    // mpfr_clear (dFdx);
    // mpfr_clear (dFdy);
    // mpfr_clear (norm);
    // mpfr_clear (temp1);
    // mpfr_clear (temp2);
    // mpfr_clear (minusOne);
    // mpfr_clear (two);
}