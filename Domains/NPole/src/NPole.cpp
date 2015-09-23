#include <NPole.h>
#include <time.h>

NPole::NPole()
{
    vel_limit = 100;
    len_limit = 1;
    num_links = 1;
    num_states = num_links+1;
    //theta_max = 5.0 * 3.14159;
    //theta_min = -5.0 * 3.14159;
    //theta_max = .20943951 * 4;
    //theta_min = -.20943951 * 4;

    theta_max = 6.28318;
    theta_min = 0;
    for(unsigned int i = 0; i < num_links; i++)
    {
        if(i == 0)
        {
            length.push_back(0.075);
            mass.push_back(0.075);
        }
        if(i == 1)
        {
            length.push_back(.025);
            mass.push_back(0.025);
        }
        //length.push_back(3);

        damp.push_back(1);
    }
    cart_mass = 1.0;

    state.set_size(num_states, 1);
    state_dot.set_size(num_states, 1);
    dt = 0.005;
    srand(time(NULL));

    double sum_l = 0;
    for(unsigned int i = 0; i < length.size(); i++)
    {
        sum_l += length[i];
    }
    fail_y = sum_l * cos(.20943951);
}

void NPole::init()
{
    all_x.clear();
    all_y.clear();
    num_iterations = 0;
    cum_r = 0;
    cycles.clear();
    cycles.push_back(0);
    cycles.push_back(0);
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        double f = (double)rand() / RAND_MAX;
        double start_theta = -.0872664626 + f * (.0872664626 - -.0872664626);
        if(it == state.begin()) *it = 0;
        else *it = 3.14159;
    }
    state_dot.fill(0);
    calculate_pos(false);
}

void NPole::step(int action)
{
    vector<double> a = m_action_mapping[action];
    for(unsigned int i = 0; i < 1; i++)
    {
        calculate_accel(a);
    }
    calculate_pos(false);

}

bool NPole::end_of_episode()
{
    //Pole Balance
    /*
    static int upright = 0;

    bool reset = true;
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        if(it == state.begin())
        {
            if(abs(*it) > 5)
            {
                upright = 0;
                return true;
            }
        }
    }
    int i = 0;
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        if(it == state.begin()) continue;
        if(i == 0)
        {
            if(abs(*it) >= 0.2) return true;
            if(abs(*it) <= 0.02) return false;
        }
        if(i == 1)
        {
            if(abs(*it) >= 0.1) return true;
            if(abs(*it) <= 0.04) return false;
        }
        i++;
    }
    return false;
*/

    //    for(unsigned int i = 0; i < link_pos_y.size(); i++)
    //    {
    //        if(link_pos_y[i] < 0) return true;
    //    }
    //    return false;
//        for(arma::mat::iterator it = state_dot.begin(); it != state_dot.end(); it++)
//        {
//            if(*it > vel_limit * .75) return true;
//            if(*it < -vel_limit * .75) return true;
//            //if(it == state_dot.begin()) continue;
//        }

        if(abs(state(0)) > 5) return true;
        if(abs(cycles[0]) > 1) return true;
        if(abs(cycles[1]) > 1) return true;


//        for(arma::mat::iterator it = state_dot.begin(); it != state_dot.end(); it++)
//        {
//            if(it == state_dot.begin() && *it > 200)
//            {
//                //cout << "Limit" << endl;
//                return true;
//            }
//            if(it == state_dot.begin() && *it < -200)
//            {
//                //cout << "Limit" << endl;
//                return true;
//            }
//            if(*it > vel_limit * .75)
//            {
//                //cout << "Limit" << endl;
//                return true;
//            }
//            if(*it < -vel_limit * .75)
//            {
//                //cout << "Limit" << endl;
//                return true;
//            }
//        }

    return false;
}

vector<double> NPole::get_state()
{
    vector<double> cur_state;
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        cur_state.push_back(*it);
    }
    for(arma::mat::iterator it = state_dot.begin(); it != state_dot.end(); it++)
    {
        if(it != state_dot.begin() && *it > vel_limit) *it = vel_limit;
        if(it != state_dot.begin() && *it < -vel_limit) *it = -vel_limit;
        cur_state.push_back(*it);
    }
    return cur_state;
}

vector<double> NPole::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(-5 * 1.2);
    for(unsigned int i = 0; i < num_links; i++)
    {
        min_ranges.push_back(theta_min);
    }
    min_ranges.push_back(-20);
    for(unsigned int i = 0; i < num_links; i++)
    {
        min_ranges.push_back(-vel_limit);
    }
    return min_ranges;
}

vector<double> NPole::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(5 * 1.2);
    for(unsigned int i = 0; i < num_links; i++)
    {
        max_ranges.push_back(theta_max);
    }
    max_ranges.push_back(20);
    for(unsigned int i = 0; i < num_links; i++)
    {
        max_ranges.push_back(vel_limit);
    }
    return max_ranges;
}

double NPole::get_reward()
{
    // calculate_performance();

    //Balance
    /*
    bool upright = false;
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        if(it == state.begin())
        {
            if(abs(*it) > 5) return -1;
        }
    }

    int i = 0;
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        if(it == state.begin()) continue;
        if(i == 0)
        {
            if(abs(*it) >= 0.2) return -1;
            if(abs(*it) <= 0.02) return 1;
        }
        if(i == 1)
        {
            if(abs(*it) >= 0.1) return -1;
            if(abs(*it) <= 0.04) return 1;
        }
        i++;
    }
    return 1;
*/

    double goal_x_2 = -abs(state(0) - 0);///5.0;
    //double goal_y_2 = -abs(link_pos_y[1] - length[0]);
    double goal_y_2 = cos(state(1));// + cos(state(1));
    //goal_y_2 = (goal_y_2 - - 2.0)/(2.0 - -2.0);

    double dist_2 = sqrt(goal_x_2 * goal_x_2 + goal_y_2 * goal_y_2);

    return goal_y_2 + goal_x_2;


    //        for(unsigned int i = 0; i < link_pos_y.size(); i++)
    //        {
    //            if(abs(link_pos_y[i] - length[i] * (i+1)) > 0.01) good = false;
    //        }
    //        if(!good) return -1;
    //        return 100;
}

void NPole::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;

    int it = 0;
    for(unsigned int i = 2; i < 3; i++)
    {
        double val;
        if(i == 0) val = 50;
        if(i == 1) val = 50;
        if(i == 2) val = 25;
        if(i == 3) val = 5;
        if(i == 4) val = 2;
        vector<double> action;
        action.push_back(val);
        possible_actions.push_back(it);
        action_mapping.insert(make_pair(it++, action));

        action.clear();
        action.push_back(-val);
        possible_actions.push_back(it);
        action_mapping.insert(make_pair(it++, action));
    }
    vector<double> action;
    action.push_back(0);
    possible_actions.push_back(it);
    action_mapping.insert(make_pair(it++, action));
    set_possible_actions(possible_actions, action_mapping);
}

void NPole::calculate_accel(vector<double> action)
{
    //    arma::Mat<double> G(num_links, num_links);
    //    for(unsigned int i = 0; i < num_links; i++)
    //    {
    //        for(unsigned int j = 0; j < num_links; j++)
    //        {
    //            int z = i;
    //            if(i > j) z = i;
    //            if(i < j) z = j;
    //            double sum_mass = 0;
    //            for(unsigned int k = z; k < num_links; k++)
    //            {
    //                sum_mass += mass[k];
    //            }
    //            double tot = sum_mass * length[j] * cos(theta[i] - theta[j]);
    //            G(i,j) = tot;
    //        }
    //    }

    //    arma::Mat<double> F(num_links, 1);

    //    for(unsigned int i = 0; i < num_links; i++)
    //    {
    //        double sum_tot = 0;
    //        for(unsigned int k = 0; k < num_links; k++)
    //        {
    //            int z = k;
    //            if(k > i) z = k;
    //            if(k < i) z = i;
    //            double sum_mass = 0;
    //            for(unsigned int h = z; h < num_links; h++)
    //            {
    //                sum_mass += mass[h];
    //            }
    //            sum_tot += (sum_mass * length[k] * sin(theta[k] - theta[i]) * pow(theta_dot[k],2));
    //        }

    //        double sum_mass = 0;
    //        for(unsigned int k = i; k < num_links; k++)
    //        {
    //            sum_mass += mass[k];
    //        }
    //        double tot = sum_mass * 9.8 * sin(theta[i]);
    //        if(i == 0)
    //        {
    //            tot += 1.0/length[i] * action[i];
    //        }
    //        F(i,0) = sum_tot + tot;
    //    }
    //    arma::Mat<double> theta_ddot = G.i() * F;
    //    //    for(arma::mat::iterator it = theta_ddot.begin(); it != theta_ddot.end(); it++)
    //    //    {
    //    //        if(*it > 1)
    //    //        {
    //    //            *it = 1;
    //    //        }
    //    //        if(*it < -1)
    //    //        {
    //    //            *it = -1;
    //    //        }
    //    //    }


    //    theta_dot = theta_dot + dt * theta_ddot;
    //        for(arma::mat::iterator it = theta_dot.begin(); it != theta_dot.end(); it++)
    //        {
    //            if(*it > vel_limit)
    //            {
    //                *it = vel_limit;
    //            }
    //            if(*it < -vel_limit)
    //            {
    //                *it = -vel_limit;
    //            }
    //        }
    //        theta = theta + dt * theta_dot;


    //Cart

    arma::Mat<double> F = calculate_F();
    arma::Mat<double> G = calculate_G();
    arma::Mat<double> U = calculate_U();

    arma::Mat<double> u(num_states, 1);
    u.fill(0);
    u(0) = action[0];

    arma::Mat<double> T = u - F - U;
    arma::Mat<double> state_ddot = G.i() * T;

//    for(arma::mat::iterator it = state_ddot.begin(); it != state_ddot.end(); it++)
//    {
//        if(it == state_ddot.begin()) continue;
//        if(*it * dt > 20)
//        {
//            //cout << "Limit" << endl;
//            *it = 20 * dt;
//        }
//        if(*it * dt < -20)
//        {
//            //cout << (*it * dt) << endl;
//            //cout << "Limit" << endl;
//            *it = -(20 * dt);
//        }
//    }
    state = state + dt * state_dot;
    state_dot = state_dot + dt * state_ddot;



    //    state_ddot.print();
    //    state.print();
    //    state_dot.print();
    //    F.print();
    //    G.print();
    //    T.print();
    //    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    //    {
    //        if(it == state.begin()) continue;
    //        if(*it >= theta_max)
    //        {
    //            cout << "Did it" << endl;
    //            *it = theta_max + 0.01;
    //        }
    //        if(*it < theta_min)
    //        {
    //            cout << "Did it" << endl;
    //            *it = theta_min + -0.01;
    //        }
    //    }

//        for(arma::mat::iterator it = state_dot.begin(); it != state_dot.end(); it++)
//        {
//            if(it == state_dot.begin() && *it > 50) *it = 50;
//            if(it == state_dot.begin() && *it < -50) *it = -50;
//            if(*it > vel_limit) *it = vel_limit;
//            if(*it < -vel_limit) *it = -vel_limit;
//        }
    int i = 0;
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        if(it == state.begin())
        {
            if(*it > 5) *it = 5.1;
            if(*it < -5) *it = -5.1;
                continue;
        }
        while(*it >= 6.28318)
        {
            cycles[i]++;
            *it = *it - 6.28318;
        }
        while(*it < 0)
        {
            cycles[i]--;
            *it = *it + 6.28318;
        }
        i++;
    }
}

void NPole::calculate_pos(bool output)
{
    link_pos_x.clear();
    link_pos_y.clear();


    for(unsigned int i = 0; i < num_states; i++)
    {
        if(i == 0)
        {
            link_pos_y.push_back(0);
            link_pos_x.push_back(state[i]);
        }
        else if (i == 1)
        {
            link_pos_y.push_back(link_pos_y[i-1] + length[i-1] * cos(state[i]));
            link_pos_x.push_back(link_pos_x[i-1] + length[i-1] * sin(state[i]));
        }
        else
        {
            link_pos_y.push_back(link_pos_y[i-1] + length[i-1] * cos(state[i]));
            link_pos_x.push_back(link_pos_x[i-1] + length[i-1] * sin(state[i]));
        }
        //cout << link_pos_x[i] << "," << link_pos_y[i] << endl;
    }

    //    for(unsigned int i = 0; i < num_links; i++)
    //    {
    //        if(i == 0)
    //        {
    //            link_pos_y.push_back(0 + length[i] * cos(theta[i]));
    //            link_pos_x.push_back(0 + length[i] * sin(theta[i]));
    //        }
    //        else
    //        {
    //            link_pos_y.push_back(link_pos_y[i-1] + length[i] * cos(theta[i]));
    //            link_pos_x.push_back(link_pos_x[i-1] + length[i] * sin(theta[i]));
    //        }
    //        cout << link_pos_x[i] << "," << link_pos_y[i] << endl;
    //        //if(theta[i] < 0 || theta[i] > 360) exit(1);
    //        //if(theta_dot[i] < -10 || theta_dot[i] > 10) exit(1);
    //    }
    all_x.push_back(link_pos_x);
    all_y.push_back(link_pos_y);
    if(output)
    {
        for(unsigned int i = 0; i < all_x.size(); i++)
        {
            for(unsigned int j = 0; j < all_x[i].size(); j++)
            {
                cout << all_x[i][j] << "," << all_y[i][j] << endl;
            }
        }
        state.print();
    }
}

double NPole::sind(double degrees)
{
    double PI = 3.14159265;
    double radians = degrees * PI / 180.0;
    return sin(radians);
}

double NPole::cosd(double degrees)
{
    double PI = 3.14159265;
    double radians = degrees * PI / 180.0;
    return cos(radians);
}

double NPole::get_performance(double steps_left)
{
    return 1;
    int i = 0;
    for(arma::mat::iterator it = state.begin(); it != state.end(); it++)
    {
        if(i == 0)
        {
            if(abs(*it) <= 0.02) return 1;
        }
        if(i == 1)
        {
            if(abs(*it) <= 0.04) return 1;
        }
        i++;
    }
    return 1;

    bool upright = false;
    if(link_pos_y[num_states-1] > fail_y) upright = true;
    if(upright)
    {
        //cout << "GOOOOOOD" << endl;
        //        for(unsigned int i = 0; i < num_links; i++)
        //        {
        //            if(link_pos_y[i] < 0) r += link_pos_y[i] + length[i]*i;
        //            else r += link_pos_y[i] - length[i]*i;
        //        }
        return 1;
        //return r - 2.0;
    }
    return 0;

}

void NPole::calculate_performance()
{

}

NPole::~NPole()
{
}

arma::Mat<double> NPole::calculate_G()
{
    arma::Mat<double> G(num_states, num_states);
    for(unsigned int i = 0; i < num_states; i++)
    {
        for(unsigned int j = 0; j < num_states; j++)
        {
            //Top Left
            if(i == 0 && j == 0)
            {
                double sum_mass = 0;
                for(unsigned int k = 0; k < num_links; k++)
                {
                    sum_mass += mass[k];
                }
                sum_mass += cart_mass;
                G(i,j) = sum_mass;
                continue;
            }

            //Diagnol
            if(i == j)
            {
                double sum_mass = 0;
                for(unsigned int k = i; k < num_states; k++)
                {
                    sum_mass += mass[k-1];
                }
                G(i,j) = sum_mass * length[i-1] * length[i-1];
                continue;
            }

            //Top Row
            if(i == 0)
            {
                double sum_mass = 0;
                for(unsigned int k = j; k < num_states; k++)
                {
                    sum_mass += mass[k-1] * length[j-1];
                }
                G(i,j) = sum_mass * cos(state[j]);
                continue;
            }

            //Left Column
            if(j == 0)
            {
                double sum_mass = 0;
                for(unsigned int k = i; k < num_states; k++)
                {
                    sum_mass += mass[k-1] * length[i-1];
                }
                G(i,j) = sum_mass * cos(state[i]);
                continue;
            }

            //Bottom Row
            if(i == num_states-1)
            {
                G(i,j) = mass[i-1] * length[j-1] * length[i-1] * cos(state[j] - state[i]);
                continue;
            }

            //Right Column
            if(j == num_states-1)
            {
                G(i,j) = mass[j-1] * length[i-1] * length[j-1] * cos(state[i] - state[j]);
                continue;
            }
        }
    }
    return G;
}

arma::Mat<double> NPole::calculate_F()
{
    arma::Mat<double> F(num_states, num_states);

    for(unsigned int i = 0; i < num_states; i++)
    {
        for(unsigned int j = 0; j < num_states; j++)
        {
            //Diagnol
            if(i==j)
            {
                F(i,j) = 0;
                continue;
                if(i == 0)
                {
                    F(i,j) = 0.3;
                }
                else if(j != num_states-1)
                {
                    F(i,j) = 0.1;
                }
                else
                {
                    double d = 0;
                    for(unsigned int k = 0; k < damp.size(); k++)
                    {
                        d += 0.1;
                    }
                    F(i,j) = d;
                }
                continue;
            }

            //Top Right
            if(i == 0 && j == num_states-1)
            {
                //F(i,j) = mass[j-1] * length[j-1] * state_dot[j] * sin(state[j]);
                F(i,j) = mass[j-1] * length[j-1] * cos(state[j]);
                continue;
            }

            //Top Row
            if(i == 0)
            {
                double sum_mass = 0;
                for(unsigned int k = j; k < num_states; k++)
                {
                    sum_mass += mass[k-1] * length[j-1];
                }
                F(i,j) = -sum_mass * state_dot[j] * sin(state[j]);
                continue;
            }

            //Left Column
            if(j == 0)
            {
                F(i,j) = mass[i-1] * length[i-1] * cos(state[i]);
                continue;
            }

            //Bottom Row
            if(i == num_states-1)
            {
                F(i,j) = -mass[i-1] * length[j-1] * length[i-1] * state_dot[i] * sin(state[j] - state[i]);
                continue;
            }

            //Right Column
            if(j == num_states-1)
            {
                F(i,j) = -mass[j-1] * length[i-1] * length[j-1] * state_dot[j] * sin(state[i] - state[j]);
                continue;
            }
        }
    }
    F = F * state_dot;
    return F;
}

arma::Mat<double> NPole::calculate_U()
{
    arma::Mat<double> U(num_states, 1);
    for(unsigned int i = 0; i < num_states; i++)
    {
        if(i == 0)
        {
            U(i) = 0;
            continue;
        }
        double sum_mass = 0;
        for(unsigned int k = i; k < num_states; k++)
        {
            sum_mass += mass[k-1];
        }
        U(i) = -sum_mass * 9.8 * length[i-1] * sin(state[i]);
    }
    return U;
}


