#include <NPole.h>
#include <time.h>

NPole::NPole()
{
    vel_limit = 20;
    len_limit = 1;
    num_links = 2;
    //theta_max = 5.0 * 3.14159;
    //theta_min = -5.0 * 3.14159;
    theta_max = 2 * 3.14159;
    theta_min = -2 * 3.14159;

    for(unsigned int i = 0; i < num_links; i++)
    {
        mass.push_back(1);
        length.push_back(len_limit);
    }
    theta.set_size(num_links, 1);
    theta_dot.set_size(num_links, 1);
    dt = 0.01;
    srand(time(NULL));
}

void NPole::init()
{
    all_x.clear();
    all_y.clear();
    num_iterations = 0;
    cum_r = 0;


    double f = (double)rand() / RAND_MAX;
    double start_theta = -3.14159 + f * (3.14159 - -3.14159);

    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
    {
        if(it == theta.begin()) *it = start_theta;
        else *it = start_theta;
    }
    theta_dot.fill(0);
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
    static int upright = 0;
//    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
//    {
//        if(*it > theta_max || *it < theta_min)
//        {
//            upright = 0;
//            return true;
//        }
//    }

    bool reset = true;
    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
    {
        if(abs(*it) > 3.14159/4.0) reset = false;
    }
    if(reset)
    {
        upright++;
    }
    if(upright == 1000)
    {
        upright = 0;
        return true;
    }
    else return false;

    //    if(abs(link_pos_y[num_links-1] - length[num_links-1] * (num_links)) > 0.1) return false;
    //    return true;

    //    for(unsigned int i = 0; i < link_pos_y.size(); i++)
    //    {
    //        if(link_pos_y[i] < 0) return true;
    //    }
    //    return false;
}

vector<double> NPole::get_state()
{
    vector<double> state;
    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
    {
        state.push_back(*it);
    }
    for(arma::mat::iterator it = theta_dot.begin(); it != theta_dot.end(); it++)
    {
        state.push_back(*it);
    }
    return state;
}

vector<double> NPole::get_min_ranges()
{
    vector<double> min_ranges;
    for(unsigned int i = 0; i < num_links; i++)
    {
        min_ranges.push_back(theta_min * 1.2);
    }
    for(unsigned int i = 0; i < num_links; i++)
    {
        min_ranges.push_back(-vel_limit);
    }
    return min_ranges;
}

vector<double> NPole::get_max_ranges()
{
    vector<double> max_ranges;
    for(unsigned int i = 0; i < num_links; i++)
    {
        max_ranges.push_back(theta_max * 1.2);
    }
    for(unsigned int i = 0; i < num_links; i++)
    {
        max_ranges.push_back(vel_limit);
    }
    return max_ranges;
}

double NPole::get_reward()
{
    calculate_performance();
    //X neeeds to be 0. Y needs to be length, length*2, length*3...

    //    for(unsigned int i = 0; i < link_pos_y.size(); i++)
    //    {
    //        if(link_pos_y[i] < 0) return -1;
    //    }
    //    return 1;
    //    double r = 0;
    //    for(unsigned int i = 0; i < num_links; i++)
    //    {
    //            if(abs(link_pos_y[i] - length[i] * (num_links)) > 0.1) r += -1.0;
    //            else r += 100.0/num_links;
    //    }
    //    return r;
    static int upright = 0;

//    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
//    {
//        if(*it > theta_max || *it < theta_min)
//        {
//            upright = 0;
//            return -100;
//        }
//    }

    bool reset = true;
    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
    {
        if(abs(*it) > 3.14159/4.0) reset = false;
    }
    if(reset)
    {
        upright++;
    }
    if(upright == 1000)
    {
        cout << "GOOOOOOD" << endl;
        upright = 0;
        double r = link_pos_y[num_links-1];
//        for(unsigned int i = 0; i < num_links; i++)
//        {
//            if(link_pos_y[i] < 0) r += link_pos_y[i] + length[i]*i;
//            else r += link_pos_y[i] - length[i]*i;
//        }
        return r - 2.0;
    }
    else
    {
        double r = link_pos_y[num_links-1];
//        for(unsigned int i = 0; i < num_links; i++)
//        {
//            if(link_pos_y[i] < 0) r += link_pos_y[i] + length[i]*i;
//            else r += link_pos_y[i] - length[i]*i;
//        }
        return r - 2.0;
    }
    //    if(abs(link_pos_y[num_links-1] - length[num_links-1] * (num_links)) > 0.1) return -1;
    //    return 100;

    //        double dist = -pow(abs(link_pos_y[num_links-1] - length[num_links-1] * (num_links)),2);
    //    return dist;


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
    for(unsigned int i = 0; i < 11; i++)
    {
        //double val = -2;
        double val = -5 + (i*1.0);

        vector<double> action;
        action.push_back(val);
        possible_actions.push_back(it);
        action_mapping.insert(make_pair(it++, action));
    }
    set_possible_actions(possible_actions, action_mapping);
}

void NPole::calculate_accel(vector<double> action)
{
    arma::Mat<double> G(num_links, num_links);
    for(unsigned int i = 0; i < num_links; i++)
    {
        for(unsigned int j = 0; j < num_links; j++)
        {
            int z = i;
            if(i > j) z = i;
            if(i < j) z = j;
            double sum_mass = 0;
            for(unsigned int k = z; k < num_links; k++)
            {
                sum_mass += mass[k];
            }
            double tot = sum_mass * length[j] * cos(theta[i] - theta[j]);
            G(i,j) = tot;
        }
    }

    arma::Mat<double> F(num_links, 1);

    for(unsigned int i = 0; i < num_links; i++)
    {
        double sum_tot = 0;
        for(unsigned int k = 0; k < num_links; k++)
        {
            int z = k;
            if(k > i) z = k;
            if(k < i) z = i;
            double sum_mass = 0;
            for(unsigned int h = z; h < num_links; h++)
            {
                sum_mass += mass[h];
            }
            sum_tot += (sum_mass * length[k] * sin(theta[k] - theta[i]) * pow(theta_dot[k],2));
        }

        double sum_mass = 0;
        for(unsigned int k = i; k < num_links; k++)
        {
            sum_mass += mass[k];
        }
        double tot = sum_mass * 9.8 * sin(theta[i]);
        if(i == 0)
        {
            tot += 1.0/length[i] * action[i];
        }
        F(i,0) = sum_tot + tot;
    }
    arma::Mat<double> theta_ddot = G.i() * F;
    //    for(arma::mat::iterator it = theta_ddot.begin(); it != theta_ddot.end(); it++)
    //    {
    //        if(*it > 1)
    //        {
    //            *it = 1;
    //        }
    //        if(*it < -1)
    //        {
    //            *it = -1;
    //        }
    //    }



        for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
        {
            if(*it >= 6.28318)
            {
                *it = *it - 6.28318;
            }
            if(*it < 0)
            {
                *it = *it + 6.28318;
            }
        }

    theta_dot = theta_dot + dt * theta_ddot;
        for(arma::mat::iterator it = theta_dot.begin(); it != theta_dot.end(); it++)
        {
            if(*it > vel_limit)
            {
                *it = vel_limit;
            }
            if(*it < -vel_limit)
            {
                *it = -vel_limit;
            }
        }
        theta = theta + dt * theta_dot;


}

void NPole::calculate_pos(bool output)
{
    link_pos_x.clear();
    link_pos_y.clear();
    for(unsigned int i = 0; i < num_links; i++)
    {
        if(i == 0)
        {
            link_pos_y.push_back(0 + length[i] * cos(theta[i]));
            link_pos_x.push_back(0 + length[i] * sin(theta[i]));
        }
        else
        {
            link_pos_y.push_back(link_pos_y[i-1] + length[i] * cos(theta[i]));
            link_pos_x.push_back(link_pos_x[i-1] + length[i] * sin(theta[i]));
        }
        cout << link_pos_x[i] << "," << link_pos_y[i] << endl;
        //if(theta[i] < 0 || theta[i] > 360) exit(1);
        //if(theta_dot[i] < -10 || theta_dot[i] > 10) exit(1);
    }
    all_x.push_back(link_pos_x);
    all_y.push_back(link_pos_y);
    if(output)
    {
        for(unsigned int i = 0; i < all_x.size(); i++)
        {
            for(unsigned int j = 0; j < num_links; j++)
            {
                cout << all_x[i][j] << "," << all_y[i][j] << endl;
            }
        }
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
    static int upright = 0;
//    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
//    {
//        if(*it > theta_max || *it < theta_min)
//        {
//            upright = 0;
//            return -10 * steps_left;
//        }
//    }

    bool reset = true;
    for(arma::mat::iterator it = theta.begin(); it != theta.end(); it++)
    {
        if(abs(*it) > 3.14159/4.0) reset = false;
    }
    if(reset)
    {
        upright++;
    }
    if(upright == 100)
    {
        return 100;
        upright = 0;
        double r = 0;
        for(unsigned int i = 0; i < num_links; i++)
        {
            r += link_pos_y[i];
        }
        return r;
    }
    else
    {
        return -1;
        double r = 0;
        for(unsigned int i = 0; i < num_links; i++)
        {
            r += link_pos_y[i];
        }
        if(r > 10)
        {
            for(unsigned int i = 0; i < num_links; i++)
            {
                cout << link_pos_y[i] << endl;
            }
            exit(1);
        }
        return r;
    }
}

void NPole::calculate_performance()
{

}

NPole::~NPole()
{
}

