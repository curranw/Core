#include "RobotChasing.h"

RobotChasing::RobotChasing()
{
    m_max_x = 1;
    m_max_y = 1;
}

void RobotChasing::init()
{
    m_agent_x = 0;
    m_agent_y = 0;
    m_robot_x = rand() % m_max_x + 1;
    m_robot_y = rand() % m_max_y + 1;
}

vector<double> RobotChasing::get_state()
{
    vector<double> state;
    state.push_back(m_agent_x);
    state.push_back(m_agent_y);
    state.push_back(m_robot_x);
    state.push_back(m_robot_y);
    return state;
}

void RobotChasing::step(int action)
{
    //Agent
    vector<double> a = m_action_mapping[action];
    //Down
    if(a[0] == 0) m_agent_y += 1;
    //Left
    if(a[0] == 1) m_agent_x -= 1;
    //Right
    if(a[0] == 2) m_agent_x += 1;
    //Up
    if(a[0] == 3) m_agent_y -= 1;
    if(m_agent_y < 0) m_agent_y = 0;
    if(m_agent_y > m_max_y) m_agent_y = m_max_y;
    if(m_agent_x < 0) m_agent_x = 0;
    if(m_agent_x > m_max_x) m_agent_x = m_max_y;

    if(m_agent_x == m_robot_x && m_agent_y == m_robot_y) return;
    //Robot
//    a = m_action_mapping[rand() % 4];
//    //Down
//    if(a[0] == 0) m_robot_y += 1;
//    //Left
//    if(a[0] == 1) m_robot_x -= 1;
//    //Right
//    if(a[0] == 2) m_robot_x += 1;
//    //Up
//    if(a[0] == 3) m_robot_y -= 1;
//    if(m_robot_y < 0) m_robot_y = 0;
//    if(m_robot_y > m_max_y) m_robot_y = m_max_y;
//    if(m_robot_x < 0) m_robot_x = 0;
//    if(m_robot_x > m_max_x) m_robot_x = m_max_y;

    //cout << m_agent_x << "," << m_agent_y << endl;
    //cout << "-------" << endl;
    //cout << m_robot_x << "," << m_robot_y << endl;

}

vector<double> RobotChasing::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(0);
    min_ranges.push_back(0);
    min_ranges.push_back(0);
    min_ranges.push_back(0);
    return min_ranges;
}

vector<double> RobotChasing::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(m_max_x);
    max_ranges.push_back(m_max_y);
    max_ranges.push_back(m_max_x);
    max_ranges.push_back(m_max_y);
    return max_ranges;
}

bool RobotChasing::end_of_episode()
{
    if(m_agent_x == m_robot_x && m_agent_y == m_robot_y)
    {
        //cout << "Done" << endl;
        return true;
    }
    return false;
}

double RobotChasing::get_reward()
{
    if(m_agent_x == m_robot_x && m_agent_y == m_robot_y) return 1;
    return -1;
}

void RobotChasing::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    for(unsigned int i = 0; i < 4; i++)
    {
        possible_actions.push_back(i);
        vector<double> a;
        a.push_back(i);
        action_mapping.insert(make_pair(i, a));
    }
    set_possible_actions(possible_actions, action_mapping);
}

arma::Mat<double> RobotChasing::calculate_probabilities()
{
    calculate_states();
    arma::Mat<double> all_probabilities(state_to_ind.size() * m_possible_actions.size(),state_to_ind.size(), arma::fill::zeros);
    int it = 0;
    for(unsigned int i = 0; i <= m_max_x; i++)
    {
        for(unsigned int j = 0; j <= m_max_y; j++)
        {
            for(unsigned int k = 0; k <= m_max_x; k++)
            {
                for(unsigned int l = 0; l <= m_max_y; l++)
                {
                    for(unsigned int a = 0; a < m_possible_actions.size(); a++)
                    {
                        vector<double> s;
                        s.push_back(i);
                        s.push_back(j);
                        s.push_back(k);
                        s.push_back(l);


                        if(i == k && j == l)
                        {
                            int ind = state_to_ind[s];
                            all_probabilities(it, ind) += 1.0;
                        }
                        else
                        {
                            vector<vector<double> > neighbors = get_neighbors(s,a);
                            for(unsigned int n = 0; n < neighbors.size(); n++)
                            {
                                vector<double> neighbor = neighbors[n];

                                //                            for(unsigned int z = 0; z < neighbor.size(); z++)
                                //                            {
                                //                                cout << neighbor[z] << ",";
                                //                            }
                                //                            cout << endl;
                                int ind = state_to_ind[neighbor];

                                all_probabilities(it, ind) += 1.0/neighbors.size();
                            }
                        }
                        it++;
                    }
                }
            }
        }
    }
    //all_probabilities.print();
    //exit(1);
    return all_probabilities;
}

vector<vector<double> > RobotChasing::get_neighbors(vector<double> state, int action)
{
    vector<vector<double> > all_neighbors;

    int agent_x = state[0];
    int agent_y = state[1];
    int robot_x = state[2];
    int robot_y = state[3];
    //Down
    if(action == 0)
    {
        if(agent_y + 1 <= m_max_y) agent_y++;
    }
    //left
    if(action == 1)
    {
        if(agent_x - 1 >= 0) agent_x--;
    }

    //right
    if(action == 2)
    {
        if(agent_x + 1 <= m_max_x) agent_x++;
    }
    //up
    if(action == 3)
    {
        if(agent_y - 1 >= 0) agent_y--;
    }

    for(unsigned int i = 0; i < 4; i++)
    {
        vector<double> neighbor;
        neighbor.push_back(agent_x);
        neighbor.push_back(agent_y);

        //Down
        if(i == 0) robot_y += 1;
        //Left
        if(i == 1) robot_x -= 1;
        //Right
        if(i == 2) robot_x += 1;
        //Up
        if(i == 3) robot_y -= 1;
        if(robot_y < 0) robot_y = 0;
        if(robot_y > m_max_y) robot_y = m_max_y;
        if(robot_x < 0) robot_x = 0;
        if(robot_x > m_max_x) robot_x = m_max_x;

        neighbor.push_back(robot_x);
        neighbor.push_back(robot_y);
        all_neighbors.push_back(neighbor);
        robot_x = state[2];
        robot_y = state[3];
    }

    return all_neighbors;
}

arma::Mat<double> RobotChasing::calculate_rewards()
{
    //    cout << state_to_ind.size() << endl;
    //    cout << (m_max_x+1) * (m_max_y+1) * (m_max_x+1) * (m_max_y+1) << endl;
    //    exit(1);
    arma::Mat<double> all_rewards(state_to_ind.size() * m_action_mapping.size(),1);
    int it = 0;
    for(unsigned int i = 0; i <= m_max_x; i++)
    {
        for(unsigned int j = 0; j <= m_max_y; j++)
        {
            for(unsigned int k = 0; k <= m_max_x; k++)
            {
                for(unsigned int l = 0; l <= m_max_y; l++)
                {
                    for(unsigned int a = 0; a < m_possible_actions.size(); a++)
                    {
                        if(i == k && j == l) all_rewards(it++) = 1;
                        else all_rewards(it++) = -1;
                    }
                }
            }
        }
    }
    //all_rewards.print();
    //exit(1);
    return all_rewards;
}


void RobotChasing::calculate_states()
{
    vector<double> m_max_ranges = get_max_ranges();
    vector<double> m_min_ranges = get_min_ranges();
    vector<int> num_states;
    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    {
        num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/1.0);
        num_states[i]++;
    }
    vector<vector<double> > all_values;
    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    {
        vector<double> value;
        for(unsigned int j = 0; j < num_states[i]; j++)
        {
            value.push_back(m_min_ranges[i] + (1.0 * (j)));
            cout << value[j] << ",";
        }
        cout << endl;
        all_values.push_back(value);
    }
    vector<vector<double>::iterator> locations;
    for(unsigned int i = 0; i < num_states.size(); i++)
    {
        locations.push_back(all_values[i].begin());
    }

    bool done = false;
    int it = 0;
    while(true)
    {
        int cur_it = num_states.size()-1;
        //Check if you're at the end of the last list. If you are, reset that list and increase the previous one.
        while(true)
        {
            if(locations[0] == all_values[0].end())
            {
                done = true;
                break;
            }
            if(locations[cur_it] == all_values[cur_it].end())
            {
                locations[cur_it] = all_values[cur_it].begin();
                cur_it--;
                locations[cur_it]++;
            }
            else break;
        }
        if(done) break;
        vector<double> s;
        for(unsigned int j = 0; j < locations.size(); j++)
        {
            s.push_back(*locations[j]);
            cout << s[j] << ",";
        }
        cout << endl;
        state_to_ind.insert(make_pair(s, it++));
        locations.back()++;
    }
}
