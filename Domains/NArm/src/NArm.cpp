#include <NArm.h>

NArm::NArm()
{
    m_num_links = 6;
    target_x = 24;
    target_y = 10;
}

void NArm::init()
{
    double f;
    for(unsigned int i = 0; i < m_num_links; i++)
    {
        f = (double)rand() / RAND_MAX;
        double angle = 0.0 + f * (3.14159 - 0.0);
        m_links_angles.push_back(angle);
        m_links_lengths.push_back((m_num_links*2)-(i*2));
    }
    calculate_pos();
}

void NArm::step(int action)
{
    vector<double> angle_changes = m_action_mapping[action];
    for(unsigned int i = 0; i < m_num_links; i++)
    {
        m_links_angles[i] += angle_changes[i];
        if(m_links_angles[i] < 0) m_links_angles[i] = 0;
        if(m_links_angles[i] > 3.14159) m_links_angles[i] = 3.14159;
    }
    calculate_pos();
}


bool NArm::end_of_episode()
{
    double x_err = abs(target_x - m_link_pos_x[m_num_links-1]);
    double y_err = abs(target_y - m_link_pos_y[m_num_links-1]);

    return x_err <= 0.01 && y_err < 0.01;
}


vector<double> NArm::get_state()
{
    vector<double> state;
    for(unsigned int i = 0; i < m_num_links; i++)
    {
        state.push_back(m_links_angles[i]);
    }
    return state;
}


vector<double> NArm::get_min_ranges()
{
    vector<double> min_ranges;
    for(unsigned int i = 0; i < m_num_links; i++)
    {
        min_ranges.push_back(0);
    }
    return min_ranges;
}


vector<double> NArm::get_max_ranges()
{
    vector<double> max_ranges;
    for(unsigned int i = 0; i < m_num_links; i++)
    {
        max_ranges.push_back(3.14159);
    }
    return max_ranges;
}


double NArm::get_reward()
{
    double x_err = abs(target_x - m_link_pos_x[m_num_links-1]);
    double y_err = abs(target_y - m_link_pos_y[m_num_links-1]);

    double reward = 0.5 * (pow(x_err,2) + pow(y_err,2));
    return -reward;
}

void NArm::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    vector<double> a;
    a.push_back(-0.0174532925);
    a.push_back(0.0174532925);

    int it = 0;
    for(int i = 0; i < m_num_links; i++)
    {
        vector<double> a2(m_num_links);
        for(unsigned int j = 0; j < a.size(); j++)
        {
            a2[i] = a[j];
            possible_actions.push_back(it);
            action_mapping.insert(make_pair(it++, a2));
        }
    }
    vector<double> zero_action(m_num_links);
    possible_actions.push_back(it);
    action_mapping.insert(make_pair(it++, zero_action));
    set_possible_actions(possible_actions, action_mapping);
}

void NArm::calculate_pos()
{
    m_link_pos_x.clear();
    m_link_pos_y.clear();
    for(unsigned int i = 0; i < m_num_links; i++)
    {
        if(i == 0)
        {
            m_link_pos_x.push_back(0 + m_links_lengths[i] * cos(m_links_angles[i]));
            m_link_pos_y.push_back(0 + m_links_lengths[i] * sin(m_links_angles[i]));
            continue;
        }
        else
        {
            m_link_pos_x.push_back(m_link_pos_x[i-1] + m_links_lengths[i] * cos(m_links_angles[i]));
            m_link_pos_y.push_back(m_link_pos_y[i-1] + m_links_lengths[i] * sin(m_links_angles[i]));
            continue;
        }
        if(m_link_pos_x[i] < 0.0001) m_link_pos_x[i] = 0;
        if(m_link_pos_y[i] < 0.0001) m_link_pos_y[i] = 0;
    }
}
NArm::~NArm()
{
}


