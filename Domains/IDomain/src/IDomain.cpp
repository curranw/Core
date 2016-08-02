#include <IDomain.h>

IDomain::IDomain()
{
    m_accumulate_data = false;
}

void IDomain::get_possible_actions(vector<int> &possible_actions, map<int, vector<double> > &action_mapping)
{
    possible_actions = m_possible_actions;
    action_mapping = m_action_mapping;
}

void IDomain::set_possible_actions(vector<int> possible_actions, map<int, vector<double> > action_mapping)
{
    m_possible_actions = possible_actions;
    m_action_mapping = action_mapping;
}

double IDomain::get_performance()
{

}
