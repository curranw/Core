#include "RMax.h"

RMax::RMax(RMaxArgs *args) : ILearningAlgorithm(args)
{
    R_max = 5;
    ValueIterationArgs* VI_args = new ValueIterationArgs();
    VI_args->resolution = args->resolution;
    VI = new ValueIteration(VI_args);
}

pair<QElement::Action, double> RMax::get_action(QElement::State s)
{

    state_to_ind = VI->get_state_to_ind();
    arma::Mat<double> R_temp = R_estimate;
    arma::Mat<double> P_temp = P_estimate;
    for(map<QElement::State, int>::iterator it = state_to_ind->begin(); it != state_to_ind->end(); it++)
    {
        int cur_ind = it->second;
        for(unsigned int a = 0; a < m_possible_actions.size(); a++)
        {
            int cur_sa = cur_ind * m_possible_actions.size() + a;
            if(N_estimate[cur_sa] < 5)
            {
                R_temp[cur_sa] = R_max;
            }
            for(map<QElement::State, int>::iterator it2 = state_to_ind->begin(); it2 != state_to_ind->end(); it2++)
            {
                int cur_ind2 = it2->second;
                P_temp(cur_sa, cur_ind2) = 0;
            }
            P_temp(cur_sa, cur_ind) = 1;
        }
    }

    VI->update_model(&R_temp, &P_temp);


    pair<QElement::Action, double> action_value = VI->get_action(s);
    //cout << action_value.first << endl;
    return action_value;
}

void RMax::set_possible_actions(vector<int> possible_actions)
{
    ILearningAlgorithm::set_possible_actions(possible_actions);
    VI->set_possible_actions(possible_actions);
    VI->calculate_states();
    state_to_ind = VI->get_state_to_ind();

    R_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),1);
    R_estimate.fill(0);
    D_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),state_to_ind->size());
    D_estimate.fill(0);
    N_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),1);
    N_estimate.fill(0);
    P_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),state_to_ind->size());
    P_estimate.fill(0);
    VI->update_model(&R_estimate, &P_estimate);
    VI->calculate_solution();
}

void RMax::set_ranges(vector<double> min_ranges, vector<double> max_ranges)
{
    ILearningAlgorithm::set_ranges(min_ranges, max_ranges);
    VI->set_ranges(min_ranges, max_ranges);
}

void RMax::set_resolution(vector<double> resolution)
{
    ILearningAlgorithm::set_resolution(resolution);
    VI->set_resolution(resolution);
}

void RMax::update(QUpdate update)
{
    state_to_ind = VI->get_state_to_ind();
    int ind = state_to_ind->find(update.state)->second;
    int ind_next = state_to_ind->find(update.next_state)->second;

    int ind_action = ind * m_possible_actions.size() + update.action;

    if(ind_action == 4)
    {
        //cout << update.state[0] << "," << update.state[1] << "," << update.state[2] << "," << update.state[3] << endl;
        //cout << "-----" << endl;
        //cout << ind << endl;
        //cout << update.reward << endl;
        //exit(1);
    }
    R_estimate(ind_action) = update.reward;
    D_estimate(ind_action, ind_next) += 1;
    N_estimate(ind_action) += 1;

    for(map<QElement::State,int>::iterator it = state_to_ind->begin(); it != state_to_ind->end(); it++)
    {
        int cur_ind = it->second;
        //int cur_ind = state_to_ind->find(s)->second;
        P_estimate(ind_action, cur_ind) = D_estimate(ind_action, cur_ind)/N_estimate(ind_action);
    }


    static int it = 0;
    it++;
    if(it % 9999 == 0 && it != 0)
    {
//        R_estimate.print();
//        P_estimate.print();
//        cout << "-----" << endl;
//        D_estimate.print();
//        N_estimate.print();

        //exit(1);
    }

}
