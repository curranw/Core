#include <IExperiment.h>

IExperiment::IExperiment(IDomain* domain, ILearningAlgorithm *learning_algorithm, IExperimentArgs *experiment_args)
{
    tot_reward = 0;
    m_domain = domain;
    m_learning_algorithm = learning_algorithm;
    m_exp_args = experiment_args;
}

void IExperiment::run_experiment()
{
    this->init();
    for(unsigned int i = 0; i < m_exp_args->num_epochs; i++)
    {
        this->epoch();
    }
    this->output_results();
}


void IExperiment::init()
{
    m_learning_algorithm->set_ranges(m_domain->get_min_ranges(), m_domain->get_max_ranges());

    //Compute actions
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    m_domain->compute_possible_actions();
    m_domain->get_possible_actions(possible_actions, action_mapping);
    m_learning_algorithm->set_possible_actions(possible_actions);

    //Variable Bookkeeping
    current_iteration = 0;
    m_learning_algorithm->init();
}


void IExperiment::epoch()
{
    this->begin_epoch();
    for(int i = 0; i < m_exp_args->num_steps; i++)
    {
        this->step();
        if(m_domain->end_of_episode()) break;
    }
    this->end_epoch();
}


void IExperiment::step()
{
    cur_step++;
    vector<double> state = m_domain->get_state();
    pair<QElement::Action, double> action_value = m_learning_algorithm->get_action(state);

    m_domain->step(action_value.first);

    vector<double> next_state = m_domain->get_state();

    double reward = m_domain->get_reward();

    update_params.reward = reward;
    update_params.state = state;
    update_params.next_state = next_state;
    update_params.action = action_value.first;
    update_params.old_value = action_value.second;

//    cout << "State: ";
//    for(unsigned int i = 0; i < state.size(); i++)
//    {
//        cout << state[i] << ",";
//    }
//    cout << endl;
//    cout << "Next State: ";
//    for(unsigned int i = 0; i < next_state.size(); i++)
//    {
//        cout << next_state[i] << ",";
//    }
//    cout << endl;
//    cout << "Action: " << action_value.first << endl;
//    cout << "Reward: " << reward << endl;

    m_learning_algorithm->update(update_params);

    tot_reward += reward;

    if(m_domain->m_accumulate_data) m_accumulated_data.push_back(state);
}

void IExperiment::get_reward()
{

}


void IExperiment::output_results()
{
    vector<double> moving_average = utils::moving_average(&m_rewards, double(m_exp_args->num_epochs)/20.0);
    utils::to_csv(&moving_average, m_exp_args->save_file);
}

void IExperiment::begin_epoch()
{
    m_domain->init();
}

void IExperiment::end_epoch()
{
    m_learning_algorithm->end_epoch();
    m_rewards.push_back(tot_reward);
    if(m_domain->m_accumulate_data) m_accumulated_data.clear();

    current_iteration++;
    if(current_iteration % 1 == 0)
        cout << current_iteration << ", " << tot_reward << endl;

    tot_reward = 0;
    cur_step = 0;
}

vector<vector<double> > IExperiment::get_accumulated_data()
{
    return m_accumulated_data;
}



IExperiment::~IExperiment()
{

}
