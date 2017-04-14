#include <SwimmersExperiment.h>

SwimmersExperiment::SwimmersExperiment(Swimmers *domain, ValueIteration *learning_algorithm, SwimmersExperimentArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

SwimmersExperiment::SwimmersExperiment(Swimmers* domain, RMax* learning_algorithm, SwimmersExperimentArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

SwimmersExperiment::SwimmersExperiment(Swimmers* domain, FittedRMax* learning_algorithm, SwimmersExperimentArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

SwimmersExperiment::SwimmersExperiment(Swimmers *domain, QTiles *learning_algorithm, SwimmersExperimentArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;
    iteration = 0;
}


void SwimmersExperiment::init()
{
    potential_prev = 0;
    IExperiment::init();
//    arma::Mat<double> probs = m_domain->calculate_probabilities();
//    arma::Mat<double> rewards = m_domain->calculate_rewards();


//    m_learning_algorithm->update_model(rewards, probs);

//    m_learning_algorithm->calculate_states();
//    m_learning_algorithm->calculate_solution();


}

void SwimmersExperiment::step()
{
    cur_step++;
    vector<double> state = m_domain->get_state();

    pair<QElement::Action, double> action_value = m_learning_algorithm->get_action(state);

    m_domain->step(action_value.first);

    vector<double> next_state = m_domain->get_state();
    double potential_post = m_domain->get_potential();
    //cout << potential_prev << "," << potential_post << "," << -(potential_prev - .99 * potential_post) << endl;

    double reward = m_domain->get_reward();

    update_params.reward = reward;// + -(potential_prev - .99 * potential_post);
    update_params.state = state;
    update_params.next_state = next_state;
    update_params.action = action_value.first;
    update_params.old_value = action_value.second;

    m_learning_algorithm->update(update_params);

    tot_reward += reward;
    potential_prev = potential_post;
    if(m_domain->m_accumulate_data) m_accumulated_data.push_back(state);
    if(m_domain->m_accumulate_data) m_accumulated_rewards.push_back(reward);
}

void SwimmersExperiment::end_epoch()
{
    if(m_domain->m_accumulate_data && m_accumulated_data.size() >= 100)
    {
        if(tot_reward > 350 && good_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                good_data.push_back(m_accumulated_data[i]);
                good_rewards.push_back(m_accumulated_rewards[i]);
            }
        }
        if(tot_reward < 100 && bad_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                bad_data.push_back(m_accumulated_data[i]);
                bad_rewards.push_back(m_accumulated_rewards[i]);
            }
        }
    }
    if(iteration % 100 == 0) cout << ((QTiles*)m_learning_algorithm)->get_table_size() << endl;
    IExperiment::end_epoch();
    iteration++;
    m_domain->viz = false;

//    if(iteration == m_exp_args->num_epochs-1)
//    {
//        cout << "OMG THIS IS AWESOME" << endl;
//        cin.get();
//        m_domain->viz = true;
//    }
}

void SwimmersExperiment::output_results()
{
    table_sizes.push_back(((QTiles*)m_learning_algorithm)->get_table_size());
    utils::to_csv<int>(&table_sizes, m_exp_args->save_file + "-table_size");
    IExperiment::output_results();
    if(m_domain->m_accumulate_data)
    {
        if(good_data.size() >= 500) utils::to_csv2d<double>(&good_data, "converged_state_data_good_" + m_exp_args->save_file);
//        if(bad_rewards.size() >= 500) utils::to_csv<double>(&bad_rewards, "converged_state_rewards_bad_" + m_exp_args->save_file);
        if(bad_data.size() >= 500) utils::to_csv2d<double>(&bad_data, "converged_state_data_bad_" + m_exp_args->save_file);
//        if(good_rewards.size() >= 500) utils::to_csv<double>(&good_rewards, "converged_state_rewards_good_" + m_exp_args->save_file);
    }
    //m_learning_algorithm->output("QTable_"  + m_exp_args->save_file);
}



SwimmersExperiment::~SwimmersExperiment()
{
    delete m_domain;
    delete m_learning_algorithm;
    delete m_exp_args;
}





