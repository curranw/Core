#include <BallBalanceExperiment.h>

BallBalanceExperiment::BallBalanceExperiment(BallBalance *domain, QTiles *learning_algorithm, BallBalanceExperimentArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    iteration = 0;
}


void BallBalanceExperiment::init()
{
    potential_prev = 0;
    IExperiment::init();
}

void BallBalanceExperiment::step()
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

    m_learning_algorithm->update(update_params);

    tot_reward += reward;
    if(m_domain->m_accumulate_data) m_accumulated_data.push_back(state);
    if(m_domain->m_accumulate_data) m_accumulated_rewards.push_back(reward);
}

void BallBalanceExperiment::end_epoch()
{
    cout << tot_reward << endl;
    if(m_domain->m_accumulate_data)
    {
        if(tot_reward > 15 && good_data.size() < 5000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                good_data.push_back(m_accumulated_data[i]);
                good_rewards.push_back(m_accumulated_rewards[i]);
            }
        }
        if(tot_reward < 10 && bad_data.size() < 5000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                bad_data.push_back(m_accumulated_data[i]);
                bad_rewards.push_back(m_accumulated_rewards[i]);
            }
        }
    }
    m_learning_algorithm->init();
    IExperiment::end_epoch();
    iteration++;
    cout << m_learning_algorithm->get_table_size() << endl;

    if(iteration >= m_exp_args->num_epochs-5)
    {
        cout << "OMG THIS IS AWESOME" << endl;
        cin.get();
        m_domain->viz = true;
    }

}

void BallBalanceExperiment::output_results()
{
    m_learning_algorithm->output("BallBalancePCAQTable");
//    table_sizes.push_back(m_learning_algorithm->get_table_size());
//    utils::to_csv<int>(&table_sizes, m_exp_args->save_file + "-table_size");
//        IExperiment::output_results();
    utils::to_csv<double>(&m_rewards, m_exp_args->save_file);

    if(m_domain->m_accumulate_data)
    {
        if(good_data.size() >= 1) utils::to_csv2d<double>(&good_data, "converged_state_data_BallBalance_good");
        if(bad_rewards.size() >= 1) utils::to_csv<double>(&bad_rewards, "converged_state_rewards_BallBalance_bad");
        if(bad_data.size() >= 1) utils::to_csv2d<double>(&bad_data, "converged_state_data_BallBalance_bad");
        if(good_rewards.size() >= 1) utils::to_csv<double>(&good_rewards, "converged_state_rewards_BallBalance_good");


    }

}



BallBalanceExperiment::~BallBalanceExperiment()
{
    delete m_domain;
    delete m_learning_algorithm;
    delete m_exp_args;
}





