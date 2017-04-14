#include <BallBalanceExperimentPCA.h>


BallBalanceExperimentPCA::BallBalanceExperimentPCA(BallBalance *domain, QTilesReuse *learning_algorithm, BallBalanceExperimentPCAArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;

    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;

}

void BallBalanceExperimentPCA::init()
{
    IExperiment::init();
    current_dimension = m_exp_args->start_dimension;
    m_learning_algorithm->set_projection_dimension(current_dimension);
    action_count.assign(13, 0);
}

void BallBalanceExperimentPCA::step()
{
    cur_step++;
    vector<double> state = m_domain->get_state();

    pair<QElement::Action, double> action_value = m_learning_algorithm->get_action(state);

    m_domain->step(action_value.first);

    action_count[action_value.first]++;
    vector<double> next_state = m_domain->get_state();
    //cout << potential_prev << "," << potential_post << "," << -(potential_prev - .99 * potential_post) << endl;

    double reward = m_domain->get_reward();

    update_params.reward = reward;// + -(potential_prev - .99 * potential_post);
    update_params.state = state;
    update_params.next_state = next_state;
    update_params.action = action_value.first;
    update_params.old_value = action_value.second;

    m_learning_algorithm->update(update_params);

    tot_reward += reward;
    if(m_domain->m_accumulate_data) m_accumulated_data.push_back(state);
    if(m_domain->m_accumulate_data) m_accumulated_rewards.push_back(reward);
}

void BallBalanceExperimentPCA::end_epoch()
{
    cout << tot_reward << endl;
//    for(unsigned int i = 0; i < action_count.size(); i++)
//    {
//        cout << "Action: " << i << ":" << action_count[i] << endl;
//    }
    action_count.assign(13, 0);
    IExperiment::end_epoch();
    if(m_exp_args->iterative)
    {
        if(m_learning_algorithm->is_converged())
        {
            table_sizes.push_back(m_learning_algorithm->get_table_size());
            current_dimension++;
            m_learning_algorithm->set_projection_dimension(current_dimension);
        }
    }
}

void BallBalanceExperimentPCA::output_results()
{

    m_learning_algorithm->output("BallBalancePCAQTable");
    table_sizes.push_back(m_learning_algorithm->get_table_size());
    utils::to_csv<int>(&table_sizes, m_exp_args->save_file + "-table_size");
//    IExperiment::output_results();
    utils::to_csv<double>(&m_rewards, m_exp_args->save_file);

}

BallBalanceExperimentPCA::~BallBalanceExperimentPCA()
{
    delete m_domain;
    delete m_learning_algorithm;
    delete m_exp_args;
}
