#include <SwimmersExperimentPCA.h>


SwimmersExperimentPCA::SwimmersExperimentPCA(Swimmers *domain, QTilesReuse *learning_algorithm, SwimmersExperimentPCAArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;

    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;
    iteration = 0;
}

void SwimmersExperimentPCA::init()
{
    running_avg = 0;
    potential_prev = 0;
    IExperiment::init();
    current_dimension = m_exp_args->start_dimension;
    m_learning_algorithm->set_projection_dimension(current_dimension);
}

void SwimmersExperimentPCA::step()
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

void SwimmersExperimentPCA::end_epoch()
{
    m_domain->viz = false;
    static int it = 0;
    if(m_exp_args->iterative)
    {
        it++;
        if(m_learning_algorithm->is_converged())
        {
            table_sizes.push_back(m_learning_algorithm->get_table_size());
            current_dimension++;
            m_learning_algorithm->set_projection_dimension(current_dimension);
            it = 0;
        }
    }
    last_running_avg = running_avg;
    iteration++;
    running_avg = last_running_avg + (tot_reward - last_running_avg)/double((iteration % 2500)+1);
    if(iteration % 100 == 0) cout << m_learning_algorithm->get_table_size() << "," << running_avg << endl;
    performance = 0;

    //iteration++;
//    if(iteration == m_exp_args->num_epochs-1)
//    {
//        cout << "OMG THIS IS AWESOME" << endl;
//        cin.get();
//        m_domain->viz = true;
//    }
    IExperiment::end_epoch();
}

void SwimmersExperimentPCA::output_results()
{
    table_sizes.push_back(m_learning_algorithm->get_table_size());
    utils::to_csv(&table_sizes, m_exp_args->save_file + "-table_size");
    IExperiment::output_results();
}

SwimmersExperimentPCA::~SwimmersExperimentPCA()
{
    delete m_domain;
    delete m_learning_algorithm;
    delete m_exp_args;
}
