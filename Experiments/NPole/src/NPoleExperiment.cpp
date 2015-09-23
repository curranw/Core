#include <NPoleExperiment.h>

NPoleExperiment::NPoleExperiment(NPole *domain, QTiles *learning_algorithm, NPoleExperimentArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

void NPoleExperiment::step()
{
    IExperiment::step();
    performance += m_domain->get_performance(m_exp_args->num_steps - cur_step);
}

void NPoleExperiment::end_epoch()
{
    if(m_domain->m_accumulate_data && m_accumulated_data.size() >= 100)
    {
        if(tot_reward > 600 && good_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                good_data.push_back(m_accumulated_data[i]);
            }
        }
        if(tot_reward < 100 && bad_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                bad_data.push_back(m_accumulated_data[i]);
            }
        }
    }
    if(current_iteration % 100 == 0) cout << m_learning_algorithm->get_table_size() << "," << performance << endl;
    performance = 0;
    IExperiment::end_epoch();
}

void NPoleExperiment::output_results()
{
    IExperiment::output_results();
    if(m_domain->m_accumulate_data)
    {
        if(good_data.size() >= 50000) utils::to_csv(&good_data, "converged_state_data_pole_good");

        if(bad_data.size() >= 50000) utils::to_csv(&bad_data, "converged_state_data_pole_bad");
    }
    m_domain->calculate_pos(true);

}



NPoleExperiment::~NPoleExperiment()
{
}





