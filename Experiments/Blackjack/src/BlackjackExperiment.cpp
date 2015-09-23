#include <BlackjackExperiment.h>

BlackjackExperiment::BlackjackExperiment(Blackjack *domain, QTiles *learning_algorithm, BlackjackExperimentArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    tot_reward = 0;
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;

}

void BlackjackExperiment::end_epoch()
{
    if(m_domain->m_accumulate_data)
    {
        if(tot_reward > 1 && good_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                good_data.push_back(m_accumulated_data[i]);
            }
        }
        if(tot_reward < -3 && bad_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                bad_data.push_back(m_accumulated_data[i]);
            }
        }
    }
    if(current_iteration % 100 == 0) cout << m_learning_algorithm->get_table_size() << endl;
    performance = 0;
    IExperiment::end_epoch();
}

void BlackjackExperiment::output_results()
{
    IExperiment::output_results();
    if(m_domain->m_accumulate_data)
    {
        if(good_data.size() >= 50000) utils::to_csv(&good_data, "converged_state_data_blackjack_good");

        if(bad_data.size() >= 50000) utils::to_csv(&bad_data, "converged_state_data_blackjack_bad");
    }

}



BlackjackExperiment::~BlackjackExperiment()
{
}





