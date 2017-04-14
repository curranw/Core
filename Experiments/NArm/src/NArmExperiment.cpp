#include <NArmExperiment.h>

NArmExperiment::NArmExperiment(NArm *domain, QTiles *learning_algorithm, NArmExperimentArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;

}

void NArmExperiment::end_epoch()
{
    if(m_domain->m_accumulate_data && m_accumulated_data.size() >= 100)
    {
        if(tot_reward > -300 && good_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                good_data.push_back(m_accumulated_data[i]);
            }
        }
        if(tot_reward > -1000 && tot_reward < -500 && bad_data.size() < 50000)
        {
            for(unsigned int i = 0; i < m_accumulated_data.size(); i++)
            {
                bad_data.push_back(m_accumulated_data[i]);
            }
        }
    }
    if(current_iteration % 100 == 0) cout << m_learning_algorithm->get_table_size() << endl;
    IExperiment::end_epoch();
}

void NArmExperiment::output_results()
{
    IExperiment::output_results();
    if(m_domain->m_accumulate_data)
    {
        if(good_data.size() >= 50000) utils::to_csv2d<double>(&good_data, "converged_state_data_arm_2_good");

        if(bad_data.size() >= 50000) utils::to_csv2d<double>(&bad_data, "converged_state_data_arm_2_bad");
    }
}



NArmExperiment::~NArmExperiment()
{
}





