#include <RobotChasingExperiment.h>

RobotChasingExperiment::RobotChasingExperiment(RobotChasing *domain, ValueIteration *learning_algorithm, RobotChasingExperimentArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

RobotChasingExperiment::RobotChasingExperiment(RobotChasing* domain, RMax* learning_algorithm, RobotChasingExperimentArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

RobotChasingExperiment::RobotChasingExperiment(RobotChasing* domain, FittedRMax* learning_algorithm, RobotChasingExperimentArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

void RobotChasingExperiment::init()
{
    IExperiment::init();

//    arma::Mat<double> probs = m_domain->calculate_probabilities();
//    arma::Mat<double> rewards = m_domain->calculate_rewards();


//    m_learning_algorithm->update_model(rewards, probs);

//    m_learning_algorithm->calculate_states();
//    m_learning_algorithm->calculate_solution();


}

void RobotChasingExperiment::end_epoch()
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

    IExperiment::end_epoch();
}

void RobotChasingExperiment::output_results()
{
    IExperiment::output_results();
//    if(m_domain->m_accumulate_data)
//    {
//        if(good_data.size() >= 50000) utils::to_csv(&good_data, "converged_state_data_pole_good");

//        if(bad_data.size() >= 50000) utils::to_csv(&bad_data, "converged_state_data_pole_bad");
//    }

}



RobotChasingExperiment::~RobotChasingExperiment()
{
}





