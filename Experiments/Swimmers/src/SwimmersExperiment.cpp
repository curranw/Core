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
    IExperiment::init();
//    arma::Mat<double> probs = m_domain->calculate_probabilities();
//    arma::Mat<double> rewards = m_domain->calculate_rewards();


//    m_learning_algorithm->update_model(rewards, probs);

//    m_learning_algorithm->calculate_states();
//    m_learning_algorithm->calculate_solution();


}

void SwimmersExperiment::end_epoch()
{
    if(m_domain->m_accumulate_data && m_accumulated_data.size() >= 100)
    {
        if(tot_reward > 500 && good_data.size() < 50000)
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

    IExperiment::end_epoch();
    iteration++;

    if(iteration == m_exp_args->num_epochs-1)
    {
        cout << "OMG THIS IS AWESOME" << endl;
        cin.get();
        m_domain->viz = true;
    }
}

void SwimmersExperiment::output_results()
{
    IExperiment::output_results();
    if(m_domain->m_accumulate_data)
    {
        if(good_data.size() >= 50000) utils::to_csv(&good_data, "converged_state_data_swimmers_good_large");
        if(bad_rewards.size() >= 50000) utils::to_csv(&bad_rewards, "converged_state_rewards_swimmers_bad_large");
        if(bad_data.size() >= 50000) utils::to_csv(&bad_data, "converged_state_data_swimmers_bad_large");
        if(good_rewards.size() >= 50000) utils::to_csv(&good_rewards, "converged_state_rewards_swimmers_good_large");


    }

}



SwimmersExperiment::~SwimmersExperiment()
{
}





