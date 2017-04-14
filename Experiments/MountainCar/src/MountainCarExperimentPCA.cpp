#include <MountainCarExperimentPCA.h>


MountainCarExperimentPCA::MountainCarExperimentPCA(MountainCar3D *domain, QTilesReuse *learning_algorithm, MountainCarExperimentPCAArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;

    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;

}

MountainCarExperimentPCA::MountainCarExperimentPCA(MountainCar3DScaled *domain, QTilesReuse *learning_algorithm, MountainCarExperimentPCAArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;

    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
}

MountainCarExperimentPCA::MountainCarExperimentPCA(MountainCar4D *domain, QTilesReuse *learning_algorithm, MountainCarExperimentPCAArgs *exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;

    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
}

void MountainCarExperimentPCA::init()
{
    IExperiment::init();
    current_dimension = m_exp_args->start_dimension;
    m_learning_algorithm->set_projection_dimension(current_dimension);
}

void MountainCarExperimentPCA::end_epoch()
{
    //cout << tot_reward << endl;
    IExperiment::end_epoch();
    if(m_exp_args->iterative)
    {
        if(m_learning_algorithm->is_converged())
        {
            current_dimension++;
            m_learning_algorithm->set_projection_dimension(current_dimension);
        }
    }
}

MountainCarExperimentPCA::~MountainCarExperimentPCA()
{
    delete m_domain;
    delete m_learning_algorithm;
    delete m_exp_args;
}
