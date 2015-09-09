#include <NArmExperimentPCA.h>


NArmExperimentPCA::NArmExperimentPCA(NArm *domain, QTilesReuse *learning_algorithm, NArmExperimentPCAArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;

    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;

}

void NArmExperimentPCA::init()
{
    IExperiment::init();
    current_dimension = m_exp_args->start_dimension;
    m_learning_algorithm->set_projection_dimension(current_dimension);
}

void NArmExperimentPCA::end_epoch()
{
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

NArmExperimentPCA::~NArmExperimentPCA()
{
    delete m_domain;
    delete m_learning_algorithm;
    delete m_exp_args;
}
