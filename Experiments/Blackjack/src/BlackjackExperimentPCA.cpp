#include <BlackjackExperimentPCA.h>


BlackjackExperimentPCA::BlackjackExperimentPCA(Blackjack *domain, QTilesReuse *learning_algorithm, BlackjackExperimentPCAArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;

    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

void BlackjackExperimentPCA::init()
{
    IExperiment::init();
    current_dimension = m_exp_args->start_dimension;
    m_learning_algorithm->set_projection_dimension(current_dimension);
}

void BlackjackExperimentPCA::end_epoch()
{
    IExperiment::end_epoch();
    static int it = 0;
    if(m_exp_args->iterative)
    {
        it++;
        if(m_learning_algorithm->is_converged())
        {
            current_dimension++;
            m_learning_algorithm->set_projection_dimension(current_dimension);
            it = 0;
        }
    }
    if(current_iteration % 100 == 0) cout << m_learning_algorithm->get_table_size() << "," << performance << endl;
    performance = 0;
}

BlackjackExperimentPCA::~BlackjackExperimentPCA()
{
    delete m_domain;
    delete m_learning_algorithm;
    delete m_exp_args;
}
