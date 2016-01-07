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
    IExperiment::init();
    current_dimension = m_exp_args->start_dimension;
    m_learning_algorithm->set_projection_dimension(current_dimension);
}

void SwimmersExperimentPCA::step()
{
    IExperiment::step();
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
    if(current_iteration % 100 == 0) cout << m_learning_algorithm->get_table_size() << "," << tot_reward << endl;
    performance = 0;

    iteration++;
    if(iteration == m_exp_args->num_epochs-1)
    {
        cout << "OMG THIS IS AWESOME" << endl;
        cin.get();
        m_domain->viz = true;
    }
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
