#include <TurtleMazeExperiment.h>

TurtleMazeExperiment::TurtleMazeExperiment(TurtleMaze* domain, QTiles* learning_algorithm, TurtleMazeExperimentArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
    performance = 0;

}

TurtleMazeExperiment::~TurtleMazeExperiment()
{
}





