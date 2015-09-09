#include <ILearningAlgorithm.h>
ILearningAlgorithm::ILearningAlgorithm(ILearningArguments* args)
{

}

void ILearningAlgorithm::set_ranges(vector<double> min_ranges, vector<double> max_ranges)
{
    m_min_ranges = min_ranges;
    m_max_ranges = max_ranges;
}

void ILearningAlgorithm::set_resolution(vector<double> resolution)
{
    m_resolution = resolution;
}

void ILearningAlgorithm::end_epoch()
{

}

void ILearningAlgorithm::set_possible_actions(vector<int> possible_actions)
{
    m_possible_actions = possible_actions;
}
