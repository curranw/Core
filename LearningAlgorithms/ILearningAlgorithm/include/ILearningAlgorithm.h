#pragma once

#include <map>
#include <vector>
#include <QElement.h>
#include <iostream>
using namespace std;

class ILearningArguments
{
public:
    //Learning Args
    double alpha;
    double gamma;

    //Eligability
    bool eligability;
    double lambda;
};

class ILearningAlgorithm
{
public:
    ILearningAlgorithm(ILearningArguments* args);
    virtual void init();
    virtual pair<QElement::Action, double> get_action(QElement::State s) = 0;
    virtual void update(QUpdate update) = 0;
    virtual void set_possible_actions(vector<int> possible_actions);
    virtual void set_ranges(vector<double> min_ranges, vector<double> max_ranges);
    virtual void set_resolution(vector<double> resolution);
    virtual void end_epoch();
protected:
    ILearningArguments* m_args;

    vector<double> m_min_ranges, m_max_ranges, m_resolution;
    vector<int> m_possible_actions;
};
