#pragma once

#include <ILearningAlgorithm.h>

class ReplayArgs : public ILearningArguments
{
public:
    string replay_file;
};

class Replay : public ILearningAlgorithm
{
public:
    Replay(ReplayArgs *args);
    void init();
    pair<QElement::Action, double> get_action(QElement::State s);
    void update(QUpdate update);

private:
    void read(string replay_file);
    vector<vector<int> > it_to_actions;
    double R_max;

};
