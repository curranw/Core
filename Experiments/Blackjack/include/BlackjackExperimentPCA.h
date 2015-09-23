#pragma once
#include <IExperiment.h>
#include <QTilesReuse.h>
#include <QTiles.h>
#include <Blackjack.h>

using namespace std;

class BlackjackExperimentPCAArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    bool iterative;
    int start_dimension;
};

class BlackjackExperimentPCA : public IExperiment
{
public:
    BlackjackExperimentPCA(Blackjack* domain, QTilesReuse* learning_algorithm, BlackjackExperimentPCAArgs* exp_args);
    virtual ~BlackjackExperimentPCA();
    void init();
    void end_epoch();
    Blackjack* m_domain;

private:
    BlackjackExperimentPCAArgs* m_exp_args;
    QTilesReuse* m_learning_algorithm;
    int current_dimension;
    double performance;
};
