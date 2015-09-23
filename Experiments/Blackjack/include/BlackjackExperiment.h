#pragma once

#include <IExperiment.h>
#include <NPole.h>
#include <QTiles.h>
#include <Blackjack.h>

using namespace std;

class BlackjackExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
};

class BlackjackExperiment : public IExperiment
{
public:
    BlackjackExperiment(Blackjack* domain, QTiles* learning_algorithm, BlackjackExperimentArgs* exp_args);
    void end_epoch();
    void output_results();
    virtual ~BlackjackExperiment();

private:
    BlackjackExperimentArgs* m_exp_args;
    QTiles* m_learning_algorithm;
    Blackjack* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;

    double performance;
    vector<double> tot_performance;

};
