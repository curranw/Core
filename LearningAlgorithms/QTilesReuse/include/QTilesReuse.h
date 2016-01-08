#pragma once

#include <Q_KNN.h>
#include <PCAInterface.h>
#include <QTiles.h>
#include <ILearningAlgorithm.h>
#include <vector>
#include <map>
#include <set>
#include <utils.h>

class QTilesReuseArgs : public ILearningArguments
{
public:
    vector<QTiles*> learning_algorithms;
    string pca_file;
    string weight_file;
    int amount;
    vector<double> resolution;
};

class QTilesReuse : public ILearningAlgorithm
{
public:
    QTilesReuse(double dimensions, QTilesReuseArgs* args);
    void set_projection_dimension(int projection_dimension);
    pair<QElement::Action, double> get_action(QElement::State state);
    void update(QElement::State *old_state, QElement::Action old_action, QElement::State *new_state, double reward);
    void update(QUpdate update);
    void solve_manifold(vector<QElement::State>* states, int amount);

    int get_table_size();

    bool is_converged();
    void clear_trace();
    QElement::State scale(QElement::State* s, int dimension);
    vector<double> compute_action_values(QElement::State* state);
    void set_possible_actions(vector<int> possible_actions);
    void compute_all_states();
    vector<QElement::State> all_states;
    QUpdate update_args;
    PCAInterface* pca_interface;
    int cur_dimension, total_dimensions;
    vector<QTiles*> m_learning_algorithms;
    set<int> used_projections;
    QTilesReuseArgs* m_args;
    vector<vector<double> > ranges_min_all, ranges_max_all;
    double num_similar;
    double num_updates;
    virtual ~QTilesReuse();
};
