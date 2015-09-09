#ifndef QREUSETILES_H
#define QREUSETILES_H

#include <Q_KNN.h>
#include <PCAInterface.h>
#include <QTiles.h>
#include <vector>
#include <map>
#include <set>

class QReuse_Tiles
{
public:
    QReuse_Tiles(double dimensions);
    void set_projection_dimension(int projection_dimension);
    QElement::Action get_action(QElement::State *state);
    void update(QElement::State *old_state, QElement::Action old_action, QElement::State *new_state, double reward);
    void solve_manifold(vector<QElement::State>* states, int amount);

    void set_range(vector<double> ranges_min_input, vector<double> ranges_max_input);
    void set_resolution(vector<double> resolution);
    void set_possible_actions(vector<QElement::Action> new_possible_actions);
    int get_table_size();

    bool is_converged();
    void clear_trace();
    QElement::State scale(QElement::State* s, int dimension);
    map<int, double> compute_action_values(QElement::State* state);

    void compute_all_states();
    vector<QElement::State> all_states;
    QUpdate update_args;
    PCAInterface* pca_interface;
    vector<double> ranges_min, ranges_max;
    vector<double> resolution;
    int cur_dimension, total_dimensions;
    vector<QElement::Action> possible_actions;
    vector<pair<QElement::State*, QElement::Action> > policy, previous_policy;
    vector<QTiles*> learning_algorithms;
    set<int> used_projections;

    vector<vector<double> > ranges_min_all, ranges_max_all;
    double num_similar;
    double num_updates;
    virtual ~QReuse_Tiles();
};

#endif // QREUSE_H
