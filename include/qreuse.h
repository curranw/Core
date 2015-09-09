#ifndef QREUSE_H
#define QREUSE_H

#include <Q_KNN.h>
#include <PCAInterface.h>
#include <QTiles.h>
#include <vector>
#include <map>
#include <set>

class QReuseSettings
{
public:
    double alpha, gamma;
    vector<double> ranges_min, ranges_max, resolution;
    vector<QElement::Action> possible_actions;
    int total_dimensions;
};

class QReuse
{
public:
    QReuse(QReuseSettings in_settings);

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

    virtual ~QReuse();


private:
    map<int, double> compute_action_values(QElement::State* state);
    void compute_all_states();
    vector<QElement::State> all_states;

    QUpdate update_args;
    PCAInterface* pca_interface;
    QReuseSettings settings;
    vector<Q_KNN*> learning_algorithms;

    set<int> used_projections;
    int cur_dimension;
    double num_similar;
    double num_updates;

};

#endif // QREUSE_H
