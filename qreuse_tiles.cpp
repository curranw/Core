#include "qreuse_tiles.h"

QReuse_Tiles::QReuse_Tiles(double dimensions)
{
    pca_interface = new PCAInterface(dimensions);
    total_dimensions = dimensions;
    for(unsigned int i = 1; i <= total_dimensions; i++)
    {
        QTilesArguments* args = new QTilesArguments();
        if(i <= 2)
        {
            args->alpha = 0.01;
            args->eligability = false;
            args->gamma = 0.99;
            args->num_tiles = 10;
        }
        else
        {
            args->alpha = 0.1;
            args->eligability = false;
            args->gamma = 0.99;
            args->num_tiles = 10;
        }
        QTiles* learning_algorithm;
        if(i == 1) learning_algorithm = new QTiles(args);
        if(i == 2) learning_algorithm = new QTiles(args);
        if(i == 3) learning_algorithm = new QTiles(args);
        else learning_algorithm = new QTiles(args);
        learning_algorithms.push_back(learning_algorithm);

        ranges_max_all.push_back(vector<double>());
        ranges_min_all.push_back(vector<double>());
    }
}
void QReuse_Tiles::set_projection_dimension(int projection_dimension)
{
    previous_policy.clear();
    policy.clear();
    cout << "Projection Dimension: " << projection_dimension << endl;
    if(total_dimensions < projection_dimension) return;
    cur_dimension = projection_dimension;
    used_projections.insert(cur_dimension);
    pca_interface->set_projection_dimension(projection_dimension);
    vector<double> temp_resolution;
    if(all_states.size() == 0)
        compute_all_states();

    if(projection_dimension != total_dimensions)
    {
        for(unsigned int i = 0; i < cur_dimension; i++)
        {
            //temp_resolution.push_back(abs(down_range_max[i] - down_range_min[i])/((total_dimensions-(cur_dimension-1) ) *10));
            //temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(50));
            temp_resolution.push_back(abs(1.0 - 0.0)/(50));
        }
        learning_algorithms[cur_dimension-1]->set_resolution(temp_resolution);
        learning_algorithms[cur_dimension-1]->set_ranges(ranges_min_all[projection_dimension-1], ranges_max_all[projection_dimension-1]);
    }
    else
    {
        resolution.clear();
        resolution.push_back(0.04);
        resolution.push_back(0.04);
        resolution.push_back(0.04);
        resolution.push_back(0.04);
        learning_algorithms[cur_dimension-1]->set_resolution(resolution);
        learning_algorithms[cur_dimension-1]->set_ranges(ranges_min, ranges_max);
    }

    //learning_algorithms[cur_dimension-1]->reset_table(new_q_table);
}

QElement::Action QReuse_Tiles::get_action(QElement::State* state)
{
    map<int, double> action_values = compute_action_values(state);
    pair<QElement::Action, double> action_to_value = learning_algorithms[cur_dimension-1]->random_action(action_values);
    update_args.action = action_to_value.first;
    update_args.old_value = action_to_value.second;
    return action_to_value.first;
}

void QReuse_Tiles::update(QElement::State* old_state, QElement::Action old_action, QElement::State* new_state, double reward)
{
    map<int, double>::const_iterator max_it;
    map<int, double> new_action_values;
    for(set<int>::iterator used_proj_it = used_projections.begin(); used_proj_it != used_projections.end(); used_proj_it++)
    {
        int i = *used_proj_it;
        pca_interface->set_projection_dimension(i);
        QElement::State down_state;
        if(i == total_dimensions)
        {
            down_state = *new_state;
        }
        else
        {
            down_state = pca_interface->transform_down(new_state);
        }

        map<int, double> cur_action_values = learning_algorithms[i-1]->get_action_values(down_state);

        if(i == cur_dimension && i != 1)
        {
            for(map<int, double>::iterator it = new_action_values.begin(); it != new_action_values.end(); it++)
            {
                if(it == new_action_values.begin())
                {
                    max_it = it;
                }
                else if (it->second > max_it->second)
                {
                    max_it = it;
                }
            }
            double new_value = max_it->second;
            //reward += new_value;
        }

        for(map<int, double>::iterator it = cur_action_values.begin(); it != cur_action_values.end(); it++)
        {
            if(cur_dimension == total_dimensions) new_action_values[it->first] += it->second;
            else new_action_values[it->first] += it->second + 100;
        }
    }

    update_args.next_state_action_values = new_action_values;
    update_args.reward = reward;
    pca_interface->set_projection_dimension(cur_dimension);

    if(cur_dimension != total_dimensions)
    {
        map<int, double> action_values_old = compute_action_values(old_state);
        learning_algorithms[cur_dimension-1]->update(update_args);
        map<int, double> action_values_new = compute_action_values(old_state);

        double max_old, max_new;
        int action_old, action_new;
        for(unsigned int i = 0; i < possible_actions.size(); i++)
        {
            if(i == 0)
            {
                max_old = action_values_old[i];
                max_new = action_values_new[i];
                action_old = i;
                action_new = i;
                continue;
            }
            if(action_values_old[i] > max_old)
            {
                max_old = action_values_old[i];
                action_old = i;
            }
            if(action_values_new[i] > max_new)
            {
                max_new = action_values_new[i];
                action_new = i;
            }
        }
        if(action_old == action_new) num_similar++;
        num_updates++;
    }
    else
    {
        learning_algorithms[cur_dimension-1]->update(update_args);
    }
}

void QReuse_Tiles::solve_manifold(vector<QElement::State>* states, int amount)
{
    vector<QElement::State>* sub_states;
    if(amount == -1) pca_interface->add_data(states);
    else
    {
        sub_states = new vector<QElement::State>();
        for(unsigned int i = 0; i < amount; i++)
        {
            sub_states->push_back(states->at(i));
        }
        pca_interface->add_data(sub_states);
        delete sub_states;
    }
    pca_interface->solve();
}

map<int, double> QReuse_Tiles::compute_action_values(QElement::State* state)
{
    map<int, double> action_values;
    for(set<int>::iterator used_proj_it = used_projections.begin(); used_proj_it != used_projections.end(); used_proj_it++)
    {
        int i = *used_proj_it;
        pca_interface->set_projection_dimension(i);
        QElement::State down_state;
        if(i == total_dimensions)
        {
            down_state = *state;
        }
        else
        {
            down_state = pca_interface->transform_down(state);
        }
        map<int, double> cur_action_values = learning_algorithms[i-1]->get_action_values(down_state);
        for(map<int, double>::iterator it = cur_action_values.begin(); it != cur_action_values.end(); it++)
        {
            if(cur_dimension == total_dimensions) action_values[it->first] += it->second;
            else action_values[it->first] += it->second + 100;
        }
    }

    static int it = 0;
    it++;
    if(state->at(0) == -0.5)
    {
        //cout << action_values[0] << "," << action_values[1] << "," << action_values[2] << endl;
        //if(cur_dimension == 2) exit(1);
    }
    update_args.state = *state;
    update_args.states_to_update = learning_algorithms[cur_dimension-1]->old_states;
    pca_interface->set_projection_dimension(cur_dimension);
    return action_values;
}

void QReuse_Tiles::compute_all_states()
{
    vector<int> num_states;
    for(unsigned int i = 0; i < ranges_min.size(); i++)
    {
        num_states.push_back((ranges_max[i] - ranges_min[i])/resolution[i]);
    }
    vector<vector<double> > all_values;
    for(unsigned int i = 0; i < num_states.size(); i++)
    {
        vector<double> values;
        //values.push_back(ranges_min[i]);
        for(unsigned int j = 0; j < num_states[i]; j++)
        {
            values.push_back(ranges_min[i] + (resolution[i] * (j)));
        }
        if(values.back() != ranges_max[i]) values.push_back(ranges_max[i]);
        all_values.push_back(values);
    }
    vector<vector<double>::iterator> locations;
    for(unsigned int i = 0; i < num_states.size(); i++)
    {
        locations.push_back(all_values[i].begin());
    }

    bool done = false;
    while(true)
    {
        int cur_it = num_states.size()-1;
        //Check if you're at the end of the last list. If you are, reset that list and increase the previous one.
        while(true)
        {
            if(locations[0] == all_values[0].end())
            {
                done = true;
                break;
            }
            if(locations[cur_it] == all_values[cur_it].end())
            {
                locations[cur_it] = all_values[cur_it].begin();
                cur_it--;
                locations[cur_it]++;
            }
            else break;
        }
        if(done) break;
        QElement::State s;
        for(unsigned int j = 0; j < locations.size(); j++)
        {
            s.push_back(*locations[j]);
        }
        for(unsigned int d = 1; d < total_dimensions; d++)
        {
            pca_interface->set_projection_dimension(d);
            vector<double> down = pca_interface->transform_down(&s);
            for(unsigned int dd = 0; dd < down.size(); dd++)
            {
                if(ranges_min_all[d-1].size() < down.size())
                {
                    ranges_min_all[d-1].push_back(down[dd]);
                    ranges_max_all[d-1].push_back(down[dd]);
                    continue;
                }
                if(down[dd] < ranges_min_all[d-1][dd])
                {
                    ranges_min_all[d-1][dd] = down[dd];
                }
                if(down[dd] > ranges_max_all[d-1][dd])
                {
                    ranges_max_all[d-1][dd] = down[dd];
                }
            }
        }
        if(all_states.size() == 0) all_states.push_back(s);
        locations.back()++;
        //all_states.push_back(s);
    }

}
void QReuse_Tiles::set_range(vector<double> ranges_min_input, vector<double> ranges_max_input)
{
    ranges_min = ranges_min_input;
    ranges_max = ranges_max_input;
    ranges_min_all[total_dimensions-1] = ranges_min;
    ranges_max_all[total_dimensions-1] = ranges_max;
}

void QReuse_Tiles::set_resolution(vector<double> resolution_input)
{
    resolution = resolution_input;
}

int QReuse_Tiles::get_table_size()
{
    return learning_algorithms[cur_dimension-1]->get_table_size();
}

void QReuse_Tiles::clear_trace()
{
    learning_algorithms[cur_dimension-1]->clear_trace();
}

bool QReuse_Tiles::is_converged()
{
    if(total_dimensions == cur_dimension) return false;
    double val = num_similar/num_updates;
    static int it = 0;
    if(it++ % 100 == 0) cout << val << endl;
    num_similar = 0;
    num_updates = 0;
    static int num = 0;
    if(val > .95) num++;
    else num = 0;
    if(num == 5)
    {
        cout << "DOIN IT!" << endl;
        num = 0;
        return true;
    }
    else return false;

}

void QReuse_Tiles::set_possible_actions(vector<QElement::Action> new_possible_actions)
{
    possible_actions = new_possible_actions;

    for(unsigned int i = 0; i < learning_algorithms.size(); i++)
    {
        learning_algorithms[i]->set_possible_actions(possible_actions);
    }

}

QReuse_Tiles::~QReuse_Tiles()
{
    cout << "HI!" << endl;
    for(unsigned int i = 0; i < learning_algorithms.size(); i++)
    {
        cout << i << endl;
        delete learning_algorithms[i];
    }
    cout << "PCA" << endl;
    delete pca_interface;
    cout << "Done" << endl;
}

