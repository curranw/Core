#include "QTilesReuse.h"

QTilesReuse::QTilesReuse(double dimensions, QTilesReuseArgs *args) : ILearningAlgorithm(args)
{
    pca_interface = new PCAInterface(dimensions);
    total_dimensions = dimensions;
    m_learning_algorithms = args->learning_algorithms;
    m_args = args;
    vector<vector<double> > pca_position_data = utils::read_csv(args->pca_file);

    solve_manifold(&pca_position_data, args->amount);

    m_resolution = args->resolution;
    for(unsigned int i = 1; i <= total_dimensions; i++)
    {
//        QTilesArguments* args = new QTilesArguments();
//        if(i <= 2)
//        {
//            args->alpha = 0.01;
//            args->eligability = false;
//            args->gamma = 0.99;
//            args->num_tiles = 10;
//        }
//        else
//        {
//            args->alpha = 0.1;
//            args->eligability = false;
//            args->gamma = 0.99;
//            args->num_tiles = 10;
//        }
//        QTiles* learning_algorithm;
//        if(i == 1) learning_algorithm = new QTiles(args);
//        if(i == 2) learning_algorithm = new QTiles(args);
//        if(i == 3) learning_algorithm = new QTiles(args);
//        else learning_algorithm = new QTiles(args);
//        m_learning_algorithms.push_back(learning_algorithm);

        ranges_max_all.push_back(vector<double>());
        ranges_min_all.push_back(vector<double>());
    }

    num_similar = 0;
    num_updates = 0;
}
void QTilesReuse::set_projection_dimension(int projection_dimension)
{
    cout << "Projection Dimension: " << projection_dimension << endl;
    if(total_dimensions < projection_dimension) return;
    cur_dimension = projection_dimension;
    used_projections.insert(cur_dimension);
    pca_interface->set_projection_dimension(projection_dimension);
    vector<double> temp_resolution;
    if(all_states.size() == 0)
    {
        compute_all_states();
        pca_interface->set_projection_dimension(projection_dimension);
    }
    if(projection_dimension != 1)
    {
        m_learning_algorithms[cur_dimension-2]->no_new = true;
    }
    if(projection_dimension != total_dimensions)
    {
        for(unsigned int i = 0; i < cur_dimension; i++)
        {
            //temp_resolution.push_back(abs(down_range_max[i] - down_range_min[i])/((total_dimensions-(cur_dimension-1) ) *10));
            //temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(50));
            //temp_resolution.push_back(abs(1.0 - 0.0)/(10));
            double temp = (m_max_ranges[i] - m_min_ranges[i])/m_args->resolution[i];
            cout << temp << endl;
            if(i == 0) temp_resolution.push_back(abs(1.0 - 0.0)/(82.0));
            if(i == 1) temp_resolution.push_back(abs(1.0 - 0.0)/(10.0));
            if(i == 2) temp_resolution.push_back(abs(1.0 - 0.0)/(4.0));
            if(i == 3) temp_resolution.push_back(abs(1.0 - 0.0)/(1.0));
            if(i == 4) temp_resolution.push_back(abs(1.0 - 0.0)/(1.0));
            if(i == 5) temp_resolution.push_back(abs(1.0 - 0.0)/(1.0));
            if(i == 6) temp_resolution.push_back(abs(1.0 - 0.0)/(1.0));
            if(i == 7) temp_resolution.push_back(abs(1.0 - 0.0)/(1.0));
//            temp_resolution.push_back(abs(1.0 - 0.0)/15.0);




        }
        m_learning_algorithms[cur_dimension-1]->set_resolution(temp_resolution);
        m_learning_algorithms[cur_dimension-1]->set_ranges(ranges_min_all[projection_dimension-1], ranges_max_all[projection_dimension-1]);
    }
    else
    {
        //Test that this actually happens!!!
        m_resolution.clear();
        for(unsigned int i = 0; i < total_dimensions; i++)
        {
            m_resolution = m_args->resolution;
           // m_resolution.push_back(0.05);
            //m_resolution.push_back(0.1);
        }
        m_learning_algorithms[cur_dimension-1]->set_resolution(m_resolution);
        m_learning_algorithms[cur_dimension-1]->set_ranges(m_min_ranges, m_max_ranges);
    }

    //m_learning_algorithms[cur_dimension-1]->reset_table(new_q_table);
}

pair<QElement::Action, double> QTilesReuse::get_action(QElement::State state)
{
    unordered_map<int, double> action_values = compute_action_values(&state);
    pair<QElement::Action, double> action_to_value = m_learning_algorithms[cur_dimension-1]->random_action(action_values);
    update_args.action = action_to_value.first;
    update_args.old_value = action_to_value.second;
    return action_to_value;
}

void QTilesReuse::update(QElement::State* old_state, QElement::Action old_action, QElement::State* new_state, double reward)
{
    unordered_map<int, double>::const_iterator max_it;
    unordered_map<int, double> new_action_values;
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

        unordered_map<int, double> cur_action_values = m_learning_algorithms[i-1]->get_action_values(down_state);
        if(i == cur_dimension && i != 1)
        {
            for(unordered_map<int, double>::iterator it = new_action_values.begin(); it != new_action_values.end(); it++)
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
            reward += new_value;
            cout << "HI" << endl;
        }

        for(unordered_map<int, double>::iterator it = cur_action_values.begin(); it != cur_action_values.end(); it++)
        {
            if(i == cur_dimension) new_action_values[it->first] += it->second;
            else new_action_values[it->first] += it->second;
        }
    }

    update_args.next_state_action_values = new_action_values;
    update_args.reward = reward;
    pca_interface->set_projection_dimension(cur_dimension);

    if(cur_dimension != total_dimensions)
    {
        unordered_map<int, double> action_values_old = compute_action_values(old_state);
        m_learning_algorithms[cur_dimension-1]->update(update_args);
        unordered_map<int, double> action_values_new = compute_action_values(old_state);

        double max_old, max_new;
        int action_old, action_new;
        for(unsigned int i = 0; i < m_possible_actions.size(); i++)
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
        m_learning_algorithms[cur_dimension-1]->update(update_args);
    }
}

void QTilesReuse::update(QUpdate update)
{
    unordered_map<int, double>::const_iterator max_it;
    unordered_map<int, double> new_action_values;
    for(set<int>::iterator used_proj_it = used_projections.begin(); used_proj_it != used_projections.end(); used_proj_it++)
    {
        int i = *used_proj_it;
        pca_interface->set_projection_dimension(i);
        QElement::State down_state;
        if(i == total_dimensions)
        {
            down_state = update.next_state;
        }
        else
        {
            down_state = pca_interface->transform_down(&update.next_state);
        }

        unordered_map<int, double> cur_action_values = m_learning_algorithms[i-1]->get_action_values(down_state);

        if(i == cur_dimension && new_action_values.size() != 0)
        {
            for(unordered_map<int, double>::iterator it = new_action_values.begin(); it != new_action_values.end(); it++)
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
            //update.reward += new_value;
        }

        for(unordered_map<int, double>::iterator it = cur_action_values.begin(); it != cur_action_values.end(); it++)
        {
            if(i == cur_dimension) new_action_values[it->first] += it->second;
            else new_action_values[it->first] += it->second;
        }
    }

    update_args.next_state_action_values = new_action_values;
    update_args.reward = update.reward;
    pca_interface->set_projection_dimension(cur_dimension);

    if(cur_dimension != total_dimensions)
    {
        unordered_map<int, double> action_values_old = compute_action_values(&update.state);
        m_learning_algorithms[cur_dimension-1]->update(update_args);
        unordered_map<int, double> action_values_new = compute_action_values(&update.state);

        double max_old, max_new;
        int action_old, action_new;
        for(unsigned int i = 0; i < m_possible_actions.size(); i++)
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
        m_learning_algorithms[cur_dimension-1]->update(update_args);
    }
}


void QTilesReuse::solve_manifold(vector<QElement::State>* states, int amount)
{
    if(!m_args->weight_file.empty())
    {
        vector<vector<double> > weights = utils::read_csv(m_args->weight_file);
        pca_interface->add_weights(&weights.at(0));
    }
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

unordered_map<int, double> QTilesReuse::compute_action_values(QElement::State* state)
{
    unordered_map<int, double> action_values;
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
        unordered_map<int, double> cur_action_values = m_learning_algorithms[i-1]->get_action_values(down_state);
        for(unordered_map<int, double>::iterator it = cur_action_values.begin(); it != cur_action_values.end(); it++)
        {
            if(i == cur_dimension) action_values[it->first] += it->second;
            else action_values[it->first] += it->second;
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
    update_args.states_to_update = m_learning_algorithms[cur_dimension-1]->old_states;
    pca_interface->set_projection_dimension(cur_dimension);
    return action_values;
}

void QTilesReuse::compute_all_states()
{
    vector<vector<double> > all_values;
    vector<int> num_states;
    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    {
        vector<double> value;
        value.push_back(m_min_ranges[i]);
        value.push_back(m_max_ranges[i]);
        all_values.push_back(value);
        num_states.push_back(2);
    }

//    vector<int> num_states;
//    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
//    {
//        num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/m_resolution[i]);
//    }
//    vector<vector<double> > all_values;
//    for(unsigned int i = 0; i < num_states.size(); i++)
//    {
//        vector<double> values;
//        //values.push_back(ranges_min[i]);
//        for(unsigned int j = 0; j < num_states[i]; j++)
//        {
//            values.push_back(m_min_ranges[i] + (m_resolution[i] * (j)));
//        }
//        if(values.back() != m_max_ranges[i]) values.push_back(m_max_ranges[i]);
//        all_values.push_back(values);
//    }
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
                    //if(d-1 == 0) cout << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << endl;
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

int QTilesReuse::get_table_size()
{
    return m_learning_algorithms[cur_dimension-1]->get_table_size();
}

void QTilesReuse::clear_trace()
{
    m_learning_algorithms[cur_dimension-1]->clear_trace();
}

bool QTilesReuse::is_converged()
{
//    cout << num_updates << endl;
//    cout << num_similar << endl;
//    exit(1);
    if(total_dimensions == cur_dimension) return false;
    double val = num_similar/num_updates;
    static int it = 0;
    if(it++ % 100 == 0) cout << val << endl;
    num_similar = 0;
    num_updates = 0;
    static int num = 0;
    if(val > .99) num++;
    else num = 0;
    if(num == 5)
    {
        num = 0;
        return true;
    }
    else return false;

//    static running_val = 0;
//    running_val = running_val+0.1 * val;
}

void QTilesReuse::set_possible_actions(vector<int> possible_actions)
{
    m_possible_actions = possible_actions;

    for(unsigned int i = 0; i < m_learning_algorithms.size(); i++)
    {
        m_learning_algorithms[i]->set_possible_actions(possible_actions);
    }

}

QTilesReuse::~QTilesReuse()
{
    for(unsigned int i = 0; i < m_learning_algorithms.size(); i++)
    {
        cout << i << endl;
        delete m_learning_algorithms[i];
    }
    delete pca_interface;

    delete m_args;
}

