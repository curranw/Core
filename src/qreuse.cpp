#include "qreuse.h"

QReuse::QReuse(QReuseSettings in_settings)
{
    settings = in_settings;
    num_similar = 0;
    num_updates = 0;
    pca_interface = new PCAInterface(settings.total_dimensions);

    for(unsigned int i = 1; i <= settings.total_dimensions; i++)
    {
        Q_KNN* learning_algorithm;
        if(i == 1) learning_algorithm = new Q_KNN(0.01, 0.99);
        else if(i == 2) learning_algorithm = new Q_KNN(0.01, 0.99);
        else if(i == 3) learning_algorithm = new Q_KNN(0.1, 0.99);
        else learning_algorithm = new Q_KNN(0.1, 0.99);

        learning_algorithm->init_knn((i-1)*3 + 1, i, 0);
        learning_algorithm->set_possible_actions(settings.possible_actions);
        learning_algorithms.push_back(learning_algorithm);
    }
}


void QReuse::set_projection_dimension(int projection_dimension)
{
    if(settings.total_dimensions < projection_dimension) return;
    cur_dimension = projection_dimension;
    //used_projections.clear();
    //if(cur_dimension != 1) used_projections.insert(cur_dimension-1);
    used_projections.insert(cur_dimension);

    if(all_states.size() == 0) compute_all_states();
    pca_interface->set_projection_dimension(projection_dimension);
    vector<QElement*> new_q_table;
    if(projection_dimension != settings.total_dimensions)
    {
        vector<double> down_range_min(projection_dimension);
        vector<double> down_range_max(projection_dimension);

        for(unsigned int i = 0; i < all_states.size(); i++)
        {
            QElement::State s = all_states[i];

            QElement::State down_state = pca_interface->transform_down(&s);
            if(i == 0)
            {
                for(unsigned int j = 0; j < down_state.size(); j++)
                {
                    down_range_min[j] = down_state[j];
                    down_range_max[j] = down_state[j];
                }
                continue;
            }
            for(unsigned int j = 0; j < down_state.size(); j++)
            {
                if(down_state[j] > down_range_max[j]) down_range_max[j] = down_state[j];
                if(down_state[j] < down_range_min[j]) down_range_min[j] = down_state[j];
            }
        }

        vector<double> temp_resolution;
        for(unsigned int i = 0; i < cur_dimension; i++)
        {
            temp_resolution.push_back(abs(down_range_min[i] - down_range_max[i])/(50));
        }

        vector<int> num_states;
        for(unsigned int i = 0; i < down_range_min.size(); i++)
        {
            num_states.push_back(abs((down_range_min[i] - down_range_max[i])/temp_resolution[i]));
        }
        vector<vector<double> > all_values;

        for(unsigned int i = 0; i < num_states.size(); i++)
        {
            vector<double> values;
            for(unsigned int j = 0; j < num_states[i]; j++)
            {
                values.push_back(down_range_min[i] + (temp_resolution[i] * (j)));
            }
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
            locations.back()++;
            QElement* ele = new QElement();
            ele->s = s;
            ele->a = settings.possible_actions;
            ele->v = vector<double>(settings.possible_actions.size());
            new_q_table.push_back(ele);
        }
    }
    else
    {
        pca_interface->set_projection_dimension(cur_dimension);
        for(unsigned int i = 0; i < all_states.size(); i++)
        {
            QElement* ele = new QElement();
            ele->s = all_states[i];
            ele->a = settings.possible_actions;
            ele->v = vector<double>(settings.possible_actions.size());
            new_q_table.push_back(ele);
            all_states[i].clear();
        }
    }

    learning_algorithms[cur_dimension-1]->reset_table(new_q_table);

}

QElement::Action QReuse::get_action(QElement::State* state)
{
    map<int, double> action_values = compute_action_values(state);
    pair<QElement::Action, double> action_to_value = learning_algorithms[cur_dimension-1]->random_action(action_values);
    update_args.action = action_to_value.first;
    update_args.old_value = action_to_value.second;
    return action_to_value.first;
}

void QReuse::update(QElement::State* old_state, QElement::Action old_action, QElement::State* new_state, double reward)
{
    map<int, double>::const_iterator max_it;
    map<int, double> new_action_values;
    for(set<int>::iterator used_proj_it = used_projections.begin(); used_proj_it != used_projections.end(); used_proj_it++)
    {
        int i = *used_proj_it;
        pca_interface->set_projection_dimension(i);
        QElement::State down_state;
        if(i == settings.total_dimensions)
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
            if(cur_dimension == settings.total_dimensions) new_action_values[it->first] += it->second;
            else new_action_values[it->first] += it->second + 100;
        }
    }

    update_args.next_state_action_values = new_action_values;
    update_args.reward = reward;
    pca_interface->set_projection_dimension(cur_dimension);

    if(cur_dimension != settings.total_dimensions)
    {
        map<int, double> action_values_old = compute_action_values(old_state);
        learning_algorithms[cur_dimension-1]->e_update(update_args);
        map<int, double> action_values_new = compute_action_values(old_state);

        double max_old, max_new;
        int action_old, action_new;
        for(unsigned int i = 0; i < settings.possible_actions.size(); i++)
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
        learning_algorithms[cur_dimension-1]->e_update(update_args);
    }

}

void QReuse::solve_manifold(vector<QElement::State>* states, int amount)
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

map<int, double> QReuse::compute_action_values(QElement::State* state)
{
    map<int, double> action_values;
    for(set<int>::iterator used_proj_it = used_projections.begin(); used_proj_it != used_projections.end(); used_proj_it++)
    {
        int i = *used_proj_it;
        pca_interface->set_projection_dimension(i);
        QElement::State down_state;
        if(i == settings.total_dimensions)
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
            if(cur_dimension == settings.total_dimensions) action_values[it->first] += it->second;
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
    update_args.old_probabilities = learning_algorithms[cur_dimension-1]->old_probabilities;
    update_args.states_to_update = learning_algorithms[cur_dimension-1]->old_states;
    pca_interface->set_projection_dimension(cur_dimension);
    return action_values;
}


void QReuse::set_range(vector<double> ranges_min_input, vector<double> ranges_max_input)
{
    settings.ranges_min = ranges_min_input;
    settings.ranges_max = ranges_max_input;
}

void QReuse::set_resolution(vector<double> resolution_input)
{
    settings.resolution = resolution_input;
}

int QReuse::get_table_size()
{
    return learning_algorithms[cur_dimension-1]->get_table_size();
}

void QReuse::clear_trace()
{
    learning_algorithms[cur_dimension-1]->clear_trace();
}

bool QReuse::is_converged()
{
    if(settings.total_dimensions == cur_dimension) return false;
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
        num = 0;
        return true;
    }
    else return false;

}

void QReuse::set_possible_actions(vector<QElement::Action> new_possible_actions)
{
    settings.possible_actions = new_possible_actions;

    for(unsigned int i = 0; i < learning_algorithms.size(); i++)
    {
        learning_algorithms[i]->set_possible_actions(settings.possible_actions);
    }

}
void QReuse::compute_all_states()
{
    vector<int> num_states;
    for(unsigned int i = 0; i < settings.ranges_min.size(); i++)
    {
        num_states.push_back((settings.ranges_max[i] - settings.ranges_min[i])/settings.resolution[i]);
        cout << num_states[i] << endl;
    }
    cout << endl;
    vector<vector<double> > all_values;
    for(unsigned int i = 0; i < num_states.size(); i++)
    {
        cout << num_states[i] << endl;
        vector<double> values;
        //values.push_back(ranges_min[i]);
        for(unsigned int j = 0; j < num_states[i]; j++)
        {
            values.push_back(settings.ranges_min[i] + (settings.resolution[i] * (j)));
        }
        //if(values.back() != ranges_max[i]) values.push_back(ranges_max[i]);
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
        locations.back()++;
        //Normalize?
        //        for(unsigned int j = 0; j < s.size(); j++)
        //        {
        //            s[j] = (s[j] - ranges_min[j])/(ranges_max[j] - ranges_min[j]);
        //        }
        all_states.push_back(s);
    }
}


QReuse::~QReuse()
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
