/*
 * Q_KNN.cpp
 *
 *  Created on: Jun 19, 2015
 *      Author: Will Curran
 */

#include <Q_KNN.h>

Q_KNN::Q_KNN(double ai, double gi) : alpha(ai), gamma(gi) {
    cur_it = 0;
    debug = false;
}

void Q_KNN::init_knn(unsigned int num_neighbors, int dim, double eps)
{
    ann_struct.num_neighbors = num_neighbors;
    ann_struct.dim = dim;
    ann_struct.eps = eps;
    ann_struct.dists = new ANNdist[ann_struct.num_neighbors];
    ann_struct.nn_idx = new ANNidx[ann_struct.num_neighbors];}

QElement::Action Q_KNN::get_action(QElement::State s)
{
    //s = scale(s);
    //Get nearby states and distances
    vector<QElement*> nearby_elements = calculate_nearby_states(s);
    if(debug)
    {
        cout << "Cur QElement::State = " << s[0] << " ";
        cout << "Nearby QElement::States = ";
        for(unsigned int i = 0; i < nearby_elements.size(); i++)
        {
            cout << nearby_elements.at(i)->s[0];
        }
        cout << endl;
        cout << "Nearby Q-Values = ";
        for(unsigned int i = 0; i < nearby_elements.size(); i++)
        {
            for(unsigned int j = 0; j < nearby_elements[i]->v.size(); j++)
            {
                cout << nearby_elements[i]->v[j]  << ",";
            }
            cout << endl;
        }
    }
    //Get values per action.
    vector<double> weights = calculate_weights(ann_struct.dists);
    vector<double> probabilities = calculate_probabilities(&weights);
    map<int, double> action_values = calculate_values(nearby_elements, &probabilities);

    //Save found states and values
    old_values.first = s;
    old_values.second = action_values;
    old_states = nearby_elements;
    old_probabilities = probabilities;

    //if(s[0] == -0.5) cout << action_values[0] << "," << action_values[1] << "," << action_values[2] << endl;

    return random_action(action_values).first;

}
pair<QElement::Action, double> Q_KNN::random_action(map<int, double> action_values)
{
    //Return best action
    double rand_val = (double)rand() / RAND_MAX;
    map<int, double>::const_iterator max_it;
    if(rand_val < 0.1)
    {
        max_it = action_values.begin();
        std::advance(max_it, rand() % action_values.size());
        old_value = max_it->second;
    }
    else
    {
        vector<map<int, double>::const_iterator> list_of_max;
        for(map<int, double>::iterator it = action_values.begin(); it != action_values.end(); it++)
        {
            if(it == action_values.begin())
            {
                list_of_max.push_back(it);
                max_it = it;
            }
            else if (it->second > max_it->second)
            {
                list_of_max.clear();
                max_it = it;
                list_of_max.push_back(it);
            }
            else if (abs(it->second - max_it->second) < 0.0001)
            {
                list_of_max.push_back(it);
            }
        }
        max_it = list_of_max[rand() % list_of_max.size()];
        old_value = max_it->second;
    }
    if(debug) cout << "Returned QElement::Action: " << max_it->first << " Value: " << max_it->second << endl;

    pair<QElement::Action, double> return_val;
    return_val = make_pair(max_it->first, max_it->second);
    return return_val;

}

void Q_KNN::update_state(QElement::State s, vector<double> V)
{
    vector<QElement*> nearby = calculate_nearby_states(s);
    for(unsigned int i = 0; i < nearby.size(); i++)
    {
        nearby[i]->v  = V;
    }
}

map<int, double> Q_KNN::get_action_values(QElement::State s)
{
    vector<QElement*> nearby_elements = calculate_nearby_states(s);
    //    cout << s.at(0) << endl;
    //    cout << nearby_elements.at(0)->s.at(0) << endl;
    //    cout << nearby_elements.at(1)->s.at(0) << endl;
    //    cout << nearby_elements.at(1)->s.size() << endl;
    //    exit(1);
    vector<double> weights = calculate_weights(ann_struct.dists);
    vector<double> probabilities = calculate_probabilities(&weights);
    map<int, double> values = calculate_values(nearby_elements, &probabilities);

    //old_values.first = s;
    old_states = nearby_elements;
    old_probabilities = probabilities;
    return values;
}

vector<QElement*> Q_KNN::calculate_nearby_states(QElement::State s)
{
    ann_struct.query_pt = annAllocPt(ann_struct.dim);
    for(unsigned int i = 0; i < s.size(); i++)
    {
        ann_struct.query_pt[i] = s[i];
    }
    if(cur_it == 0)
    {

        knn_search();
        //check_datapoints(s);
        cur_it++;
    }
    else
    {
        knn_search();
        //If the nearest state is too far away, make a new state.
        //check_datapoints(s);
    }

    delete[] ann_struct.query_pt;
    //Grab nearby states.
    vector<QElement*> nearby_states;
    for(unsigned int i = 0; i < cur_neighbors; i++)
    {
        QElement* cur_ele = q_table[ann_struct.nn_idx[i]];
        nearby_states.push_back(cur_ele);
    }

    //check_actions(nearby_states);
    return nearby_states;
}
void Q_KNN::knn_search()
{
    if(q_table.size() < ann_struct.num_neighbors)
    {
        cur_neighbors = q_table.size();
    }
    else
    {
        cur_neighbors = ann_struct.num_neighbors;
    }
    ann_struct.kd_tree->annkSearch(
                ann_struct.query_pt,
                cur_neighbors,
                ann_struct.nn_idx,
                ann_struct.dists,
                ann_struct.eps
                );
}
vector<double> Q_KNN::calculate_weights(ANNdistArray nearby_dists)
{
    vector<double> weights;
    double weights_sum = 0;

    double nearby_dists_sum = 0;
    for(unsigned int i = 0; i < cur_neighbors; i++)
    {
        nearby_dists_sum += nearby_dists[i];
    }
    for(unsigned int i = 0; i < cur_neighbors; i++)
    {
        //cout << nearby_dists[i] << ",";
        //nearby_dists[i] = nearby_dists[i]/nearby_dists_sum;
        nearby_dists[i] = nearby_dists[i];
        //cout << nearby_dists[i] << ",";
    }
    //cout << endl;

    for(unsigned int i = 0; i < cur_neighbors; i++)
    {
        weights.push_back(1/(1+nearby_dists[i]*nearby_dists[i]));
        weights_sum += weights[i];
    }
    return weights;
}

vector<double> Q_KNN::calculate_probabilities(vector<double>* weights)
{
    vector<double> probabilities;
    double weights_sum = 0;
    for(unsigned int i = 0; i < cur_neighbors; i++)
    {
        weights_sum += weights->at(i);
    }
    for(unsigned int i = 0; i < cur_neighbors; i++)
    {
        probabilities.push_back(weights->at(i)/weights_sum);
    }
    return probabilities;
}

map<int, double> Q_KNN::calculate_values(vector<QElement*> nearby_states, vector<double>* probabilities)
{
    map<int, double> action_values;
    for(unsigned int i = 0; i < nearby_states[0]->a.size(); i++)
    {
        action_values.insert(make_pair(nearby_states[0]->a[i], 0));
    }

    for(unsigned int i = 0; i < cur_neighbors; i++)
    {
        QElement* ele = nearby_states[i];
        int j = 0;
        for(vector<QElement::Action>::iterator it = ele->a.begin(); it != ele->a.end(); it++)
        {
            if(action_values.find(*it) == action_values.end())
            {
                j++;
                continue;
            }
            action_values[*it] += nearby_states[i]->v[j++] * probabilities->at(i);
        }
    }
    return action_values;
}

void Q_KNN::update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward)
{
    //old_s = scale(old_s);
    //new_s = scale(new_s);
    //If (somehow) the old values were overwritten, recalculate them.
    if(old_values.first != old_s)
    {
        cout << "here" << endl;
        vector<QElement*> nearby_elements = calculate_nearby_states(old_s);
        vector<double> weights = calculate_weights(ann_struct.dists);
        vector<double> probabilities = calculate_probabilities(&weights);
        map<int, double> action_values = calculate_values(nearby_elements, &probabilities);
        old_values.second = action_values;
        old_values.first = old_s;
        old_probabilities = probabilities;
        old_states = nearby_elements;
        old_value = action_values[old_a];
    }

    //Calculate values for new state.
    vector<QElement*> nearby_elements = calculate_nearby_states(new_s);
    vector<double> weights = calculate_weights(ann_struct.dists);
    vector<double> probabilities = calculate_probabilities(&weights);
    map<int, double> new_values = calculate_values(nearby_elements, &probabilities);

    //Calculate delta

    map<int, double>::const_iterator max_it;
    for(map<int, double>::iterator it = new_values.begin(); it != new_values.end(); it++)
    {
        if(it == new_values.begin()) max_it = it;
        else if (it->second > max_it->second) max_it = it;
    }
    double max_new = max_it->second;

    for(map<int, double>::iterator it = old_values.second.begin(); it != old_values.second.end(); it++)
    {
        if(it == old_values.second.begin()) max_it = it;
        else if (it->second > max_it->second) max_it = it;
    }
    double max_old = max_it->second;
    double delta = reward + gamma * max_new - old_value;
    if(debug) cout << max_new << endl;
    //Update

    if(debug)
    {
        QElement::State s = old_states[0]->s;
        cout << "Cur QElement::State = " << s[0] << "," << s[1] << endl;
        cout << "Nearby QElement::States = ";
        for(unsigned int i = 0; i < old_states.size(); i++)
        {
            cout << old_states.at(i)->s[0];
        }
        cout << endl;
        cout << "Nearby Q-Values = ";
        for(unsigned int i = 0; i < old_states.size(); i++)
        {
            for(unsigned int j = 0; j < old_states[i]->v.size(); j++)
            {
                cout << old_states[i]->v[j]  << ",";
            }
            cout << endl;
        }
        cout << "Action: " << old_a << endl;
        cout << "Reward: " << reward << endl;
        cout << "Delta:" << delta << endl;
        cout << "Max New:" << max_new << endl;
        cout << "Max Old:" << max_old << endl;
    }
    for(unsigned int i = 0; i < old_states.size(); i++)
    {
        QElement* ele = old_states[i];
        int location = *find(ele->a.begin(), ele->a.end(), old_a);
        ele->v[location] += alpha * delta * old_probabilities[i];
    }
}

void Q_KNN::update(QUpdate update)
{
    /*
    //Calculate values for new state.
    unordered_map<int, double> new_action_values = update.next_state_action_values;
    //Calculate delta
    unordered_map<int, double>::const_iterator max_it;
    for(unordered_map<int, double>::iterator it = new_action_values.begin(); it != new_action_values.end(); it++)
    {
        if(it == new_action_values.begin()) max_it = it;
        else if (it->second > max_it->second) max_it = it;
    }
    double max_new = max_it->second;

    double delta = update.reward + gamma * max_new - update.old_value;

    for(unsigned int i = 0; i < update.states_to_update.size(); i++)
    {
        QElement* ele = update.states_to_update[i];
        int location = *find(ele->a.begin(), ele->a.end(), update.action);
        ele->v[location] += alpha * delta * update.old_probabilities[i];
    }

}

void Q_KNN::e_update(QUpdate update)
{
    //alpha *= .999999;
    //static int it2;
    //if(it2++ % 2000 == 0) cout << it2 << "," << alpha << endl;
    //Calculate values for new state.
    unordered_map<int, double> new_action_values = update.next_state_action_values;
    //Calculate delta
    unordered_map<int, double>::const_iterator max_it;
    for(unordered_map<int, double>::iterator it = new_action_values.begin(); it != new_action_values.end(); it++)
    {
        if(it == new_action_values.begin()) max_it = it;
        else if (it->second > max_it->second) max_it = it;
    }
    double max_new = max_it->second;


    //Update e-trace

    for(unsigned int i = 0; i < update.states_to_update.size(); i++)
    {
        vector<double>* e = &eligability[update.states_to_update[i]];
        for(unsigned int j = 0; j < e->size(); j++)
        {
            if(j != update.action || (e->at(j) < 0.01 && e->at(j) > 0))
            {
                e->at(j) = 0;
            }
            else
            {
                e->at(j) = update.old_probabilities[i];
                ele_to_update.insert(update.states_to_update[i]);
            }
        }
    }

    //Remove traces too small
    int num_erased = 0;
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); )
    {
        QElement* cur_ele = *i;
        vector<double>* e = &eligability[cur_ele];
        bool erase = true;
        for(unsigned int j = 0; j < e->size(); j++)
        {
            if(e->at(j) > 0.01 && e->at(j) != 0)
            {
                erase = false;
            }
        }
        if(erase)
        {
            num_erased++;
            for(unsigned int j = 0; j < e->size(); j++)
            {
                e->at(j) = 0;
            }
            ele_to_update.erase(i++);
            //set<QElement*>::iterator it= std::find(ele_to_update.begin(), ele_to_update.end(), cur_ele);
            //if(it != ele_to_update.end())
            //{
            //    ele_to_update.erase(it);
            //}
        }
        else
        {
            ++i;
        }
    }
    //#pragma omp parallel for
    double delta = update.reward + gamma * max_new - update.old_value;
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); i++)
    {
        QElement* cur_ele = *i;
        //int location = *find(ele->a.begin(), ele->a.end(), old_a);
        vector<double>* e = &eligability[cur_ele];
        for(unsigned int j = 0; j < cur_ele->v.size(); j++)
        {
            cur_ele->v[j] += alpha * delta * e->at(j);
            e->at(j) = gamma * 0.1 * e->at(j);
        }
    }
    */
}

void Q_KNN::e_update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward)
{
    //old_s = scale(old_s);
    //new_s = scale(new_s);
    //If (somehow) the old values were overwritten, recalculate them.
    if(old_values.first != old_s)
    {
        //cout << "FIRST != OLD" << endl;
        vector<QElement*> nearby_elements = calculate_nearby_states(old_s);
        vector<double> weights = calculate_weights(ann_struct.dists);
        vector<double> probabilities = calculate_probabilities(&weights);
        map<int, double> action_values = calculate_values(nearby_elements, &probabilities);
        old_values.second = action_values;
        old_values.first = old_s;
        old_probabilities = probabilities;
        old_states = nearby_elements;
        old_value = action_values[old_a];
    }

    //Calculate values for new state.
    vector<QElement*> nearby_elements = calculate_nearby_states(new_s);
    vector<double> weights = calculate_weights(ann_struct.dists);
    vector<double> probabilities = calculate_probabilities(&weights);
    map<int, double> new_values = calculate_values(nearby_elements, &probabilities);

    if(old_s.at(0) == -0.5)
    {
        //cout << new_values[0] << "," << new_values[1] << "," << new_values[2] << endl;
    }
    //Calculate delta

    map<int, double>::const_iterator max_it;
    for(map<int, double>::iterator it = new_values.begin(); it != new_values.end(); it++)
    {
        if(it == new_values.begin()) max_it = it;
        else if (it->second > max_it->second) max_it = it;
    }
    double max_new = max_it->second;

    for(map<int, double>::iterator it = old_values.second.begin(); it != old_values.second.end(); it++)
    {
        if(it == old_values.second.begin()) max_it = it;
        else if (it->second > max_it->second) max_it = it;
    }
    double max_old = max_it->second;
    double delta = reward + gamma * max_new - old_value;

    if(debug) cout << max_new << endl;


    //Updater
    if(debug)
    {
        QElement::State s = old_states[0]->s;
        cout << "Cur QElement::State = " << s[0] << "," << s[1] << endl;
        cout << "Nearby QElement::States = ";
        for(unsigned int i = 0; i < old_states.size(); i++)
        {
            cout << old_states.at(i)->s[0];
        }
        cout << endl;
        cout << "Nearby Q-Values = ";
        for(unsigned int i = 0; i < old_states.size(); i++)
        {
            for(unsigned int j = 0; j < old_states[i]->v.size(); j++)
            {
                cout << old_states[i]->v[j]  << ",";
            }
            cout << endl;
        }
        cout << "Action: " << old_a << endl;
        cout << "Reward: " << reward << endl;
        cout << "Delta:" << delta << endl;
        cout << "Max New:" << max_new << endl;
        cout << "Max Old:" << max_old << endl;
    }


    //Update e-trace

    for(unsigned int i = 0; i < old_states.size(); i++)
    {
        vector<double>* e = &eligability[old_states[i]];
        for(unsigned int j = 0; j < e->size(); j++)
        {
            if(j != old_a || (e->at(j) < 0.01 && e->at(j) > 0))
            {
                e->at(j) = 0;
            }
            else
            {
                e->at(j) = old_probabilities[i];
                ele_to_update.insert(old_states[i]);
            }
        }
    }

    //Remove traces too small
    int num_erased = 0;
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); )
    {
        QElement* cur_ele = *i;
        vector<double>* e = &eligability[cur_ele];
        bool erase = true;
        for(unsigned int j = 0; j < e->size(); j++)
        {
            if(e->at(j) > 0.01 && e->at(j) != 0)
            {
                erase = false;
            }
        }
        if(erase)
        {
            num_erased++;
            for(unsigned int j = 0; j < e->size(); j++)
            {
                e->at(j) = 0;
            }
            ele_to_update.erase(i++);
            //set<QElement*>::iterator it= std::find(ele_to_update.begin(), ele_to_update.end(), cur_ele);
            //if(it != ele_to_update.end())
            //{
            //    ele_to_update.erase(it);
            //}
        }
        else
        {
            ++i;
        }
    }

    //#pragma omp parallel for
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); i++)
    {
        QElement* cur_ele = *i;
        //int location = *find(ele->a.begin(), ele->a.end(), old_a);
        vector<double>* e = &eligability[cur_ele];
        for(unsigned int j = 0; j < cur_ele->v.size(); j++)
        {
            cur_ele->v[j] += alpha * delta * e->at(j);
            e->at(j) = gamma * 0.1 * e->at(j);
        }
    }
}

void Q_KNN::clear_trace()
{
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); i++)
    {
        {
            QElement* cur_ele = *i;
            std::fill(eligability[cur_ele].begin(), eligability[cur_ele].end(), 0);
        }
    }

}


int Q_KNN::get_table_size()
{
    return q_table.size();
}

void Q_KNN::set_possible_actions(vector<QElement::Action> new_possible_actions)
{
    possible_actions = new_possible_actions;
}

void Q_KNN::reset_table(vector<QElement*> new_table)
{
    cur_it = 0;
    q_table.clear();
    q_table = new_table;


    if(cur_it != 0) delete[] ann_struct.data_pts;

    ann_struct.data_pts = annAllocPts(q_table.size(), ann_struct.dim);
    for(unsigned int i = 0; i < q_table.size(); i++)
    {
        QElement* cur_ele = q_table[i];
        for(int j = 0; j < ann_struct.dim; j++)
        {
            ann_struct.data_pts[i][j] = cur_ele->s.at(j);
        }
    }
    ann_struct.n_pts = q_table.size();

    //Recreate the tree
    if(cur_it != 0) delete ann_struct.kd_tree;
    ann_struct.kd_tree = new ANNkd_tree(ann_struct.data_pts, ann_struct.n_pts, ann_struct.dim);


    cout << "NEW Q_TABLE!!" << endl;

    for(unsigned int i = 0; i < q_table.size(); i++)
    {
        eligability[q_table[i]] = vector<double>(possible_actions.size());
    }
    //    for(unsigned int i = 0; i < q_table.size(); i++)
    //    {
    //        QElement* ele = q_table[i];

    //        for(unsigned int j = 0; j < ele->s.size(); j++)
    //        {
    //            cout << ele->s[j] << ",";
    //        }
    //        cout << endl;
    //    }
}

void Q_KNN::get_nearest_state(QElement::State* s, QElement::State** nearest_state)
{
    QElement::State s2 = *s;
    ann_struct.query_pt = annAllocPt(ann_struct.dim);
    for(unsigned int i = 0; i < s2.size(); i++)
    {
        ann_struct.query_pt[i] = s2.at(i);
    }

    int save_neighbors = ann_struct.num_neighbors;
    ann_struct.num_neighbors = 1;
    knn_search();
    ann_struct.num_neighbors = save_neighbors;
    *nearest_state = &(q_table[ann_struct.nn_idx[0]]->s);

    delete[] ann_struct.query_pt;
}


void Q_KNN::set_range(vector<double> ranges_min_input, vector<double> ranges_max_input)
{
    ranges_min = ranges_min_input;
    ranges_max = ranges_max_input;
}

void Q_KNN::set_resolution(vector<double> resolution_input)
{
    resolution = resolution_input;
}
#include <unistd.h>
void Q_KNN::init_q_table()
{
    if(q_table.size() != 0)
    {
        q_table.clear();
        delete[] ann_struct.data_pts;
        delete ann_struct.kd_tree;
    }
    vector<int> num_states;
    for(unsigned int i = 0; i < ranges_min.size(); i++)
    {
        cout << ranges_max[i] << "," << ranges_min[i] << endl;
        cout << (ranges_max[i] - ranges_min[i]) << endl;
        cout << resolution[i] << endl;
        num_states.push_back((ranges_max[i] - ranges_min[i])/resolution[i]);
        cout << num_states[i] << endl;
    }
    vector<vector<double> > all_values;
    cout << endl;
    for(unsigned int i = 0; i < num_states.size(); i++)
    {
        cout << num_states[i] << endl;
        vector<double> values;
        //values.push_back(ranges_min[i]);
        for(unsigned int j = 0; j < num_states[i]; j++)
        {
            values.push_back(ranges_min[i] + (resolution[i] * (j)));
        }
        cout << values.size() << endl;
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

        QElement* ele = new QElement();
        //ele->s = scale(s);
        ele->s = s;
        ele->a = possible_actions;
        ele->v = vector<double>(possible_actions.size());
        q_table.push_back(ele);
        //cout << s[0] << "," << s[1] << "," << s[2] << endl;
        //cout << s[0] << endl;
    }
    cout << q_table.size() << endl;

    ann_struct.data_pts = annAllocPts(q_table.size(), ann_struct.dim);
    for(unsigned int i = 0; i < q_table.size(); i++)
    {
        QElement* cur_ele = q_table[i];
        for(int j = 0; j < ann_struct.dim; j++)
        {
            ann_struct.data_pts[i][j] = cur_ele->s.at(j);
        }
    }
    ann_struct.n_pts = q_table.size();

    //Recreate the tree
    cout << "current dimension!!!!" << ann_struct.dim << endl;

    ann_struct.kd_tree = new ANNkd_tree(ann_struct.data_pts, ann_struct.n_pts, ann_struct.dim);

    for(unsigned int i = 0; i < q_table.size(); i++)
    {
        eligability[q_table[i]] = vector<double>(possible_actions.size());
    }

}

Q_KNN::~Q_KNN() {
    cout << "Deleting data points" << endl;
    if(cur_it != 0) delete[] ann_struct.data_pts;

    cout << "Deleting Q Table" << endl;
    for(unsigned int i = 0; i < q_table.size(); i++)
    {
        delete q_table[i];
    }
    q_table.clear();
    cout << "Deleting kd tree" << endl;
    if(cur_it != 0) delete ann_struct.kd_tree;
    cout << "Deleting ann" << endl;
    annClose();
}
