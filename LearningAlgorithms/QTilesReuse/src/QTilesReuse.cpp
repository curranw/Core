#include "QTilesReuse.h"
#include <limits>
#include <algorithm>
QTilesReuse::QTilesReuse(double dimensions, QTilesReuseArgs *args) : ILearningAlgorithm(args)
{
    pca_interface = new PCAInterface(dimensions);
    total_dimensions = dimensions;
    m_learning_algorithms = args->learning_algorithms;
    m_args = args;
    vector<vector<double> > pca_position_data = utils::read_csv<double>(args->pca_file);

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
    running_avg = 0;
    tot_reward = 0;
    cur_dim_it = 0;
}
void QTilesReuse::set_projection_dimension(int projection_dimension)
{
//    if(projection_dimension != 2) projection_dimension = 8;
    std::random_shuffle ( projectable_states.begin(), projectable_states.end() );
    static int pro = 0;
    pro++;
    cout << "Projection Dimension: " << projection_dimension << endl;
    if(total_dimensions < projection_dimension) return;
    cur_dimension = projection_dimension;

    //Test
    //used_projections.clear();

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
            double y = abs(m_max_ranges[i] - m_min_ranges[i])/(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i]));
              //temp_resolution.push_back(.25);
//            temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(32 - (projection_dimension*3.5)));

            temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(4.0));
//            if(i == 0) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(2.0));
//            if(i == 1) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(2.0));
//            if(i == 2) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(4.0));
//            if(i == 3) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(4.0));
//            if(i == 4) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(8.0));
//            if(i == 5) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(8.0));
//            if(i == 6) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(8.0));
//            if(i == 7) temp_resolution.push_back(abs(ranges_max_all[projection_dimension-1][i] - ranges_min_all[projection_dimension-1][i])/(8.0));


        }
        m_learning_algorithms[cur_dimension-1]->set_resolution(temp_resolution);



        m_learning_algorithms[cur_dimension-1]->set_ranges(ranges_min_all[projection_dimension-1], ranges_max_all[projection_dimension-1]);


        m_learning_algorithms[cur_dimension-1]->init();

//        cout << projection_dimension << endl;
//        vector<double> temp_res = m_resolution;
//        for(unsigned int i = 0; i < temp_res.size(); i++)
//        {
//            temp_res[i] = (m_max_ranges[i] - m_min_ranges[i])/8.0;;
//            cout << temp_res[i] << endl;
//        }
//        if(projectable_states.empty())
//        {
//            //Project Up
//            vector<int> num_states;
//            for(unsigned int i = 0; i < m_min_ranges.size(); i++)
//            {
//                num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/temp_res[i]);
//            }
//            vector<vector<double> > all_values;
//            for(unsigned int i = 0; i < num_states.size(); i++)
//            {
//                vector<double> values;
//                //values.push_back(ranges_min[i]);
//                for(unsigned int j = 0; j < num_states[i]; j++)
//                {
//                    values.push_back(m_min_ranges[i] + (temp_res[i] * (j)));
//                }
//                if(values.back() != m_max_ranges[i]) values.push_back(m_max_ranges[i]);
//                all_values.push_back(values);
//            }
//            vector<vector<double>::iterator> locations;
//            for(unsigned int i = 0; i < num_states.size(); i++)
//            {
//                locations.push_back(all_values[i].begin());
//            }

//            bool done = false;
//            while(true)
//            {
//                int cur_it = num_states.size()-1;
//                //Check if you're at the end of the last list. If you are, reset that list and increase the previous one.
//                while(true)
//                {
//                    if(locations[0] == all_values[0].end())
//                    {
//                        done = true;
//                        break;
//                    }
//                    if(locations[cur_it] == all_values[cur_it].end())
//                    {
//                        locations[cur_it] = all_values[cur_it].begin();
//                        cur_it--;
//                        locations[cur_it]++;
//                    }
//                    else break;
//                }
//                if(done) break;
//                QElement::State s;
//                for(unsigned int j = 0; j < locations.size(); j++)
//                {
//                    s.push_back(*locations[j]);
//                }
//                locations.back()++;
//                projectable_states.push_back(s);
//            }
//        }

//#include <utils.h>
//        utils::to_csv2d<double>(&projectable_states, "all_states_mountaincar_3d");
//        exit(1);
/*
        if(pro != 1)
        {

            cout << projection_dimension << endl;
            vector<double> temp_res = m_resolution;
            for(unsigned int i = 0; i < temp_res.size(); i++)
            {
                temp_res[i] = (m_max_ranges[i] - m_min_ranges[i])/4.0;;
                cout << temp_res[i] << endl;
            }
            if(projectable_states.empty())
            {
                //Project Up
                vector<int> num_states;
                for(unsigned int i = 0; i < m_min_ranges.size(); i++)
                {
                    num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/temp_res[i]);
                }
                vector<vector<double> > all_values;
                for(unsigned int i = 0; i < num_states.size(); i++)
                {
                    vector<double> values;
                    //values.push_back(ranges_min[i]);
                    for(unsigned int j = 0; j < num_states[i]; j++)
                    {
                        values.push_back(m_min_ranges[i] + (temp_res[i] * (j)));
                    }
                    if(values.back() != m_max_ranges[i]) values.push_back(m_max_ranges[i]);
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
                    projectable_states.push_back(s);
                }
            }


//            projectable_states.clear();
//            unordered_map<int, QElement *>* table = m_learning_algorithms[cur_dimension-1]->get_table();
//            for(unordered_map<int, QElement *>::iterator it = table->begin(); it != table->end(); it++)
//            {
//                projectable_states.push_back(it->second->s);
//            }
//            for(unordered_map<int, QElement *>*::iterator it = table.begin(); it != table.end(); it++)
//            {

//            }
            */


        /*
            //New Q-Table
            unordered_map<int, QElement*> q_table_new;
            int* tiles_array = new int[32];
            int* tiles_array_new = new int[32];
            int num_unchanged_size = 0;
            int prev_size = 0;
            for(unsigned int i = 0; i < projectable_states.size(); i++)
            {
                int num_tiles = 32;
                //Down Tiles
                pca_interface->set_projection_dimension(cur_dimension-1);
                vector<double> down = pca_interface->transform_down(&projectable_states[i]);
                float projected_state[down.size()];
                for(unsigned int j = 0; j < down.size(); j++)
                {
                    double resolution = abs(ranges_max_all[projection_dimension-2][j] - ranges_min_all[projection_dimension-2][j])/8.0;
                    projected_state[j] = down[j]/resolution;
                }
                int memory_size = numeric_limits<int>::max();

                int num_states = down.size();
                tiles(tiles_array,num_tiles,memory_size,projected_state,num_states);

                //New Tiles
                pca_interface->set_projection_dimension(cur_dimension);
                vector<double> down_new = pca_interface->transform_down(&projectable_states[i]);
                float projected_state_new[down_new.size()];
                for(unsigned int j = 0; j < down_new.size(); j++)
                {
                    double resolution_new = abs(ranges_max_all[projection_dimension-1][j] - ranges_min_all[projection_dimension-1][j])/8.0;
                    projected_state_new[j] = down_new[j]/resolution_new;
                }

                int num_states_new = down_new.size();
                tiles(tiles_array_new,num_tiles,memory_size,projected_state_new,num_states_new);

                vector<QElement*> old_elements = m_learning_algorithms[cur_dimension-2]->get_elements(tiles_array);

//                if(prev_size == q_table_new.size())
//                {
//                    num_unchanged_size++;
//                }
//                else
//                {
//                    prev_size = q_table_new.size();
//                    num_unchanged_size = 0;
//                }
//                if(q_table_new.size() == m_learning_algorithms[cur_dimension-2]->q_table_m.size()) break;
                if(old_elements.size() == 0) continue;
                //New Tiles
                unordered_map<int, QElement*>::iterator it;
                for(unsigned int ele = 0; ele < old_elements.size(); ele++)
                {
                    int tile = tiles_array_new[ele];
                    it = q_table_new.find(tile);
                    QElement* new_qele;
                    if(it == q_table_new.end())
                    {
                        new_qele = new QElement();
                        new_qele->v.assign(5, 0);
                        q_table_new.insert(make_pair(tile,new_qele));
                    }
                    else
                    {
                        new_qele = it->second;
                    }
                    for(unsigned int ele_v = 0; ele_v< new_qele->v.size(); ele_v++)
                    {
                        if(new_qele->v[ele_v] == 0) new_qele->v[ele_v] = old_elements[ele]->v[ele_v];
                    }
                }
            }
            delete[] tiles_array_new;
            delete[] tiles_array;

            double avg = m_learning_algorithms[cur_dimension-2]->get_average();
            for(unordered_map<int, QElement*>::iterator it = q_table_new.begin(); it != q_table_new.end(); it++)
            {
                for(unsigned int i = 0; i < it->second->v.size(); i++)
                {
                    if(it->second->v[i] == 0) it->second->v[i] = avg;
                }
            }
            m_learning_algorithms[cur_dimension-1]->set_table(q_table_new);
            //projectable_states.clear();
        }
*/
        //if(cur_dimension != 1) m_learning_algorithms[cur_dimension-1]->set_table(m_learning_algorithms[cur_dimension-2]);

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
        m_learning_algorithms[cur_dimension-1]->init();


/*
        //New Q-Table
        unordered_map<int, QElement*> q_table_new;
        int* tiles_array = new int[32];
        int* tiles_array_new = new int[32];
        for(unsigned int i = 0; i < projectable_states.size(); i++)
        {
            int num_tiles = 32;
            //Down Tiles
            pca_interface->set_projection_dimension(cur_dimension-1);
            vector<double> down = pca_interface->transform_down(&projectable_states[i]);
            float projected_state[down.size()];
            for(unsigned int j = 0; j < down.size(); j++)
            {
                double resolution = (ranges_max_all[projection_dimension-2][j] - ranges_min_all[projection_dimension-2][j])/8.0;
                projected_state[j] = down[j]/resolution;
            }
            int memory_size = numeric_limits<int>::max();

            int num_states = down.size();
            tiles(tiles_array,num_tiles,memory_size,projected_state,num_states);

            //New Tiles
            pca_interface->set_projection_dimension(cur_dimension);
            vector<double> down_new = projectable_states[i];
            float projected_state_new[down_new.size()];
            for(unsigned int j = 0; j < down_new.size(); j++)
            {
                projected_state_new[j] = down_new[j]/m_resolution[j];
            }

            int num_states_new = down_new.size();
            tiles(tiles_array_new,num_tiles,memory_size,projected_state_new,num_states_new);

            vector<QElement*> old_elements = m_learning_algorithms[cur_dimension-2]->get_elements(tiles_array);

//            if(q_table_new.size() == m_learning_algorithms[cur_dimension-2]->q_table_m.size()) break;
            if(old_elements.size() == 0) continue;
            //New Tiles
            unordered_map<int, QElement*>::iterator it;
            for(unsigned int ele = 0; ele < old_elements.size(); ele++)
            {
                int tile = tiles_array_new[ele];
                it = q_table_new.find(tile);
                QElement* new_qele;
                if(it == q_table_new.end())
                {
                    new_qele = new QElement();
                    new_qele->v.assign(5, 0);
                    q_table_new.insert(make_pair(tile,new_qele));
                }
                else
                {
                    new_qele = it->second;
                }
                for(unsigned int ele_v = 0; ele_v< new_qele->v.size(); ele_v++)
                {
                    if(new_qele->v[ele_v] == 0) new_qele->v[ele_v] = old_elements[ele]->v[ele_v];
                }
            }
        }


        double avg = m_learning_algorithms[cur_dimension-2]->get_average();
        for(unordered_map<int, QElement*>::iterator it = q_table_new.begin(); it != q_table_new.end(); it++)
        {
            for(unsigned int i = 0; i < it->second->v.size(); i++)
            {
                if(it->second->v[i] == 0) it->second->v[i] = avg;
            }
        }
        m_learning_algorithms[cur_dimension-1]->set_table(q_table_new);
        projectable_states.clear();

        */
    }


//        double avg_weight = 0;
//        for(unsigned int i = 0; i < projection_dimension-1; i++)
//        {
//            avg_weight += m_learning_algorithms[i]->get_average();
//        }
//        if(projection_dimension-1 != 0) avg_weight = avg_weight/double(projection_dimension-1);
//        m_learning_algorithms[cur_dimension-1]->set_average(avg_weight);
}

pair<QElement::Action, double> QTilesReuse::get_action(QElement::State state)
{
    vector<double> action_values = compute_action_values(&state);
    pair<QElement::Action, double> action_to_value = m_learning_algorithms[cur_dimension-1]->random_action(action_values);
    update_args.action = action_to_value.first;
    update_args.old_value = action_to_value.second;
    update_args.old_values = action_values;
    return action_to_value;
}

void QTilesReuse::update(QElement::State* old_state, QElement::Action old_action, QElement::State* new_state, double reward)
{
    /*
    int max_it;
    double max_val;
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

        vector<double> cur_action_values = m_learning_algorithms[i-1]->get_action_values(down_state);
        if(i == cur_dimension && i != 1)
        {
            for(unsigned int j = 0; j < new_action_values.size(); j++)
            {
                if(j == 0)
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
    */
}

void QTilesReuse::update(QUpdate update)
{
    if(cur_dimension != total_dimensions) projectable_states.push_back(update.state);
    unordered_map<int, double>::const_iterator max_it;
    vector<double> new_action_values(m_possible_actions.size());
    vector<int> best_actions(m_possible_actions.size());
    int best_next_action;
    for(set<int>::iterator used_proj_it = used_projections.begin(); used_proj_it != used_projections.end(); used_proj_it++)
    {
        int i = *used_proj_it;
        pca_interface->set_projection_dimension(i);
        QElement::State down_state;
        if(i == total_dimensions)
        {
            down_state = update.next_state;
            update_args.next_state = down_state;
        }
        else
        {
            down_state = pca_interface->transform_down(&update.next_state);
            if(i == cur_dimension) update_args.next_state = down_state;
        }

        vector<double> cur_action_values = m_learning_algorithms[i-1]->get_action_values(down_state);

        for(int j = 0; j < cur_action_values.size(); j++)
        {
            if(i == cur_dimension) new_action_values[j] += cur_action_values[j];
            else new_action_values[j] += cur_action_values[j];
        }
        if(i != cur_dimension)
        {
            int action = 0;
            double max = cur_action_values[0];
            for(int j = 1; j < cur_action_values.size(); j++)
            {
                if(cur_action_values[j] > max)
                {
                    action = j;
                    max = cur_action_values[j];
                }
            }
            best_actions[action]++;
        }
        if(i == cur_dimension)
        {
            best_next_action = 0;
            double max = new_action_values[0];
            for(int j = 1; j < new_action_values.size(); j++)
            {
                if(new_action_values[j] > max)
                {
                    best_next_action = j;
                    max = new_action_values[j];
                }
            }
        }
    }
    double potential = 0;
    if(cur_dimension != *used_projections.begin())
    {
        int action = 0;
        double max = best_actions[0];
        for(unsigned int i = 1; i < best_actions.size(); i++)
        {
            if(best_actions[i] > max)
            {
                action = i;
                max = best_actions[i];
            }
        }

        //if(best_next_action == action) potential += .1 * .99;
        if(update_args.action == update_args.best_action) potential -= .1;
    }

    update_args.next_state_action_values = new_action_values;
    update_args.reward = update.reward;// + potential;// * pow(.9999, cur_dim_it);
    pca_interface->set_projection_dimension(cur_dimension);

    if(cur_dimension != total_dimensions)
    {
        vector<double> action_values_old = compute_action_values(&update.state);
        m_learning_algorithms[cur_dimension-1]->update(update_args);
        vector<double> action_values_new = compute_action_values(&update.state);
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
    tot_reward += update.reward;
}


void QTilesReuse::solve_manifold(vector<QElement::State>* states, int amount)
{
    //    if(!m_args->weight_file.empty())
    //    {
    //        vector<vector<double> > weights = utils::read_csv<double>(m_args->weight_file);
    //        pca_interface->add_weights(&weights.at(0));
    //    }
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

void QTilesReuse::end_epoch()
{
    m_learning_algorithms[cur_dimension-1]->end_epoch();
}

vector<double> QTilesReuse::compute_action_values(QElement::State* state)
{
    vector<double> action_values(m_possible_actions.size());
    vector<int> best_actions(m_possible_actions.size());
    for(set<int>::iterator used_proj_it = used_projections.begin(); used_proj_it != used_projections.end(); used_proj_it++)
    {
        int i = *used_proj_it;
        QElement::State down_state;
        if(i == total_dimensions)
        {
            down_state = *state;
        }
        else
        {
            pca_interface->set_projection_dimension(i);
            down_state = pca_interface->transform_down(state);
        }
        update_args.state = down_state;
        vector<double> cur_action_values = m_learning_algorithms[i-1]->get_action_values(down_state);
        for(int j = 0; j < cur_action_values.size(); j++)
        {
            if(i == cur_dimension) action_values[j] += cur_action_values[j];
            else action_values[j] += cur_action_values[j];
        }
        if(i != cur_dimension)
        {
            int action = 0;
            double max = cur_action_values[0];
            for(int j = 1; j < cur_action_values.size(); j++)
            {
                if(cur_action_values[j] > max)
                {
                    action = j;
                    max = cur_action_values[j];
                }
            }
            best_actions[action]++;
        }
    }

    if(cur_dimension != *used_projections.begin())
    {
        int action = 0;
        double max = best_actions[0];
        for(unsigned int i = 1; i < best_actions.size(); i++)
        {
            if(best_actions[i] > max)
            {
                action = i;
                max = best_actions[i];
            }
        }
        //cout << action << endl;
        update_args.best_action = action;
    }
    static int it = 0;
    it++;
    if(state->at(0) == -0.5)
    {
        //cout << action_values[0] << "," << action_values[1] << "," << action_values[2] << endl;
        //if(cur_dimension == 2) exit(1);
    }
    //
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

void QTilesReuse::init()
{

}

void QTilesReuse::clear_trace()
{
    m_learning_algorithms[cur_dimension-1]->clear_trace();
}

bool QTilesReuse::is_converged()
{
    cur_dim_it++;
    //    cout << num_updates << endl;
    //    cout << num_similar << endl;
    //    exit(1);
    if(total_dimensions == cur_dimension) return false;
    double val = num_similar/num_updates;
    static double percentage = .99;
    //    last_running_avg = running_avg;
    //    running_avg = last_running_avg + (tot_reward - last_running_avg)/double(++it);
    //    tot_reward = 0;
    //if(it % 100 == 0) cout << percentage << endl;
    num_similar = 0;
    num_updates = 0;
    cout << val << endl;
    static int num = 0;
    percentage = percentage * .9999;
    if((val > .90 || cur_dim_it > 250) && (cur_dim_it > (50 * cur_dimension))) num++;
    else num = 0;
    if(num == 5)
    {
        cur_dim_it = 0;
        num = 0;
        percentage = .99;
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

