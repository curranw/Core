#include <QTiles.h>
#include <limits>
inline int range_rand( int maxRange )
{
    return (int) ( drand48() * maxRange );
}

inline int range_rand( int minRange, int maxRange )
{
    return range_rand( maxRange - minRange ) + minRange;
}

QTiles::QTiles(QTilesArguments* args) : ILearningAlgorithm(args)
{
    no_new = false;
    test_it = 1;
    alpha = args->alpha;
    gamma = args->gamma;
    num_tiles = args->num_tiles;
    m_args = args;
    cur_it = 0;
    count = 0;
    cur_eps = .1;
    tiles_array = new int[num_tiles];

    m_resolution = args->resolution;

}

void QTiles::init()
{
    for(unsigned int i = 0; i < m_possible_actions.size(); i++)
    {
        V_local[i] = 0;
    }
    if(m_args->eligability) clear_trace();
}

void QTiles::output(string file)
{
    vector<vector<double> > output_data;
    vector<double> alg_info;
    alg_info.push_back(num_tiles);
    output_data.push_back(alg_info);
    output_data.push_back(m_resolution);

    for(unordered_map<int, QElement*>::iterator it = q_table_m.begin(); it != q_table_m.end(); it++)
    {
        vector<double> state_num;
        state_num.push_back(double(it->first));
        output_data.push_back(state_num);
        output_data.push_back(it->second->v);
    }
    utils::to_csv(&output_data, file);
}

void QTiles::read(string file)
{
    vector<vector<double> > input_data = utils::read_newest_csv(file);
    if(input_data.empty()) return;
    num_tiles = input_data[0][0];
    m_resolution = input_data[1];

    for(unsigned int i = 2; i < input_data.size(); )
    {
        int tile_num = input_data[i++][0];
        //QElement::State state = input_data[i++];
        //vector<QElement::Action> actions(input_data[i].begin(), input_data[i++].end());
        vector<double> values = input_data[i++];
        QElement* ele = new QElement();
        ele->s = vector<double>();
        ele->a = vector<int>();
        ele->v = values;
        q_table_m.insert(make_pair(tile_num, ele));
    }
    memory_size = 128000;
}

pair<QElement::Action, double> QTiles::get_action(QElement::State s)
{
    if(m_args->eligability && eligibility_action_value.first != -1)
    {
        vector<QElement*> nearby_states = calculate_nearby_states(s);
        vector<double> V = get_action_values(nearby_states);
        eligibility_action_value.second = V[eligibility_action_value.first];
        return eligibility_action_value;
    }
    vector<QElement*> nearby_states = calculate_nearby_states(s);
    vector<double> V = get_action_values(nearby_states);

    pair<QElement::Action, double> action_values = random_action(V);
    //cout << action_values.first << endl;
    return action_values;
}

pair<QElement::Action, double> QTiles::random_action(vector<double> action_values)
{
    //Return best action
    double rand_val = (double)rand() / RAND_MAX;
    int max_it;
    double max_val;
    if(rand_val < 0.1)
    {
        max_it = rand() % action_values.size();
        max_val = action_values[max_it];
    }
    else
    {
        vector<int> list_of_max;
        for(int i = 0; i < action_values.size(); i++)
        {
            if(i == 0)
            {
                list_of_max.push_back(i);
                max_it = i;
                max_val = action_values[i];
            }
            else if (action_values[i] > max_val)
            {
                list_of_max.clear();
                max_it = i;
                max_val = action_values[i];
                list_of_max.push_back(i);
            }
            else if (abs(action_values[i] - max_val) < 0.000001)
            {
                list_of_max.push_back(i);
            }
        }
        max_it = list_of_max[rand() % list_of_max.size()];
        max_val = action_values[max_it];
    }

    pair<QElement::Action, double> return_val;
    return_val = make_pair(max_it, max_val);
    return return_val;

}


vector<double> QTiles::get_action_values(vector<QElement*> nearby_states)
{
    //unordered_map<QElement::Action, double> V;

    vector<double> vec(m_possible_actions.size());
    for(unsigned int i = 0; i < nearby_states.size(); i++)
    {
        QElement* ele = nearby_states[i];
        for(unsigned int j = 0; j < m_possible_actions.size(); j++)
        {
            vec[j] += ele->v[j];
        }
    }
    return vec;
}



vector<double> QTiles::get_action_values(QElement::State s)
{
    vector<QElement*> nearby_elements = calculate_nearby_states(s);
    return get_action_values(nearby_elements);
    //cout << s.at(0) << endl;
    //cout << nearby_elements.at(0)->s.at(0) << endl;
    //cout << nearby_elements.at(1)->s.at(0) << endl;
    //cout << nearby_elements.at(1)->s.size() << endl;
    //exit(1)
}

vector<QElement*> QTiles::calculate_nearby_states(QElement::State s)
{
    if(test_it++ % 1000000 == 0)
    {
        test_it = 1;
        cout << s.size() << "," << q_table_m.size() << "," << cur_eps << endl;
    }
    int num_states = s.size();
    float projected_state[s.size()];
    if(s.size() != m_resolution.size())
    {
        cout << "State size != resolution" << endl;
        exit(1);
    }
    for(unsigned int i = 0; i < s.size();i++)
    {
        projected_state[i] = (s[i] - m_min_ranges[i])/(m_max_ranges[i]-m_min_ranges[i]);
        if(projected_state[i] > 1 || projected_state[i] < 0)
        {
            cout << "State not scaled" << endl;
            cout << "Projected: ";
            for(unsigned int i = 0; i < m_min_ranges.size(); i++)
            {
                cout << projected_state[i] << ",";
            }
            cout << endl;
            cout << "Original: ";
            for(unsigned int i = 0; i < s.size(); i++)
            {
                cout << s[i] << ",";
            }
            cout << endl;
            exit(1);
        }
    }
    //double* projected_state = &s[0];
    for(unsigned int i = 0; i < s.size(); i++)
    {
        projected_state[i] = projected_state[i]/m_resolution[i];
    }

    if(q_table_m.empty())
    {
        memory_size = 1;
        for(int i = 0; i < s.size(); i++)
        {
            memory_size = memory_size * 1.0/m_resolution[i];
            //memory_size = memory_size * 1.0/m_resolution[i];
        }
        memory_size = memory_size * num_tiles * 10;
        memory_size = numeric_limits<int>::max();
    }
    //cout << m_resolution[0] << endl;
    //exit(1);

    //tileswrap(tiles_array,num_tiles,memory_size,projected_state,num_states, wrap_array, temp, 0);
    tiles(tiles_array,num_tiles,128000,projected_state,num_states);
    //GetTiles(tiles_array,tiles,memory_size,projected_state,num_states);



    vector<QElement*> nearby_states;
    for(int i = 0; i < num_tiles; i++)
    {
        //nearby_states.push_back(q_table[tiles_array[i]]);
        unordered_map<int, QElement*>::iterator it = q_table_m.find(tiles_array[i]);
        if(it != q_table_m.end())
        {
            nearby_states.push_back(it->second);
        }
        else
        {
            if(!no_new)
                nearby_states.push_back(add_tiles(tiles_array[i]));
        }
    }
    if(s[0] == -0.5)
    {
        //cout << tiles_array[0] << "," << tiles_array[1] << "," << tiles_array[2] << endl;
    }
    old_states = nearby_states;
    return nearby_states;

}

QElement* QTiles::add_tiles(int tile)
{

    QElement* ele = new QElement();
    for(unsigned int j = 0; j < m_possible_actions.size(); j++)
    {
        //ele->v.push_back(double(rand())/RAND_MAX);
        ele->v.push_back((double(rand())/RAND_MAX)/100.0);
        //ele->a.push_back(j);
    }
    q_table_m.insert(make_pair(tile,ele));

    if(m_args->eligability) eligibility.insert(make_pair(ele, vector<double>(m_possible_actions.size())));
    return ele;
}
void QTiles::update(QUpdate update)
{
    if(m_args->eligability)
    {
        e_update(update);
        return;
    }
    vector<QElement*> nearby_states_old;
    vector<double> old_V;
    vector<double> new_V;
    if(update.next_state_action_values.empty())
    {
        nearby_states_old = calculate_nearby_states(update.state);
        old_V = get_action_values(nearby_states_old);
        if(old_V[update.action] != update.old_value)
        {
            cout << "OMG I DID THIS FOR A REASON" << endl;
            cout << "OMG I DID THIS FOR A REASON" << endl;
            cin.get();
        }
        //update.old_value = old_V[update.action];
        vector<QElement*> nearby_states_new = calculate_nearby_states(update.next_state);
        new_V = get_action_values(nearby_states_new);
    }
    else
    {
        new_V = update.next_state_action_values;
        //old_V = get_action_values(update.states_to_update);
    }
    double max_new = -999999;
    for(int i = 0; i < new_V.size(); i++)
    {
        double value = new_V[i];
        if (value > max_new)
        {
            max_new = value;
        }
    }
    //double delta = update.reward - update.old_value;
    double delta = update.reward + gamma * max_new - update.old_value;

    if(delta < 0)
    {
        //cout << "BREAK" << endl;
    }
    if(update.states_to_update.empty())
    {
        update.states_to_update = nearby_states_old;
    }

    for(unsigned int i = 0; i < update.states_to_update.size(); i++)
    {
        update.states_to_update[i]->v[update.action] += alpha/num_tiles * delta;
    }
}

void QTiles::e_update(QElement::State state, QElement::Action action, QElement::State new_state,double reward)
{
    /*
    vector<QElement*> nearby_states_old = calculate_nearby_states(state);
    unordered_map<QElement::Action, double> old_V = get_action_values(nearby_states_old);

    vector<QElement*> nearby_states_new = calculate_nearby_states(new_state);
    unordered_map<QElement::Action, double> new_V = get_action_values(nearby_states_new);

    double max_new = -999999;
    for(unordered_map<QElement::Action, double>::iterator it = new_V.begin(); it != new_V.end(); it++)
    {
        double value = it->second;
        if (value > max_new)
        {
            max_new = value;
        }
    }

    //    vector<double> max_value;
    //    for(unsigned int i = 0; i < nearby_states_new.size(); i++)
    //    {
    //        max_value.push_back(-999999);
    //        for(unsigned int j = 0; j < nearby_states_new[i]->v.size(); j++)
    //        {
    //            double value = nearby_states_new[i]->v[j];
    //            if(value > max_value[i])
    //            {
    //                max_value[i] = value;
    //            }
    //        }
    //    }





    //Update e-trace

    for(unsigned int i = 0; i < nearby_states_old.size(); i++)
    {
        //cout << nearby_states_old[i] << ",";
        vector<double>* e = &eligability[nearby_states_old[i]];
        for(unsigned int j = 0; j < e->size(); j++)
        {
            if(j != action || (e->at(j) < 0.01 && e->at(j) > 0))
            {
                e->at(j) = 0;
            }
            else
            {
                e->at(j) += 1;
                ele_to_update.insert(nearby_states_old[i]);
            }
        }
    }
    //cout << endl;
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
    //    cout << num_erased << endl;
    //    cout << ele_to_update.size() << endl;
    //    static int it = 0;
    //    if(++it == 5) exit(1);
    double delta = reward + gamma * max_new - old_V[action];
    //#pragma omp parallel for
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); i++)
    {
        QElement* cur_ele = *i;
        //int location = *find(ele->a.begin(), ele->a.end(), old_a);
        vector<double>* e = &eligability[cur_ele];
        for(unsigned int j = 0; j < cur_ele->v.size(); j++)
        {
            cur_ele->v[j] += (alpha/tiles) * delta * e->at(j);
            e->at(j) = gamma * 0.1 * e->at(j);
        }
    }
    */
}



void QTiles::e_update(QUpdate update)
{
    vector<QElement*> nearby_states_old;
    vector<double> old_V;
    vector<double> new_V;
    if(update.next_state_action_values.empty())
    {
        nearby_states_old = calculate_nearby_states(update.state);
        old_V = get_action_values(nearby_states_old);
        if(old_V[update.action] != update.old_value)
        {
            cout << "OMG I DID THIS FOR A REASON" << endl;
            cout << "OMG I DID THIS FOR A REASON" << endl;
            cin.get();
        }
        //update.old_value = old_V[update.action];
        vector<QElement*> nearby_states_new = calculate_nearby_states(update.next_state);
        new_V = get_action_values(nearby_states_new);
    }
    else
    {
        new_V = update.next_state_action_values;
        //old_V = get_action_values(update.states_to_update);
    }
    double max_new = -999999;
    int max_action = 0;
    for(int i = 0; i < new_V.size(); i++)
    {
        double value = new_V[i];
        if (value > max_new)
        {
            max_action = i;
            max_new = value;
        }
    }

    eligibility_action_value.first = -1;
    eligibility_action_value = get_action(update.next_state);
    double delta = update.reward + gamma * max_new - update.old_value;

    if(update.states_to_update.empty())
    {
        update.states_to_update = nearby_states_old;
    }

    //Update e-trace
    for(unsigned int i = 0; i < update.states_to_update.size(); i++)
    {
        vector<double>* e = &eligibility[update.states_to_update[i]];
        for(unsigned int j = 0; j < e->size(); j++)
        {
            if(j == update.action)
            {
                e->at(j) += 1;
                ele_to_update.insert(update.states_to_update[i]);
            }
        }
    }

    //#pragma omp parallel for
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); i++)
    {
        QElement* cur_ele = *i;
        //int location = *find(ele->a.begin(), ele->a.end(), old_a);
        vector<double>* e = &eligibility[cur_ele];
        for(unsigned int j = 0; j < cur_ele->v.size(); j++)
        {
            if(e->at(j) == 0) continue;
            cur_ele->v[j] += alpha/num_tiles * delta * e->at(j);

            if(max_action == eligibility_action_value.first) e->at(j) = gamma * 0.5 * e->at(j);
            else e->at(j) = 0;

            if(e->at(j) < 0.01) e->at(j) = 0;

        }
    }

}

void QTiles::clear_trace()
{
    eligibility_action_value.first = -1;
    eligibility_action_value.second = 0;
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); i++)
    {
        QElement* cur_ele = *i;
        std::fill(eligibility[cur_ele].begin(), eligibility[cur_ele].end(), 0);
    }
}


void QTiles::update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward)
{
    /*
    vector<QElement*> nearby_states_old = calculate_nearby_states(old_s);
    unordered_map<QElement::Action, double> old_V = get_action_values(nearby_states_old);

    vector<QElement*> nearby_states_new = calculate_nearby_states(new_s);
    unordered_map<QElement::Action, double> new_V = get_action_values(nearby_states_new);

    double max_new = -999999;
    for(unordered_map<QElement::Action, double>::iterator it = new_V.begin(); it != new_V.end(); it++)
    {
        double value = it->second;
        if (value > max_new)
        {
            max_new = value;
        }
    }

    double delta = reward + gamma * max_new - old_V[old_a];
    for(unsigned int i = 0; i < nearby_states_old.size(); i++)
    {
        nearby_states_old[i]->v[old_a] += alpha/tiles * delta;
    }
    */
}

int QTiles::get_table_size()
{
    return q_table_m.size();
}

void QTiles::end_epoch()
{
    if(m_args->eligability) clear_trace();
}

QTiles::~QTiles()
{
    for(unordered_map<int, QElement*>::iterator it = q_table_m.begin(); it != q_table_m.end(); it++)
    {
        delete it->second;
    }
    delete[] tiles_array;
    delete m_args;
}



