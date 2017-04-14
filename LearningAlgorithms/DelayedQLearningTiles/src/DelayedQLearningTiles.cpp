#include <DelayedQLearningTiles.h>
#include <limits>
inline int range_rand( int maxRange )
{
    return (int) ( drand48() * maxRange );
}

inline int range_rand( int minRange, int maxRange )
{
    return range_rand( maxRange - minRange ) + minRange;
}

DelayedQLearningTiles::DelayedQLearningTiles(DelayedQLearningTilesArguments* args) : ILearningAlgorithm(args)
{
    m = 980;
    epsilon_error = 0.1;
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
    t = 0;
    cur_timestep = 0;

}

void DelayedQLearningTiles::init()
{

}

void DelayedQLearningTiles::output(string file)
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
    unsigned int* randseq = get_rand_seq(0);
    vector<unsigned int> randseq_output;
    for(int i = 0; i < 2048; i++)
    {
        //cout << randseq[i] << endl;
        randseq_output.push_back(randseq[i]);
    }
    utils::to_csv<unsigned int>(&randseq_output, file + "rnd_seq");
    utils::to_csv2d<double>(&output_data, file);
}

void DelayedQLearningTiles::read(string file)
{
    vector<vector<double> > input_data = utils::read_newest_csv<double>(file);
    if(input_data.empty()) return;
    num_tiles = input_data[0][0];
    m_resolution = input_data[1];

    for(unsigned int i = 2; i < input_data.size()-1; )
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

    vector<vector<unsigned int> > rnd_seq_input = utils::read_newest_csv<unsigned int>(file + "rnd_seq");

    unsigned int randseq[2048];
    for(unsigned int i = 0; i < rnd_seq_input[0].size(); i++)
    {
        randseq[i] = rnd_seq_input[0][i];
    }
    get_rand_seq(randseq);


    memory_size = 536870912;
}


pair<QElement::Action, double> DelayedQLearningTiles::get_action(QElement::State s)
{
    vector<QElement*> nearby_states = calculate_nearby_states(s);
    vector<double> V = get_action_values(nearby_states);

    pair<QElement::Action, double> action_values = random_action(V);
    return action_values;
}

pair<QElement::Action, double> DelayedQLearningTiles::random_action(vector<double> action_values)
{
    //Return best action
    int max_it;
    double max_val;
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

    pair<QElement::Action, double> return_val;
    return_val = make_pair(max_it, max_val);
    return return_val;

}


vector<double> DelayedQLearningTiles::get_action_values(vector<QElement*> nearby_states)
{
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

vector<double> DelayedQLearningTiles::get_update_values(vector<QElement *> nearby_states)
{
    vector<double> vec(m_possible_actions.size());
    for(unsigned int i = 0; i < nearby_states.size(); i++)
    {
        QElementDelayed* ele = (QElementDelayed*)nearby_states[i];
        for(unsigned int j = 0; j < m_possible_actions.size(); j++)
        {
            vec[j] += ele->u[j];
        }
    }
    return vec;
}



vector<double> DelayedQLearningTiles::get_action_values(QElement::State s)
{
    vector<QElement*> nearby_elements = calculate_nearby_states(s);
    return get_action_values(nearby_elements);
}

vector<QElement*> DelayedQLearningTiles::calculate_nearby_states(QElement::State s)
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
    if(q_table_m.empty())
    {
        memory_size = numeric_limits<int>::max();
    }

    for(unsigned int i = 0; i < s.size(); i++)
    {
        projected_state[i] = s[i]/m_resolution[i];
    }

    tiles(tiles_array,num_tiles,memory_size,projected_state,num_states);

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

    old_states = nearby_states;
    return nearby_states;

}

QElement* DelayedQLearningTiles::add_tiles(int tile)
{

    QElement* ele_v = new QElementDelayed();
    for(unsigned int j = 0; j < m_possible_actions.size(); j++)
    {
        ele_v->v.push_back((1.0/(1.0-m_args->gamma))/num_tiles);
        ((QElementDelayed*)(ele_v))->u.push_back(0);
        ((QElementDelayed*)(ele_v))->learn.push_back(true);
        ((QElementDelayed*)(ele_v))->l.push_back(0);
        ((QElementDelayed*)(ele_v))->t.push_back(0);
    }
    q_table_m.insert(make_pair(tile,ele_v));
    return ele_v;
}
void DelayedQLearningTiles::update(QUpdate update)
{
    update.states_to_update = calculate_nearby_states(update.state);
    bool update_now = false;
    cur_timestep++;
    double false_learns = 0;
    for(unsigned int i = 0; i < update.states_to_update.size(); i++)
    {
        QElementDelayed* ele = (QElementDelayed*)update.states_to_update[i];
        if(!ele->learn[update.action])
        {
            //update_now = false;
            if(ele->t[update.action] < t)
            {
                ele->learn[update.action] = true;
            }
            else continue;
        }

        vector<QElement*> nearby_states_new = calculate_nearby_states(update.next_state);
        vector<double> new_V = get_action_values(nearby_states_new);
        double max_new = -999999;
        for(int i = 0; i < new_V.size(); i++)
        {
            double value = new_V[i];
            if (value > max_new)
            {
                max_new = value;
            }
        }
        double delta = update.reward + gamma * max_new;

        ele->u[update.action] += 1.0/num_tiles * delta;
        ele->l[update.action] += 1;

        if(ele->l[update.action] >= m)
        {
            update_now = true;
        }
        //t-test for later
        if(ele->t[update.action] >= t) false_learns++;
    }
    false_learns/=update.states_to_update.size();
    if(update_now)
    {
        bool do_false_learns = false;
        vector<double> U = get_update_values(update.states_to_update);
        vector<double> Q = get_action_values(update.states_to_update);
        if(abs(Q[update.action] - U[update.action]/double(m)) > 2 * epsilon_error)
        {
            //cout << "UPDATE" << endl;
            double delta = U[update.action]/m + epsilon_error;
            for(unsigned int i = 0; i < update.states_to_update.size(); i++)
            {
                QElementDelayed* ele = (QElementDelayed*)update.states_to_update[i];

                if(ele->l[update.action] < m) continue;
                update.states_to_update[i]->v[update.action] = 1.0/num_tiles * delta;
            }
            t = cur_timestep;
        }
        else if(false_learns >= .50)
        {
            do_false_learns = true;
        }
        for(unsigned int i = 0; i < update.states_to_update.size(); i++)
        {
            QElementDelayed* ele = (QElementDelayed*)update.states_to_update[i];
            if(ele->l[update.action] < m) continue;
            ele->t[update.action] = cur_timestep;
            ele->u[update.action] = 0;
            ele->l[update.action] = 0;
            if(do_false_learns) ele->learn[update.action] = false;
        }
    }
//    if(m_args->eligability)
//    {
//        e_update(update);
//        return;
//    }
//    vector<QElement*> nearby_states_old;
//    vector<double> old_V;
//    vector<double> new_V;
//    if(update.next_state_action_values.empty())
//    {
//        nearby_states_old = calculate_nearby_states(update.state);
//        old_V = get_action_values(nearby_states_old);
//        if(old_V[update.action] != update.old_value)
//        {
//            cout << "OMG I DID THIS FOR A REASON" << endl;
//            cout << "OMG I DID THIS FOR A REASON" << endl;
//            cin.get();
//        }
//        //update.old_value = old_V[update.action];
//        vector<QElement*> nearby_states_new = calculate_nearby_states(update.next_state);
//        new_V = get_action_values(nearby_states_new);
//    }
//    else
//    {
//        new_V = update.next_state_action_values;
//        //old_V = get_action_values(update.states_to_update);
//    }
//    double max_new = -999999;
//    for(int i = 0; i < new_V.size(); i++)
//    {
//        double value = new_V[i];
//        if (value > max_new)
//        {
//            max_new = value;
//        }
//    }
//    //double delta = update.reward - update.old_value;
//    double delta = update.reward + gamma * max_new - update.old_value;

//    if(delta < 0)
//    {
//        //cout << "BREAK" << endl;
//    }
//    if(update.states_to_update.empty())
//    {
//        update.states_to_update = nearby_states_old;
//    }

//    for(unsigned int i = 0; i < update.states_to_update.size(); i++)
//    {
//        //        update.states_to_update[i]->used[update.action] = true;
//        update.states_to_update[i]->v[update.action] += alpha/num_tiles * delta;
//    }
}

void DelayedQLearningTiles::e_update(QElement::State state, QElement::Action action, QElement::State new_state,double reward)
{

}



void DelayedQLearningTiles::e_update(QUpdate update)
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

void DelayedQLearningTiles::clear_trace()
{
    eligibility_action_value.first = -1;
    eligibility_action_value.second = 0;
    for(set<QElement*>::iterator i = ele_to_update.begin(); i != ele_to_update.end(); i++)
    {
        QElement* cur_ele = *i;
        std::fill(eligibility[cur_ele].begin(), eligibility[cur_ele].end(), 0);
    }
}


void DelayedQLearningTiles::update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward)
{

}

int DelayedQLearningTiles::get_table_size()
{
    return q_table_m.size();
}

void DelayedQLearningTiles::end_epoch()
{
    if(m_args->eligability) clear_trace();
    cur_timestep = 0;
}

DelayedQLearningTiles::~DelayedQLearningTiles()
{
    for(unordered_map<int, QElement*>::iterator it = q_table_m.begin(); it != q_table_m.end(); it++)
    {
        delete it->second;
    }
    delete[] tiles_array;
    delete m_args;
}



