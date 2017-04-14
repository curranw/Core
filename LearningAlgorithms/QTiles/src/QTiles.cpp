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
    use_tiles = true;
}

void QTiles::init()
{
    for(unsigned int i = 0; i < m_possible_actions.size(); i++)
    {
        V_local[i] = 0;
    }
    if(m_args->eligability) clear_trace();
    if(!use_tiles)
    {
        uint num_inputs = m_resolution.size();
        uint num_outputs = m_possible_actions.size();
        uint num_layers = 3;
        //uint num_neurons_hidden = num_inputs * 25;
        uint num_neurons_hidden = 200;
        //        std::array<uint, num_layers> layers = {num_inputs, num_neurons_hidden, num_neurons_hidden, num_neurons_hidden, num_outputs};;
        fann_net = new FANN::neural_net(FANN::LAYER, num_layers, num_inputs, num_neurons_hidden, num_outputs);

        fann_net->set_activation_function_hidden(FANN::SIGMOID_SYMMETRIC_STEPWISE);
        fann_net->set_activation_function_output(FANN::LINEAR);

        //fann_net->set_training_algorithm(FANN::TRAIN_BATCH);
        fann_net->set_training_algorithm(FANN::TRAIN_BATCH);
        fann_net->set_train_error_function(FANN::ERRORFUNC_TANH);
        fann_net->set_learning_rate(0.01);
        fann_net->set_activation_steepness_hidden(0.1);
        fann_net->set_activation_steepness_output(0.1);
        //fann_net->randomize_weights(-10, 10);

        //        nn = new NeuralNet(m_resolution.size(), 24, m_possible_actions.size());
        nn_qele = new QElement();
        nn_qele->v.assign(m_possible_actions.size(),0);




        //            for(unsigned int tot = 0; tot < 10000; tot++)
        //            {
        //                fann_net->reset_MSE();
        //                for(double i = 1; i < 10; i+=0.1)
        //                {
        //                    vector<double> input;
        //                    input.push_back(i);
        //                    vector<double> output;
        //                    output.push_back(i*i);
        //                    fann_type* output_a = &output[0];
        //                    fann_type* input_a = &input[0];
        //                    fann_net->train(input_a,output_a);
        //                }
        //                cout << fann_net->get_MSE() << endl;
        //            }


        //            cout << "TEST" << endl;
        //            for(unsigned int i = 1; i < 10; i++)
        //            {
        //                vector<double> input;
        //                input.push_back(i);
        //                fann_type* input_a = &input[0];
        //                cout << *fann_net->run(input_a) << endl;
        //            }

    }
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

void QTiles::read(string file)
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

void QTiles::set_table(unordered_map<int, QElement*> table)
{
    cout << "Setting Table:" << table.size() << endl;
    q_table_m = table;
}

double QTiles::get_average()
{
        double average = 0;
        int tot = 0;
        for(unordered_map<int, QElement*>::iterator it = q_table_m.begin(); it != q_table_m.end(); it++)
        {
            QElement* ele = it->second;
            for(unsigned int i = 0; i < ele->v.size(); i++)
            {
                if(ele->v[i] == 0) continue;
                average += ele->v[i];
                tot++;
            }
        }
        average/=double(tot);
        return average;
}

void QTiles::set_average(double average)
{
    default_weight = average;
    cout << "DEFAULT WEIGHT: " << default_weight << endl;
}

vector<QElement*> QTiles::get_elements(int* input_tiles)
{
    vector<QElement*> nearby_states;
    for(int i = 0; i < num_tiles; i++)
    {
        //nearby_states.push_back(q_table[tiles_array[i]]);
        unordered_map<int, QElement*>::iterator it = q_table_m.find(input_tiles[i]);
        if(it != q_table_m.end())
        {
            nearby_states.push_back(it->second);
        }
        else
        {
            return vector<QElement*>();
        }
    }
    return nearby_states;
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
    if(!use_tiles)
    {
        //        for(unsigned int i = 0; i < s.size(); i++)
        //        {
        //            s[i] = (s[i] - m_min_ranges[i])/(m_max_ranges[i]-m_min_ranges[i]);
        //        }

        fann_type* val = fann_net->run(&s[0]);

        for(unsigned int i = 0; i < m_possible_actions.size(); i++)
        {
            nn_qele->v[i] = val[i];
        }

        //nn_qele->v = vector<double>(std::begin(val), std::end(val));
        vector<QElement*> temp(1);
        temp[0] = nn_qele;
        return temp;


        //        nn_qele->v = nn->operator ()(s);
        //        vector<QElement*> temp(1);
        //        temp[0] = nn_qele;
        //        return temp;
    }
    if(use_tiles)
    {
//        for(unsigned int i = 0; i < s.size(); i++)
//        {
//            s[i] = (s[i] - m_min_ranges[i])/(m_max_ranges[i]-m_min_ranges[i]) * 1;
//        }
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
        //    for(unsigned int i = 0; i < s.size();i++)
        //    {
        //        projected_state[i] = (s[i] - m_min_ranges[i])/(m_max_ranges[i]-m_min_ranges[i]);
        //        if(projected_state[i] > 1 || projected_state[i] < 0)
        //        {
        //            cout << "State not scaled" << endl;
        //            cout << "Projected: ";
        //            for(unsigned int i = 0; i < m_min_ranges.size(); i++)
        //            {
        //                cout << projected_state[i] << ",";
        //            }
        //            cout << endl;
        //            cout << "Original: ";
        //            for(unsigned int i = 0; i < s.size(); i++)
        //            {
        //                cout << s[i] << ",";
        //            }
        //            cout << endl;
        //            exit(1);
        //        }
        //    }
        //    for(unsigned int i = 0; i < s.size(); i++)
        //    {
        //        projected_state[i] = projected_state[i]/m_resolution[i];
        //    }

        if(q_table_m.empty())
        {
            //        memory_size = 1;
            //        for(int i = 0; i < s.size(); i++)
            //        {
            //            memory_size = memory_size * 1.0/m_resolution[i];
            //            //memory_size = memory_size * 1.0/m_resolution[i];
            //        }
            //        memory_size = memory_size * num_tiles;
            //        if(memory_size == 0)
            //        {
            //            memory_size = numeric_limits<int>::max();
            //        }
            memory_size = numeric_limits<int>::max();
        }
        memory_size = numeric_limits<int>::max();
        //cout << m_resolution[0] << endl;
        //exit(1);

        for(unsigned int i = 0; i < s.size(); i++)
        {
            projected_state[i] = s[i]/m_resolution[i];
        }


        //cout << memory_size << endl;
        //tileswrap(tiles_array,num_tiles,memory_size,projected_state,num_states, wrap_array, temp, 0);
        tiles(tiles_array,num_tiles,memory_size,projected_state,num_states);
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
                {
                    QElement* temp = add_tiles(tiles_array[i]);
                    nearby_states.push_back(temp);
                }
            }
        }
        static int blah = 0;
        blah++;
        if(blah % 1000001 == 0)
        {
            blah = 0;
            cout << "Table: " << m_min_ranges.size() << ":" << q_table_m.size() << endl;
        }
        old_states = nearby_states;
        return nearby_states;
    }
}

QElement* QTiles::add_tiles(int tile)
{

    QElement* ele = new QElement();
    for(unsigned int j = 0; j < m_possible_actions.size(); j++)
    {
        //ele->v.push_back(double(rand())/RAND_MAX);
        ele->v.push_back((double(rand())/RAND_MAX)/32.0);
//        ele->v.push_back(0);
        //ele->v.push_back(default_weight);
        //ele->used.push_back(false);
        //ele->a.push_back(j);
    }
    q_table_m.insert(make_pair(tile,ele));

    if(m_args->eligability) eligibility.insert(make_pair(ele, vector<double>(m_possible_actions.size())));
    return ele;
}
void QTiles::update(QUpdate update)
{
    minibatch = true;
    static int batch_size = 8;
    if(!use_tiles)
    {
        if(minibatch)
        {
            NN_Experience exp;
            exp.action = update.action;
            exp.next_state = update.next_state;
            exp.state = update.state;
            exp.low_next_state_action_values = update.next_state_action_values;
            exp.reward = update.reward;
            exp.test = update.next_state_action_values;


            //Compute static low-dim values
            vector<double> q_high_dim_value = get_action_values(update.state);
            exp.low_dim_values = update.old_values;
            for(unsigned int i = 0; i < exp.low_dim_values.size(); i++)
            {
                exp.low_dim_values[i] = (exp.low_dim_values[i] - q_high_dim_value[i]);
            }




            //test
//            if(update.state.size() == 2 && !experience.empty())
//            {
//                NN_Experience test = experience[0];
//                vector<double> q_high_dim_value = get_action_values(test.state);
//                vector<double> low;
//                for(unsigned int i = 0; i < test.test.size(); i++)
//                {
//                    low.push_back((test.test[i] - q_high_dim_value[i]));
//                }
//            }



            vector<double> q_high_dim_next_value = get_action_values(update.next_state);
            for(unsigned int i = 0; i < exp.low_next_state_action_values.size(); i++)
            {
                exp.low_next_state_action_values[i] = (exp.low_next_state_action_values[i] - q_high_dim_next_value[i]);
            }

            experience.push_back(exp);
            if(experience.size() > 10000)
            {
                std::swap(experience[rand() % experience.size()], experience.back());
                experience.pop_back();
            }

            if(experience.size() < batch_size) return;
            FANN::training_data* const fann_td = new FANN::training_data();
            fann_type** inputs_pp = new fann_type*[batch_size];
            //        for(unsigned int i = 0; i < inputs.size(); i++)
            //        {
            //            inputs_pp[i] = &inputs[i][0];
            //        }
            fann_type** outputs_pp = new fann_type*[batch_size];
            //        for(unsigned int i = 0; i < outputs.size(); i++)
            //        {
            //            outputs_pp[i] = &outputs[i][0];
            //        }


            vector<vector<double> > q_true_all(batch_size);

            for(unsigned int minibatch_num = 0; minibatch_num < batch_size; minibatch_num++)
            {
                int ind = rand() % experience.size();
                NN_Experience* nn_e = &experience[ind];
                inputs_pp[minibatch_num] = &nn_e->state[0];


                vector<double> q_true = get_action_values(nn_e->state);


                //Re-compute next-best action.
                vector<double> new_V = get_action_values(nn_e->next_state);

                for(unsigned int i = 0; i < nn_e->low_next_state_action_values.size(); i++)
                {
                    new_V[i] += nn_e->low_next_state_action_values[i];
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

                double t = q_true[nn_e->action];
                q_true[nn_e->action] = nn_e->reward + gamma * max_new;

                if(!nn_e->low_dim_values.empty())
                {
                    q_true[nn_e->action] -= nn_e->low_dim_values[nn_e->action];
                }

                q_true_all[minibatch_num] = q_true;
                outputs_pp[minibatch_num] = &q_true_all[minibatch_num][0];
                fann_net->reset_MSE();
                fann_net->train(&nn_e->state[0], &q_true[0]);
                fann_net->reset_MSE();
            }
//            fann_td->set_train_data(batch_size, update.state.size(), inputs_pp, 5, outputs_pp);
//            fann_net->train_epoch(*fann_td);
            delete inputs_pp;
            delete outputs_pp;
            delete fann_td;
        }
        else
        {
            vector<double> q_true;
            if(update.old_values.empty())
            {
                q_true = get_action_values(update.state);
            }
            else
            {
                q_true = update.old_values;
            }

            vector<double> new_V;
            if(update.next_state_action_values.empty())
            {
                new_V = get_action_values(update.next_state);
            }
            else
            {
                new_V = update.next_state_action_values;
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
            q_true[update.action] = update.reward + gamma * max_new;


            if(!update.old_values.empty())
            {
                vector<double> q_true_single = get_action_values(update.state);
                for(unsigned int i = 0; i < update.old_values.size(); i++)
                {
                    q_true[i] -= (update.old_values[i] - q_true_single[i]);
                }
            }

            for(unsigned int i = 0; i < update.state.size(); i++)
            {
                //update.state[i] = (update.state[i] - m_min_ranges[i])/(m_max_ranges[i]-m_min_ranges[i]);
            }

            //        double r = rand()/double(RAND_MAX);
            //        if(r < 0.05)
            //        {
            //            inputs.push_back(update.state);
            //            outputs.push_back(q_true);
            //        }


            fann_net->train(&update.state[0], &q_true[0]);
        }
        //fann_net->reset_MSE();

        //        vector<double> q_true = get_action_values(update.state);

        //        vector<double> new_V = get_action_values(update.next_state);
        //        double max_new = -999999;
        //        for(int i = 0; i < new_V.size(); i++)
        //        {
        //            double value = new_V[i];
        //            if (value > max_new)
        //            {
        //                max_new = value;
        //            }
        //        }

        //        q_true[update.action] = update.reward + gamma * max_new;
        //        nn->backProp(update.state, q_true);
    }
    if(use_tiles)
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

        double delta = update.reward + gamma * max_new - update.old_value;


//        nearby_states_old = calculate_nearby_states(update.state);
//        old_V = get_action_values(nearby_states_old);

//        if(old_V[update.action] != update.old_value)
//        {
//            double a = 1;
//            cout << "UH OH" << endl;
//            exit(1);
//        }
//        double low_dim = (update.old_value - old_V[update.action]);


//        if(diff != 0)
//        {
//            cout << "HI" << endl;
//        }
//        double delta;
//        if(update.state.size() == 2) delta = update.reward + gamma * max_new - (old_V[update.action]);
//        else delta = update.reward + gamma * max_new - (low_dim);

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
            //        update.states_to_update[i]->used[update.action] = true;
            update.states_to_update[i]->v[update.action] += alpha/num_tiles * delta;
        }
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

unordered_map<int, QElement *>* QTiles::get_table()
{
    return &q_table_m;
}

void QTiles::end_epoch()
{
    if(!use_tiles)
    {
        //        FANN::training_data* const fann_td = new FANN::training_data();
        //        fann_type** inputs_pp = new fann_type*[inputs.size()];
        //        for(unsigned int i = 0; i < inputs.size(); i++)
        //        {
        //            inputs_pp[i] = &inputs[i][0];
        //        }
        //        fann_type** outputs_pp = new fann_type*[outputs.size()];
        //        for(unsigned int i = 0; i < outputs.size(); i++)
        //        {
        //            outputs_pp[i] = &outputs[i][0];
        //        }
        //        fann_td->set_train_data(outputs.size(), 8, inputs_pp, 5, outputs_pp);
        //        fann_net->train_epoch(*fann_td);

        cout << "MSE: " << fann_net->get_MSE() << endl;
        fann_net->reset_MSE();
        //        outputs.clear();
        //        inputs.clear();

        //        delete inputs_pp;
        //        delete outputs_pp;
    }
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



