#include <ValueIteration.h>

ValueIteration::ValueIteration(ValueIterationArgs *args) : ILearningAlgorithm(args)
{
    m_resolution = args->resolution;
}

pair<QElement::Action, double> ValueIteration::get_action(QElement::State s)
{
    int ind = state_to_ind[s] * m_possible_actions.size();
    double max;
    int max_ind;
    for(unsigned int i = 0; i < m_possible_actions.size(); i++)
    {
        if(i == 0)
        {
            max = Q_best(ind + i);
            max_ind = i;
        }
        else if(max < Q_best(ind + i))
        {
            max = Q_best(ind + i);
            max_ind = i;
        }
    }
    //    cout << "----Action ----" << endl;
    //    cout << max_ind << endl;
    //    cout << max << endl;
    //    cout << ind << endl;
    //    cout << "-------" << endl;
    return make_pair(max_ind, max);
}

void ValueIteration::update_model(arma::Mat<double>* R_new, arma::Mat<double>* P_new)
{
    R = R_new;
    P = P_new;
    V.set_size(P->n_cols,1);
    V.fill(0);
}

void ValueIteration::update(QUpdate update)
{

}

void ValueIteration::calculate_states()
{
    vector<int> num_states;
    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    {
        if(m_resolution.empty()) num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/m_resolution[i]);
        else num_states.push_back(round((m_max_ranges[i] - m_min_ranges[i])/m_resolution[i]));

        num_states[i]++;
    }
    vector<vector<double> > all_values;
    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    {
        vector<double> value;
        for(unsigned int j = 0; j < num_states[i]; j++)
        {
            if(m_resolution.empty()) value.push_back(m_min_ranges[i] + (1.0 * (j)));
            else value.push_back(m_min_ranges[i] + (m_resolution[i] * (j)));
            //cout << value[j] << ",";
        }
        //cout << endl;
        all_values.push_back(value);
    }
    vector<vector<double>::iterator> locations;
    for(unsigned int i = 0; i < num_states.size(); i++)
    {
        locations.push_back(all_values[i].begin());
    }

    bool done = false;
    int it = 0;
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
            cout << s.at(j) << ",";
        }
        cout << endl;
        state_to_ind.insert(make_pair(s, it++));
        //states.push_back(s);
        locations.back()++;
    }
}

void ValueIteration::calculate_solution()
{
    //arma::vec a = arma::nonzeros((*P));
    //cout << "SIZE OF NONZERO: " ;
    //cout << a.size() << endl;
    //openblas_set_num_threads(4);
    //cout << "Armadillo version: " << arma::as_string() << endl;
    double eps = 0.01;
    int it = 0;
    while(true)
    {
        it++;
        //if(it % 10 == 0) cout << it << endl;
        //std::clock_t    start;
        //start = std::clock();
        arma::Mat<double> Q = (*R) + .99 * (*P) * V;
        //std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
        arma::Mat<double> old_V = V;
        for(unsigned int i = 0; i < V.size(); i++)
        {
            double max;
            for(unsigned int j = 0; j < m_possible_actions.size(); j++)
            {
                int ind = i * m_possible_actions.size() + j;
                if(j == 0) max = Q(ind);
                else if(max < Q(ind)) max = Q(ind);
            }
            V(i) = max;
        }
        //V.print();

        //        for(map<QElement::State*, int>::iterator it = state_to_ind.begin(); it != state_to_ind.end(); it++)
        //        {
        //            int ind = it->second;
        //            double max;
        //            for(unsigned int i = 0; i < m_possible_actions.size(); i++)
        //            {
        //                if(i == 0) max = Q(ind,i);
        //                else if(max < Q(ind,i)) max = Q(ind,i);
        //            }
        //            V(ind) = max;
        //        }
        arma::Mat<double> V_diff = V - old_V;
        bool done = true;
        for(arma::mat::iterator it = V_diff.begin(); it != V_diff.end(); it++)
        {
            if(abs(*it) > eps) done = false;
            if(!done) break;
        }
        if(done) break;

    }
    cout << "Iterations: " << it << " ";
    Q_best = (*R) + .99 * (*P) * V;
}

map<QElement::State, int> *ValueIteration::get_state_to_ind()
{
    return &state_to_ind;
}

double ValueIteration::get_Q(int ind, int action)
{
    return Q_best(ind * m_possible_actions.size() + action);
}

int ValueIteration::sub2ind( int row, int col, int cols)
{
    //row++;
    //col++;
    return row*cols+col;// - 1;
}

int ValueIteration::getStateDimensionIndex(double val, double range_min, double range_max, double range_size){
    // Turns a value for the dimension of a state to an index for the range of that dimension
    double percent_of_range = (val-range_min)/(range_max-range_min);
    int index_in_range = round((range_size-1)*percent_of_range);
    //if (index_in_range==int(range_size)) index_in_range = int(range_size)-1; // edge case protection

    return index_in_range;
}

int ValueIteration::get_ind(QElement::State s)
{
    for(unsigned int i = 0; i < s.size(); i++)
    {
        double rem = abs(s[i]/m_resolution[i]);
        double diff = abs(round(rem) - rem);
        if(diff > 0.0001)
        {
            return -1;
        }
    }
    //    vector<int> num_states;
    //    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    //    {
    //        if(m_resolution.empty()) num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/m_resolution[i]);
    //        else num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/m_resolution[i]);

    //        num_states[i]++;
    //    }

    //    vector<int> locs;
    //    for(unsigned int i = 0; i < s.size(); i++)
    //    {
    //        double range = (m_max_ranges[i] - m_min_ranges[i]);
    //        double val = s[i] - m_min_ranges[i];
    //        double val2 = (val/range) * num_states[i];
    //        double rem = fmod(val2, 1);
    //        int loc;
    //        if(abs(val2 - num_states[i]) < 0.0001) val2 = num_states[i] - 1;
    //        loc = val2;
    ////        if(rem >= 0.5) loc = floor(val2 * num_states[i]);
    ////        else loc = round(val2 * num_states[i]);
    //        locs.push_back(loc);
    //    }

    //    int ind = 0;
    //    for(unsigned int i = 0; i < locs.size(); i++)
    //    {
    //        for(unsigned int j = i+1; j < locs.size(); j++)
    //        {
    //            ind += locs[i] * num_states[j];
    //        }
    //    }
    //    ind += *(--locs.end());
    //    if(locs.back() == num_states.back()) ind--;
    //ind--;



    vector<int> num_states(m_min_ranges.size());
    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    {
        num_states[i] = round((m_max_ranges[i] - m_min_ranges[i])/m_resolution[i]) + 1;

    }

    vector<int> locs(s.size());
    for(unsigned int i = 0; i < s.size(); i++)
    {
        locs[i] = getStateDimensionIndex(s[i],m_min_ranges[i],m_max_ranges[i],num_states[i]);
    }

    // add guard for locs.empty?
    int ind = locs[0];
    for (int i=1; i<locs.size(); i++){
        ind = sub2ind(ind,locs[i],num_states[i]);
    }

    for(map<QElement::State, int>::iterator it = state_to_ind.begin(); it != state_to_ind.end(); it++)
    {
        if(it->second == ind)
        {
            for(unsigned int i = 0; i < s.size(); i++)
            {
                //cout << s[i] << ",";
                if(abs(s[i] - it->first.at(i)) > 0.001)
                {
                    exit(1);
                }
            }
            //cout << endl;
            break;
        }
    }
    return ind;
}

