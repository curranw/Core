#include <ValueIteration.h>

ValueIteration::ValueIteration(ILearningArguments *args) : ILearningAlgorithm(args)
{

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

void ValueIteration::update_model(arma::Mat<double> R_new, arma::Mat<double> P_new)
{
    R = R_new;
    P = P_new;
    V.set_size(P.n_cols,1);
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
        num_states.push_back((m_max_ranges[i] - m_min_ranges[i])/1.0);
        num_states[i]++;
    }
    vector<vector<double> > all_values;
    for(unsigned int i = 0; i < m_min_ranges.size(); i++)
    {
        vector<double> value;
        for(unsigned int j = 0; j < num_states[i]; j++)
        {
            value.push_back(m_min_ranges[i] + (1.0 * (j)));
            cout << value[j] << ",";
        }
        cout << endl;
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
        locations.back()++;
    }
}

void ValueIteration::calculate_solution()
{
    double eps = 0.001;
    while(true)
    {
        cout << "Start" << endl;
        arma::Mat<double> Q = R + .99 * P * V;
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
    Q_best = R + .99 * P * V;
}

