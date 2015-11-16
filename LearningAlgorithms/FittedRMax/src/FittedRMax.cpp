#include "FittedRMax.h"
#include <cmath>
FittedRMax::FittedRMax(FittedRMaxArgs *args) : ILearningAlgorithm(args)
{
    m_resolution = args->resolution;
    ValueIterationArgs* VI_args = new ValueIterationArgs();
    VI_args->resolution = args->resolution;
    R_max = 0;
    VI = new ValueIteration(VI_args);
}

void FittedRMax::init()
{
    VI->calculate_states();

    state_to_ind = VI->get_state_to_ind();
    cout << state_to_ind->size() << endl;
    R_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),1);
    R_estimate.fill(0);
    //D_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),state_to_ind->size());
    //D_estimate.fill(0);
    N_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),1);
    N_estimate.fill(0);
    P_estimate.set_size(state_to_ind->size() * m_possible_actions.size(),state_to_ind->size());
    P_estimate.fill(0);
    VI->update_model(&R_estimate, &P_estimate);
    //    VI->calculate_solution();
}

pair<QElement::Action, double> FittedRMax::get_action(QElement::State s)
{
    state_to_ind = VI->get_state_to_ind();

    if(updated_states.size() != 0)
    {
        //R_temp = R_estimate;
        //P_temp = P_estimate;
        for(set<int>::iterator it = updated_states.begin(); it != updated_states.end(); it++)
        {
            int cur_ind = *it;
            for(unsigned int a = 0; a < m_possible_actions.size(); a++)
            {
                int cur_sa = cur_ind * m_possible_actions.size() + a;
                R_temp[cur_sa] = R_estimate[cur_sa];
                if(N_estimate[cur_sa] < 1)
                {
                    R_temp[cur_sa] = R_max;

                    for(map<QElement::State, int>::iterator it2 = state_to_ind->begin(); it2 != state_to_ind->end(); it2++)
                    {
                        int cur_ind2 = it2->second;
                        P_temp(cur_sa, cur_ind2) = 0;
                    }
                    P_temp(cur_sa, cur_ind) = 1;
                }
                else
                {
                    for(map<QElement::State, int>::iterator it2 = state_to_ind->begin(); it2 != state_to_ind->end(); it2++)
                    {
                        int cur_ind2 = it2->second;
                        P_temp(cur_sa, cur_ind2) = P_estimate(cur_sa, cur_ind2);
                    }
                }
            }
        }
        //cout << "Num Updates:" << updated_states.size() << endl;
        updated_states.clear();
    }
    else
    {
        R_temp = R_estimate;
        P_temp = P_estimate;
        for(map<QElement::State, int>::iterator it = state_to_ind->begin(); it != state_to_ind->end(); it++)
        {
            int cur_ind = it->second;
            for(unsigned int a = 0; a < m_possible_actions.size(); a++)
            {
                int cur_sa = cur_ind * m_possible_actions.size() + a;
                if(N_estimate[cur_sa] < 1)
                {
                    R_temp[cur_sa] = R_max;

                    for(map<QElement::State, int>::iterator it2 = state_to_ind->begin(); it2 != state_to_ind->end(); it2++)
                    {
                        int cur_ind2 = it2->second;
                        P_temp(cur_sa, cur_ind2) = 0;
                    }
                    P_temp(cur_sa, cur_ind) = 1;
                }
            }
        }
    }
//    static int it = 0;
//    it++;

    VI->update_model(&R_temp, &P_temp);
    std::clock_t    start;
    start = std::clock();
    VI->calculate_solution();
    std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;


//    if(it % 1000 == 0 && it != 0)
//    {
//        R_temp.print();
//        P_temp.print();
//        cout << "-----" << endl;
//        D_estimate.print();
//        N_estimate.print();
//        VI->Q_best.print();
//        cout << "--------" << endl;
//        VI->V.print();
//        exit(1);
//    }



    //Approximate Q_Best.
    map<int, double> coeffs = calculate_mlinterp(s);
    vector<double> approx_Q;
    for(unsigned int a = 0; a < m_possible_actions.size(); a++)
    {
        double q_val = 0;
        for(map<int,double>::iterator it = coeffs.begin(); it != coeffs.end(); it++)
        {
            int ind = it->first;
            double val = it->second;
            q_val += VI->get_Q(ind, a) * val;
        }
        approx_Q.push_back(q_val);
    }

    double max;
    int max_ind;
    for(unsigned int i = 0; i < m_possible_actions.size(); i++)
    {
        if(i == 0)
        {
            max = approx_Q[i];
            max_ind = i;
        }
        else if(max < approx_Q[i])
        {
            max = approx_Q[i];
            max_ind = i;
        }
    }
    if(std::isnan(max))
    {
        R_temp.print();
        P_temp.print();
        //cout << "-----" << endl;
        D_estimate.print();
        N_estimate.print();

        for(map<int,double>::iterator it = coeffs.begin(); it != coeffs.end(); it++)
        {
            cout << it->first << "," << it->second << endl;
            for(unsigned int i = 0; i < m_possible_actions.size(); i++)
            {
                cout << VI->get_Q(it->first, i) << endl;
            }
            VI->calculate_solution();
        }

        exit(1);
    }
    //cout << "Action:Value : " << max_ind << ":" << max << endl;
    pair<QElement::Action, double> action_value = make_pair(max_ind, max);
    //pair<QElement::Action, double> action_value = VI->get_action(s);
    return action_value;
}

void FittedRMax::set_possible_actions(vector<int> possible_actions)
{
    ILearningAlgorithm::set_possible_actions(possible_actions);
    VI->set_possible_actions(possible_actions);
}

void FittedRMax::set_ranges(vector<double> min_ranges, vector<double> max_ranges)
{
    ILearningAlgorithm::set_ranges(min_ranges, max_ranges);
    VI->set_ranges(min_ranges, max_ranges);
}

void FittedRMax::set_resolution(vector<double> resolution)
{
    ILearningAlgorithm::set_resolution(resolution);
    VI->set_resolution(resolution);
}

void FittedRMax::update(QUpdate update)
{
    state_to_ind = VI->get_state_to_ind();
    static int it = 0;
    it++;
    if(update.reward == 0)
    {
        cout << "HI" << endl;
    }
    //cout << "Cur It: " << it << endl;
    //cout << "Cur State: " << update.state[0] << "," << update.state[1] << endl;//"," << update.state[2] << "," << update.state[3] << endl;
    //cout << "Cur Action: " << update.action << endl;
    for(map<QElement::State,int>::iterator state_it = state_to_ind->begin(); state_it != state_to_ind->end(); state_it++)
    {
        int cur_ind_update = state_it->second;
        QElement::State cur_state_update = state_it->first;
        //cout << "Cur Ind: " << cur_ind_update << endl;
        for(unsigned int a_update = 0; a_update < m_possible_actions.size(); a_update++)
        {
            int cur_sa_update = cur_ind_update * m_possible_actions.size() + a_update;
            double weight = calculate_dist(update.state, update.action, cur_state_update, a_update);
            if(weight < .001) continue;
            double alpha;
            if(weight < .00001 && N_estimate[cur_sa_update] == 0) continue;
            else alpha = weight/(N_estimate[cur_sa_update] + weight);
            //cout << "Alpha:" << alpha << endl;
            N_estimate[cur_sa_update] += weight;
            R_estimate[cur_sa_update] = (1 - alpha) * R_estimate[cur_sa_update] + alpha * update.reward;
            updated_states.insert(cur_ind_update);
            QElement::State action_effect = action_effect_function(update.state, update.next_state, cur_state_update);
            //action_effect = update.next_state;
            for(unsigned int af = 0; af < action_effect.size(); af++)
            {
                if(action_effect[af] > m_max_ranges[af] || action_effect[af] < m_min_ranges[af])
                {
                    action_effect = update.next_state;
                    break;
                }
            }
            //cout << "Action Effect: " << action_effect[0] << "," << action_effect[1] << endl;

            vector<QElement::State> coeffs_states;
            map<int, double> coeffs = calculate_mlinterp(action_effect);

            double sum = 0;

//            for(vector<QElement::State>::iterator it = coeffs_states.begin(); it != coeffs_states.end(); it++)
//            {
//                double weight2 = calculate_dist(*it, 1, action_effect, 1);
//                cout << weight2 << endl;
//            }
            for(map<int, double>::iterator it = coeffs.begin(); it != coeffs.end(); it++)
            {
                int ind = it->first;
                double coeff = it->second;
                sum += coeff;
                //cout << ind << "," << coeff << endl;
                if(coeff > 1) exit(1);
                int cur_sa = cur_ind_update * m_possible_actions.size() + update.action;
                if(coeff < 0.001) continue;
                P_estimate(cur_sa, ind) = (1 - alpha) * P_estimate(cur_sa, ind) + alpha * coeff;
            }
            if(abs(sum - 1) > 0.0001)
            {
                cout << "MLINTERP BROKE!!!" << endl;
                cout << sum << endl;
                cout << "Action Effect: " << endl;
                for(unsigned int i = 0; i < action_effect.size(); i++)
                {
                    cout << action_effect[i] << endl;
                }
                exit(1);
            }
            //P_estimate.print();
            //cout << endl;
        }
    }

    //VI->update_model(R_estimate, P_estimate);alpha = 0

    //VI->calculate_solution();

}

double FittedRMax::calculate_dist(QElement::State s1, QElement::Action a1, QElement::State s2, QElement::Action a2)
{
    for(unsigned int i = 0; i < s1.size(); i++)
    {
        s1[i] = (s1[i] - m_min_ranges[i])/(m_max_ranges[i] - m_min_ranges[i]);
        s2[i] = (s2[i] - m_min_ranges[i])/(m_max_ranges[i] - m_min_ranges[i]);
    }
    if(a1 != a2) return 0;
    double dist = 0;
    for(unsigned int i = 0; i < s1.size(); i++)
    {
        dist += pow(s1[i] - s2[i],2);
    }
    dist = sqrt(dist)/0.08;
    dist = -pow(dist,2);
    dist = exp(dist);
    if(dist < 0.01) dist = 0;
    return dist;
}

//s1 is the estimated variable.
map<int, double> FittedRMax::calculate_mlinterp(QElement::State s1, vector<QElement::State> *states)
{
//            s1.clear();
//            s1.push_back(0.156919);
//            s1.push_back(-0.43988);
//            s1.push_back(0.0350067);
//            s1.push_back(0.0341475);
//            m_resolution.clear();
    //        m_resolution.push_back(2);
    //        m_resolution.push_back(2);
    //Find Corners
    vector<QElement::State > all_corners;

    state_to_ind = VI->get_state_to_ind();
    map<int, double> coeffs;
    for(map<QElement::State ,int>::iterator it = state_to_ind->begin(); it != state_to_ind->end(); it++)
    {
        coeffs.insert(make_pair(it->second, 0));
    }

    //map<QElement::State ,int>::iterator it = state_to_ind->find(s1);
    int ind = VI->get_ind(s1);
    if(ind != -1)
    //if(it != state_to_ind->end())
    {
        //cout << "Skipping" << endl;
        coeffs[ind] = 1;
        return coeffs;
    }

    vector<vector<double> > all_values;
    vector<int> num_states;

    for(unsigned int i = 0; i < s1.size(); i++)
    {
        double rem = s1[i]/m_resolution[i];
        double closest_low;
        double closest_high;
        double temp = floor(rem);
        if(abs(round(rem) - rem) < 0.0000000001)
        {
            closest_low = 0;
            closest_high = m_resolution[i];
        }
        else
        {
            if(s1[i] < 0)
            {
                int rem_round = int(rem);
                closest_high = rem_round * m_resolution[i];
                closest_low = closest_high - m_resolution[i];
                closest_low = abs(s1[i] - closest_low);
                closest_high = abs(s1[i] - closest_high);
            }
            else
            {
                int rem_round = int(rem);
                closest_low = rem_round * m_resolution[i];
                closest_high = closest_low + m_resolution[i];
                closest_low = abs(s1[i] - closest_low);
                closest_high = abs(s1[i] - closest_high);
            }

        }


        vector<double> value;
        value.push_back(-closest_low);
        value.push_back(closest_high);
        //cout << value[0] << "," << value[1] << endl;
        all_values.push_back(value);
        num_states.push_back(2);
    }

    //    for(unsigned int i = 0; i < m_resolution.size(); i++)
    //    {
    //        //cout << s1[i] << ",";
    //        vector<double> value;
    //        value.push_back(m_resolution[i]);
    //        value.push_back(-m_resolution[i]);
    //        all_values.push_back(value);
    //        num_states.push_back(2);
    //    }
    //cout << endl;
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
            s.push_back(s1[j] + *locations[j]);
        }
        locations.back()++;

        bool good = true;
        for(unsigned int j = 0; j < s.size(); j++)
        {
            if(s[j] < m_min_ranges[j] || s[j] > m_max_ranges[j])
            {
                if(abs(s[j] - m_min_ranges[j]) < 0.0001) continue;
                if(abs(s[j] - m_max_ranges[j]) < 0.0001) continue;
                good = false;
            }
        }
        if(good)
        {
            all_corners.push_back(s);

        }
    }
    int expected_num_corners = pow(2,m_resolution.size());



    //Find Diagnols
    vector<pair<QElement::State, QElement::State> > corner_to_diag;
    for(unsigned int i = 0; i < all_corners.size(); i++)
    {
        QElement::State cur_corner = all_corners[i];
        double val = 1;
        QElement::State diagnol;
        for(unsigned int j = 0; j < cur_corner.size(); j++)
        {
            if(cur_corner[j] > s1[j]) diagnol.push_back(cur_corner[j] - m_resolution[j]);
            if(cur_corner[j] < s1[j]) diagnol.push_back(cur_corner[j] + m_resolution[j]);
            if(cur_corner[j] == s1[j])
            {
                if(cur_corner[j] + m_resolution[j] <= m_max_ranges[j]) diagnol.push_back(cur_corner[j] + m_resolution[j]);
                else diagnol.push_back(cur_corner[j] - m_resolution[j]);
            }
        }

        if(all_corners.size() != expected_num_corners)
        {
            vector<QElement::State >::iterator it = find(all_corners.begin(), all_corners.end(), diagnol);
            if(it == all_corners.end()) corner_to_diag.push_back(make_pair(diagnol, cur_corner));
        }
        corner_to_diag.push_back(make_pair(cur_corner, diagnol));
        //        for(unsigned int j = 0; j < cur_corner.size(); j++)
        //        {
        //            double temp = abs(diagnol[j] - s1[j]);
        //            if(temp == 0) temp = 1;
        //            val *= temp;
        //        }
        //        val = val/normal;
        //        int ind = state_to_ind->find(cur_corner)->second;
        //        cout << val << endl;
        //        coeffs.insert(make_pair(ind, val));
    }

    //Find max and min corners.
    vector<double> max_corners = s1;
    vector<double> min_corners = s1;
    for(unsigned int i = 0; i < corner_to_diag.size(); i++)
    {
        QElement::State cur_corner = corner_to_diag[i].first;
        for(unsigned int j = 0; j < cur_corner.size(); j++)
        {
            //cout << cur_corner[j] << ",";
            if(cur_corner[j] > max_corners[j]) max_corners[j] = cur_corner[j];
            if(cur_corner[j] < min_corners[j]) min_corners[j] = cur_corner[j];
        }
        //cout << endl;
    }
    //Find the normalization term
    double normal = 1;
    for(unsigned int i = 0; i < max_corners.size(); i++)
    {
        normal *= (max_corners[i] - min_corners[i]);
    }
    //cout << normal << endl;

    for(unsigned int i = 0; i < corner_to_diag.size(); i++)
    {
        pair<QElement::State, QElement::State> cur_corner_to_diag = corner_to_diag[i];
        // cout << "Corner:" << cur_corner_to_diag.first[0] << "," << cur_corner_to_diag.first[1] << endl;
        //cout << "Diag:" << cur_corner_to_diag.second[0] << "," << cur_corner_to_diag.second[1] << endl;
    }
    for(unsigned int i = 0; i < corner_to_diag.size(); i++)
    {
        pair<QElement::State, QElement::State> cur_corner_to_diag = corner_to_diag[i];
        double val = 1;
        for(unsigned int j = 0; j < cur_corner_to_diag.first.size(); j++)
        {
            double temp = abs(cur_corner_to_diag.second[j] - s1[j]);
            //if(temp == 0) temp = 1;
            val *= temp;
            //cout << cur_corner_to_diag.first[j] << ",";
        }
        //cout << endl;
        val = val/normal;
        int ind = VI->get_ind(cur_corner_to_diag.first);
        if(ind == -1) continue;
        //cout << val << endl;
        //if(val != 0) coeffs.insert(make_pair(ind, val));
        //coeffs.insert(make_pair(ind, val));
        coeffs[ind] = val;
        if(states != 0) states->push_back(cur_corner_to_diag.first);
    }
    //exit(1);




    //temp.insert(make_pair(ind, 1));
    return coeffs;

}


QElement::State FittedRMax::action_effect_function(QElement::State s1, QElement::State s2, QElement::State s3)
{
    //temp = s2 - s1
    QElement::State temp;
    for(unsigned int i = 0; i < s1.size(); i++)
    {
        temp.push_back(s2[i] - s1[i]);
    }

    //action_effect = s3 + temp
    QElement::State action_effect;
    for(unsigned int i = 0; i < s3.size(); i++)
    {
        action_effect.push_back(s3[i] + temp[i]);
    }

    return action_effect;

}
