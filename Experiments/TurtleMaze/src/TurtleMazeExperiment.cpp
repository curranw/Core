#include <TurtleMazeExperiment.h>

TurtleMazeExperiment::TurtleMazeExperiment(TurtleMaze* domain, QTiles* learning_algorithm, TurtleMazeExperimentArgs* exp_args)
    : IExperiment(domain, learning_algorithm, exp_args)
{
    m_learning_algorithm = learning_algorithm;
    m_domain = domain;
    m_exp_args = exp_args;
    if(m_exp_args->demonstrations) domain->m_accumulate_data = true;
}

void TurtleMazeExperiment::init()
{
    IExperiment::init();
    //m_learning_algorithm->read("TurtleMazeQTable");


    //m_learning_algorithm->output("TurtleMazeQTable");



}

void TurtleMazeExperiment::step()
{
    cur_step++;
    vector<double> state = m_domain->get_state();
    pair<QElement::Action, double> action_value = m_learning_algorithm->get_action(state);

    m_domain->step(action_value.first);

    vector<double> next_state = m_domain->get_state();
    double reward = m_domain->get_reward();

    tot_reward += reward;
    for(map<vector<double>, pair<double, int> >::iterator it = demo_updates.begin(); it != demo_updates.end(); )
    {
        vector<double> temp_state = it->first;
        bool same = true;
        for(int i = 0; i < temp_state.size(); i++)
        {
            if(abs(temp_state[i] - state[i]) > 0.25)
            {
                same = false;
            }
            if(!same) break;
        }
        if(same)
        {
            num_demo_used++;
            reward += it->second.first;
            it->second.second--;
            if(it->second.second == 0)
            {
                demo_updates.erase(it++);
            }
            else
            {
                ++it;
                break;
            }
            break;
        }
        ++it;
    }


    bool bad = false;
    for(int i = 0; i < state.size(); i++)
    {
        if (state[i] == 0 && next_state[i] == 0)
        {
            bad = true;
        }
        else
        {
            bad = false;
            break;
        }
    }
    if(bad)
    {
        cout << "BREAK HERE" << endl;
        cout << state[0] << "," << state[1] << "," << state[2] << endl;
        cout << next_state[0] << "," << next_state[1] << "," << next_state[2] << endl;
        cout << action_value.second << endl;
        cout << reward << endl;
        cout << endl;
        exit(1);
    }


    update_params.reward = reward;
    update_params.state = state;
    update_params.next_state = next_state;
    update_params.action = action_value.first;
    update_params.old_value = action_value.second;

    m_learning_algorithm->update(update_params);


    if(m_domain->m_accumulate_data) m_accumulated_data.push_back(state);
    if(m_domain->m_accumulate_data) m_accumulated_rewards.push_back(reward);
}

void TurtleMazeExperiment::end_epoch()
{
    cout << tot_reward << endl;
    typedef double carrie;
    cout << num_demo_used << endl;
    IExperiment::end_epoch();
    performance.push_back(m_domain->get_performance());
//    if(num_demo_used == 0 && demo_updates.size() != 0)
//    {
//        finish_learning = true;
//        if(!finish_learning) output_results();
//    }
//    else if(current_iteration != m_exp_args->num_epochs) output_results();

//    if(current_iteration % 10 == 0 || (num_demo_used == 0 && demo_updates.size() != 0))
//    {
//        QTiles* temp_learning_alg = (QTiles*)m_learning_algorithm;
//        vector<carrie> max_ranges = m_domain->get_max_ranges();
//        vector<carrie> min_ranges = m_domain->get_min_ranges();
//        carrie max_x = max_ranges[0];
//        carrie max_y = max_ranges[1];
//        carrie max_theta = max_ranges[2];
//        carrie min_x = min_ranges[0];
//        carrie min_y = min_ranges[1];
//        carrie min_theta = min_ranges[2];
//        vector<vector<carrie> > output;
//        vector<vector<carrie> > accumulation;
//        for(double x = min_x; x < max_x; x += 0.5)
//        {
//            vector<double> single_accumulation;
//            for(double y = min_y; y < max_y; y += 0.5)
//            {
//                double sum = 0;
//                for(double theta = min_theta; theta < max_theta; theta+= 0.314159)
//                {
//                    vector<double> state;
//                    state.push_back(x);
//                    state.push_back(y);
//                    state.push_back(theta);
//                    vector<double> action_values = temp_learning_alg->get_action_values(state);
//                    int max_action = 0;
//                    double max_value = action_values[0];
//                    for(unsigned int a = 1; a < action_values.size(); a++)
//                    {
//                        if(action_values[a] > max_value)
//                        {
//                            max_action = a;
//                            max_value = action_values[a];
//                        }
//                    }
//                    state.push_back(max_action);
//                    state.push_back(max_value);
//                    output.push_back(state);
//                    sum += action_values[0];
//                    sum += action_values[1];
//                    sum += action_values[2];
//                }
//                single_accumulation.push_back(sum);
//            }
//            accumulation.push_back(single_accumulation);
//        }
//        //utils::to_csv(&output, "data_visualization");
//    }
//    num_demo_used = 0;
    //utils::to_csv(&accumulation, "accumulation");

    if(current_iteration == 10)
    {
        load_updates("export_file_1.txt");
    }
    if(current_iteration == 50)
    {
        load_updates("export_file_1.txt");
        load_updates("export_file_2.txt");
    }
    if(current_iteration == 100)
    {
        load_updates("export_file_1.txt");
        load_updates("export_file_2.txt");
        load_updates("export_file_3.txt");
    }
}

void TurtleMazeExperiment::output_results()
{
      utils::to_csv<double>(&m_rewards, m_exp_args->save_file);
    m_learning_algorithm->output("TurtleMazeQTable");

//    vector<vector<double> > output_data;
//    for(map<vector<double>, pair<double, int> >::iterator it = demo_updates.begin(); it != demo_updates.end(); it++)
//    {
//        output_data.push_back(it->first);
//        vector<double> reward;
//        reward.push_back(it->second.first);
//        output_data.push_back(reward);
//        vector<double> used;
//        used.push_back(it->second.second);
//        output_data.push_back(used);

//    }
//    utils::to_csv_overwrite(&output_data, "demo_data_used.csv");

    utils::to_csv(&performance, m_exp_args->save_file + "_performance");
}

void TurtleMazeExperiment::load_updates(string update_file)
{
    QTiles* temp_learning_alg = (QTiles*)m_learning_algorithm;
    //Parse turtlebot data

//    vector<vector<double> > old_demo_data = utils::read_csv("demo_data_used.csv");
//    for(unsigned int i = 0; i < old_demo_data.size();)
//    {
//        vector<double> state = old_demo_data[i++];
//        vector<double> reward = old_demo_data[i++];
//        vector<double> used = old_demo_data[i++];
//        demo_updates.insert(make_pair(state, make_pair(reward[0], used[0])));
//    }

    ifstream data_file;

    data_file.open (update_file);
    vector<pair<double, map<string, double> > > data;
    string key, line;
    double value, time;
    char c;
    while(getline (data_file,line))
    {
        map<string, double> data_map;
        stringstream ss(line);
        ss >> time;
        ss >> c;
        if (ss >> c && c == '{')
            while (ss >> c && std::getline(ss, key, '\''))
            {
                if (ss >> c && c == ':' && ss >> value)
                    data_map[key] = value;
                if (ss >> c && c != ',')
                    break;
            }
        data.push_back(make_pair(time, data_map));
        ss.str("");
    }

    //Create state->action connections. Use next-state as same state.
    vector<pair<vector<double>, int> > state_action_data;
    vector<pair<double, QUpdate> > updates;
    for(vector<pair<double, map<string, double> > >::iterator data_it = data.begin(); data_it != data.end(); data_it++)
    {
        time = data_it->first;
        vector<double> state;
        map<string, double> data = data_it->second;

        if(data.find("/ground_truth/pose/pose/position/x") == data.end()) continue;
        double x_loc = data.at("/ground_truth/pose/pose/position/x");
        double y_loc = data.at("/ground_truth/pose/pose/position/y");

        double x_q = data.at("/ground_truth/pose/pose/orientation/x");
        double y_q = data.at("/ground_truth/pose/pose/orientation/y");
        double z_q = data.at("/ground_truth/pose/pose/orientation/z");
        double w_q = data.at("/ground_truth/pose/pose/orientation/w");

        double reward = data.at("Reward");
        //            if(reward > 1) reward = 1;
        //            if(reward < -1) reward = -1;
        tf::Quaternion q(x_q, y_q, z_q, w_q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        state.push_back(x_loc);
        state.push_back(y_loc);
        state.push_back(yaw);

        for(unsigned int i = 0; i <3; i++)
        {
            QUpdate update;
            update.reward = reward;
            update.state = state;
            update.action = i;
            updates.push_back(make_pair(time, update));
        }

        int i = 0;
        for(vector<pair<double, QUpdate> >::reverse_iterator it = updates.rbegin(); i != 3; it++)
        {
            if(updates.empty()) break;
            if(abs(time - it->first) >= 1)
            {
                it->second.next_state = it->second.state;
            }
            else it->second.next_state = state;
            i++;
        }
        //demo_updates.insert(make_pair(state, make_pair(reward, 10)));
    }

    for(unsigned int i = 0; i < updates.size(); i++)
    {
        QUpdate* update = &updates[i].second;
        vector<double> action_values = temp_learning_alg->get_action_values(update->state);
        update->old_value = action_values[update->action];
        m_learning_algorithm->update(*update);
    }
}

TurtleMazeExperiment::~TurtleMazeExperiment()
{

}




