#include "utils.h"

utils::utils()
{
}

vector<double> utils::moving_average(vector<double>* data, int bin_size)
{
    vector<double> moving_average;
    if(bin_size == 0) return moving_average;
    double tot = 0;
    for(unsigned int i = 0; i < data->size(); i++)
    {
        tot += data->at(i);
        if(i == 0)
        {
            moving_average.push_back(tot);
        }
        if(i == data->size()-1)
        {
            moving_average.push_back(tot/(i % bin_size));
        }
        if(i % bin_size == 0 && i != 0 && i != data->size()-1)
        {
            moving_average.push_back(tot/bin_size);
            tot = 0;
        }
    }
    return moving_average;
}

void utils::to_csv(vector<double>* data, string name)
{
    name = fix_filename(name);
    ofstream file (name);
    for(unsigned int i = 0; i < data->size(); i++)
    {
        if(i != data->size()-1) file << data->at(i) << ",";
        else file << data->at(i) << endl;
    }
    file.close();

}

void utils::to_csv(vector<vector<double> >* data, string name)
{
    name = fix_filename(name);
    ofstream file (name);
    for(unsigned int i = 0; i < data->size(); i++)
    {
        for(unsigned int j = 0; j < data->at(i).size(); j++)
        {
            if(j != data->at(i).size()-1) file << data->at(i).at(j) << ",";
            else file << data->at(i).at(j) << endl;
        }
    }
    file.close();
}


vector<vector<double> > utils::read_csv(string name)
{
    ifstream file (name);
    std::string line;
    vector<vector<double> > csv_vals;
    while(std::getline(file, line))
    {
        vector<double> csv_val;
        std::istringstream s(line);
        std::string field;

        while (getline(s, field,','))
        {
            double val = stod(field);
            csv_val.push_back(val);
        }
        csv_vals.push_back(csv_val);
    }
    file.close();
    return csv_vals;
}

string utils::fix_filename(string name)
{
    int it = 0;
    while(true)
    {
        cout << name << endl;
        it++;
        ifstream test_exists((name + ".csv").c_str());
        if(test_exists.good())
        {
            if(it == 1)
            {
                name = name + "_1";
            }
            else
            {
                if(it < 11) name = name.substr(0, name.size()-2);
                if(it >= 11) name = name.substr(0, name.size()-3);
                name = name + "_" + to_string(it);
            }
        }
        else
        {
            break;
        }
    }
    name = name + ".csv";
    return name;
}
