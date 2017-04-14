#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>


using namespace std;
class utils
{
public:
    utils();
    static vector<double> moving_average(vector<double>* data, int bin_size);

    template <typename T>
    static void to_csv(vector<T>* data, string name);

//    static void to_csv2d(vector<int>* data, string name);

    template<typename T>
    static void to_csv2d(vector<vector<T> >* data, string name);
    static void to_csv_overwrite(vector<vector<double> > *data, string name);

    template <typename T>
    static vector<vector<T> > read_csv(string name);
//    static vector<vector<unsigned int> > read_csv(string name);

    template <typename T>
    static vector<vector<T> > read_newest_csv(string name);

private:
    static string fix_filename(string name);
    static string newest_filename(string name);
};


template<typename T>
vector<vector<T> > utils::read_csv(string name)
{
    ifstream file (name);
    std::string line;
    vector<vector<T> > csv_vals;
    while(std::getline(file, line))
    {
        vector<T> csv_val;
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

template <typename T>
vector<vector<T> > utils::read_newest_csv(string name)
{
    string newest_name = newest_filename(name);
    ifstream test_exists((newest_name).c_str());
    if(!test_exists.good()) return vector<vector<T> >();
    return utils::read_csv<T>(newest_name);
}

template <typename T>
void utils::to_csv(vector<T>* data, string name)
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

template <typename T>
void utils::to_csv2d(vector<vector<T> >* data, string name)
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

#endif // UTILS_H
