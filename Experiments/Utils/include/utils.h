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
    static void to_csv(vector<double>* data, string name);
    static void to_csv(vector<int>* data, string name);
    static void to_csv(vector<vector<double> >* data, string name);
    static void to_csv_overwrite(vector<vector<double> > *data, string name);
    static vector<vector<double> > read_csv(string name);
    static vector<vector<double> > read_newest_csv(string name);
private:
    static string fix_filename(string name);
    static string newest_filename(string name);
};

#endif // UTILS_H
