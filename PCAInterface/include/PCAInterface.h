#pragma once

#include <pca.h>
#include <vector>


using namespace std;
class PCAInterface
{
private:


public:
    PCAInterface(int num_variables);
    virtual ~PCAInterface();
    stats::pca* pca;
    void solve();
    void add_data(vector<double> *data);
    void add_data(vector<vector<double> > *data);
    void set_projection_dimension(int projection_dimension);
    vector<double> transform_down(vector<double> *data);
    vector<double> transform_up(vector<double> *data);
    vector<double> get_error();
    void normalize(vector<double> *data);
    double get_neighbor_distance();

    vector<double> min;
    vector<double> max;
    vector<vector<double> > saved_data;
};
