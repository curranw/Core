#include <Python.h>
#include <vector>
#include<iostream>

using namespace std;
class PCAInterfacePython
{
public:
    PCAInterfacePython();

    void add_data(vector<double>* state);
    vector<double> transform_down(vector<double>* state, int dim);
    void build_dim_reduction(int dim_i, int dim_max);
    virtual ~PCAInterfacePython();

private:
    PyObject* p_transform_down;
    PyObject* p_add_data;
    PyObject* p_build_dim_reduction;
};
