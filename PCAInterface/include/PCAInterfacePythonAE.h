#include <Python.h>
#include <vector>
#include<iostream>

#include "autoencoder_api_cdef_api.h"

using namespace std;

class PCAInterfacePythonAE
{
public:
    PCAInterfacePythonAE();

    void add_data(vector<double>* state);
    vector<double> transform_down(vector<double>* state, int dim);
    void build_dim_reduction(int dim_max);
    virtual ~PCAInterfacePythonAE();

private:
    PyObject* p_transform_down;
    PyObject* p_add_data;
    PyObject* p_build_dim_reduction;

    NNDimReduce* nn_dim_reduce;


};


