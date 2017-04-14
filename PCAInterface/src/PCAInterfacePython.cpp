#include <PCAInterfacePython.h>

#include <unistd.h>

PCAInterfacePython::PCAInterfacePython()
{
    char cCurrentPath[FILENAME_MAX];
    getcwd(cCurrentPath,sizeof(cCurrentPath));
    strcat(cCurrentPath,"/../../src/Core/PCAInterface");
    PyObject *pmod, *pclass, *pargs, *pinst;
    Py_Initialize(  );


    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyString_FromString(cCurrentPath));
    Py_DECREF(sys);
    Py_DECREF(path);


    pmod   = PyImport_ImportModule("dimensionality_reduction");
    pclass = PyObject_GetAttrString(pmod, "DimReductionClass");

    pargs = Py_BuildValue("()");
    pinst  = PyEval_CallObject(pclass, pargs);

    p_add_data  = PyObject_GetAttrString(pinst, "add_data");
    p_transform_down  = PyObject_GetAttrString(pinst, "transform_down");
    p_build_dim_reduction  = PyObject_GetAttrString(pinst, "build_dim_reduction");
    Py_DECREF(pmod);
    Py_DECREF(pclass);
    Py_DECREF(pargs);
    Py_DECREF(pinst);
}

void PCAInterfacePython::add_data(vector<double> *state)
{
    PyObject *pValue;
    PyObject *l = PyList_New(state->size());
    for(size_t i=0; i< state->size(); i++){
        pValue = PyFloat_FromDouble(state->at(i));
        PyList_SetItem(l, i, pValue);
    }
    PyObject *arglist = Py_BuildValue("(O)", l);
    PyObject *pState = PyObject_CallObject(p_add_data, arglist);
}

vector<double> PCAInterfacePython::transform_down(vector<double>* state, int dim)
{
    PyObject *pValue;
    PyObject *l = PyList_New(state->size());
    for(size_t i=0; i<state->size(); i++){
        pValue = PyFloat_FromDouble(state->at(i));
        PyList_SetItem(l, i, pValue);
    }
    PyObject *arglist = Py_BuildValue("(Oi)", l, dim);
    PyObject *pState = PyObject_CallObject(p_transform_down, arglist);

    Py_DECREF(pValue);
    vector<double> return_val;
    if (PyList_Check(pState)) {
        for(Py_ssize_t i = 0; i < PyList_Size(pState); i++) {
            pValue = PyList_GetItem(pState, i);
            return_val.push_back( PyFloat_AsDouble(pValue) );
        }
    }

    Py_DECREF(l);
    Py_DECREF(arglist);
    Py_DECREF(pState);
    return return_val;

}

void PCAInterfacePython::build_dim_reduction(int dim_i, int dim_max)
{
    PyObject *arglist = Py_BuildValue("(ii)", dim_i, dim_max);
    PyObject *pState = PyObject_CallObject(p_build_dim_reduction, arglist);
}

PCAInterfacePython::~PCAInterfacePython()
{

}
