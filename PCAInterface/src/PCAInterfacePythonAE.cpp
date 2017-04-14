#include <PCAInterfacePythonAE.h>

#include <unistd.h>

PCAInterfacePythonAE::PCAInterfacePythonAE()
{
    Py_Initialize(  );
    char cCurrentPath[FILENAME_MAX];
    getcwd(cCurrentPath,sizeof(cCurrentPath));
    strcat(cCurrentPath,"/../../src/Core/PCAInterface");
    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyString_FromString(cCurrentPath));
    Py_DECREF(sys);
    Py_DECREF(path);


    import_autoencoder_api_cdef();
    //initautoencoder_api_cdef();
    //PyInit_autoencoder_api_cdef();
    nn_dim_reduce = buildNNDimReduce();
    cout << "HI" << endl;
    //    PyObject *arglist_1 = Py_BuildValue("");
    //    PyObject *arglist_2 = Py_BuildValue("__cinit__");
    //    int blah = __pyx_pw_20autoencoder_api_cdef_11NNDimReduce_1__cinit__((PyObject*)a,arglist_1, arglist_2);
    //a = &blah;
    //__pyx_pf_20autoencoder_api_cdef_11NNDimReduce___init__(a);



    //    Py_Initialize(  );





    //    pmod   = PyImport_ImportModule("autoencoder_api");
    //    pclass = PyObject_GetAttrString(pmod, "NNDimReduce");

    //    pargs = Py_BuildValue("()");
    //    pinst  = PyEval_CallObject(pclass, pargs);

    //    p_add_data  = PyObject_GetAttrString(pinst, "add_data");
    //    p_transform_down  = PyObject_GetAttrString(pinst, "transform_down");
    //    p_build_dim_reduction  = PyObject_GetAttrString(pinst, "build_dim_reduction");
    //    Py_DECREF(pmod);
    //    Py_DECREF(pclass);
    //    Py_DECREF(pargs);
    //    Py_DECREF(pinst);
}

void PCAInterfacePythonAE::add_data(vector<double> *state)
{
//    PyObject *pValue;
//    PyObject *l = PyList_New(state->size());
//    for(size_t i=0; i< state->size(); i++){
//        pValue = PyFloat_FromDouble(state->at(i));
//        PyList_SetItem(l, i, pValue);
//    }
//    PyObject *arglist = Py_BuildValue("(O)", l);

    accessor_add_data(nn_dim_reduce, *state);
    //    __pyx_vtabptr_20autoencoder_api_cdef_NNDimReduce->add_data(a,arglist);
    //    PyObject *pState = PyObject_CallObject(p_add_data, arglist);
}

vector<double> PCAInterfacePythonAE::transform_down(vector<double>* state, int dim)
{
//    PyObject *pValue;
//    PyObject *l = PyList_New(state->size());
//    for(size_t i=0; i<state->size(); i++){
//        pValue = PyFloat_FromDouble(state->at(i));
//        PyList_SetItem(l, i, pValue);
//    }
//    PyObject *arglist = Py_BuildValue("(Oi)", l, dim);
//    PyObject *pState = PyObject_CallObject(p_transform_down, arglist);

//    Py_DECREF(pValue);
//    vector<double> return_val;
//    if (PyList_Check(pState)) {
//        for(Py_ssize_t i = 0; i < PyList_Size(pState); i++) {
//            pValue = PyList_GetItem(pState, i);
//            return_val.push_back( PyFloat_AsDouble(pValue) );
//        }
//    }

//    Py_DECREF(l);
//    Py_DECREF(arglist);
//    Py_DECREF(pState);
    vector<double> temp = accessor_transform_down(nn_dim_reduce, *state, dim);
    return temp;

}

void PCAInterfacePythonAE::build_dim_reduction(int dim_max)
{
//    PyObject *arglist = Py_BuildValue("(i)", dim_max);
//    PyObject *pState = PyObject_CallObject(p_build_dim_reduction, arglist);
    accessor_build_dim_reduction(nn_dim_reduce, dim_max);
}

PCAInterfacePythonAE::~PCAInterfacePythonAE()
{

}
