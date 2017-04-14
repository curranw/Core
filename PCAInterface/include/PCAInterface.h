#pragma once

#include <PCAInterfacePython.h>
#include <PCAInterfacePythonAE.h>

#include <pca.h>
#include <vector>
#include <PCAInterfaceROS.h>
#include <shogun/preprocessor/PCA.h>
#include <shogun/preprocessor/KernelPCA.h>
#include <shogun/converter/Isomap.h>
#include <shogun/kernel/GaussianKernel.h>
#include <shogun/kernel/LinearKernel.h>
#include <shogun/base/init.h>

#include <shogun/lib/config.h>
#include <shogun/labels/MulticlassLabels.h>
#include <shogun/preprocessor/FisherLDA.h>
#include <shogun/features/DenseFeatures.h>
#include <shogun/lib/common.h>
#include <shogun/features/DataGenerator.h>
#include "PCA.h"
#include "LLE.h"
#include "keras_model.h"
#include <utils.h>
//#include <kpca.h>


using namespace std;
class PCAInterface
{
public:
    PCAInterface(int num_variables);
    virtual ~PCAInterface();
    stats::pca* pca;
    //KPCA* kpca;
    void solve();
    void add_data(vector<double> *data);
    void add_data(vector<vector<double> > *data);
    void add_weights(vector<double> *weights);
    void set_projection_dimension(int projection_dimension);
    vector<double> transform_down(vector<double> *data);
    vector<double> transform_up(vector<double> *data);
    vector<double> get_error();
    void normalize(vector<double> *data);
    double get_neighbor_distance();

    vector<double> min;
    vector<double> max;
    vector<vector<double> > saved_data;
    double wrap(double rad);

    bool use_pca;
    bool use_kpca;
    bool use_shogun;
    bool use_ros;
    bool use_iso;
    bool use_python;
    bool use_keras;
    bool use_keras_single;
    PCAInterfaceROS* pir;

    shogun::CPCA* s_pca;
    vector<shogun::CPCA*> list_spca;

    shogun::CKernelPCA* s_kpca;
    //shogun::CGaussianKernel* gk;
    shogun::CGaussianKernel* gk;
    shogun::CIsomap* iso;
    vector<shogun::CKernelPCA*> list_skpca;

    PCA* PCAobj;

    vector<LLE*> all_LLEobj;
    LLE* LLEobj;
    bool use_lle;
    Matrix<double> training_data;

    bool use_ae;
    PCAInterfacePython* pip;
    PCAInterfacePythonAE* pipae;

    vector<KerasModel*> models;
    KerasModel* cur_model;

private:
    int m_projection_dimension;

};
