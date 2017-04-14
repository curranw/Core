#include <PCAInterface.h>

//#include "autoencoder_api_cdef.c"
//#include <tapkee/tapkee.hpp>

//using namespace tapkee;

PCAInterface::PCAInterface(int num_variables)
{
    use_python = false;
    use_ae = false;
    use_keras = true;
    use_keras_single = false;
    if(use_python)
    {
        if(use_ae) pipae = new PCAInterfacePythonAE();
        else pip = new PCAInterfacePython();

    }
    use_pca = false;

    use_lle = false;
    use_ros = false;
    use_kpca = false;
    use_shogun = false;
    use_iso = false;
    PCAobj = new PCA;
    if(use_pca)
    {

        pca = new stats::pca(num_variables);
    }
    //    else if (use_kpca)
    //    {
    //        kpca = new KPCA();
    //        kpca->set_components(2);
    //        kpca->set_normalise(1);
    //        kpca->set_kernel(2);
    //        kpca->set_order(2);
    //        kpca->load_data("converged_state_data_swimmers_good_large_NEWEST_2.csv");
    //    }

    if(use_ros) pir = new PCAInterfaceROS();

}

PCAInterface::~PCAInterface()
{
    if(use_pca)
    {
        delete pca;
    }
    //    else if(use_kpca)
    //    {
    //        delete kpca;
    //    }
}

void PCAInterface::solve()
{
    if(use_keras || use_keras_single) return;
    if(use_pca)
    {
        pca->set_do_normalize(false);
        pca->solve();
        pca->save("Swimmers_3d_dx");
        //get_error();
    }
    if(use_python)
    {

    }
    //    else if(use_kpca)
    //    {
    //        kpca->run_kPCA();
    //    }



    //        DenseMatrix matrix(saved_data[0].size(), int(saved_data.size()/1000.0));

    //        for(unsigned int i = 0; i < saved_data[0].size(); i++)
    //        {
    //            for(unsigned int j = 0; j < saved_data.size()/1000.0; j++)
    //            {
    //                matrix(i,j) = saved_data[j][i];
    //            }
    //        }
    //        DenseMatrix matrix(saved_data.size(), saved_data[0].size());

    //        for(unsigned int i = 0; i < saved_data.size(); i++)
    //        {
    //            for(unsigned int j = 0; j < saved_data[0].size(); j++)
    //            {
    //                matrix(i,j) = saved_data[i][j];
    //            }
    //        }

    //        TapkeeOutput output = initialize()
    //            .withParameters((method=KernelPCA, target_dimension=2))
    //            .embedUsing(matrix);

    //        DenseVector dv(saved_data[0].size());
    //        for(unsigned int i = 0; i < saved_data[0].size(); i++)
    //        {
    //            dv(i) = saved_data[0][i];
    //        }

    //        DenseVector dv2 = output.projection(dv);
    //    pca->save("Arm_Touch_Normalized");
    //    exit(1);
    //pca->set_do_normalize(false);
}

void PCAInterface::add_data(vector<double>* data)
{
    if(use_pca) pca->add_record(*data);
}

void PCAInterface::add_data(vector<vector<double> >* data)
{
    if(use_keras)
    {
        for(unsigned int i = 1; i <= data->at(0).size(); i++)
        {
            KerasModel* model = new KerasModel();
            //            model->LoadModel("/nfs/attic/smartw/users/curranw/ManifoldLearningNew/ManifoldLearning/src/Core/PCAInterface/encoder.model.mtn_car.regular." + std::to_string(i));
            model->LoadModel("/nfs/attic/smartw/users/curranw/ManifoldLearningNew/ManifoldLearning/src/Core/PCAInterface/encoder.model.batch_2048.evenmoredata." + std::to_string(i));
            models.push_back(model);

        }
    }
    if(use_keras_single)
    {
        KerasModel* model = new KerasModel();
        model->LoadModel("/nfs/attic/smartw/users/curranw/ManifoldLearningNew/ManifoldLearning/src/Core/PCAInterface/encoder.model.single");
        models.push_back(model);
    }

    if(use_python)
    {
        if(use_ae)
        {
            for(unsigned int i = 0; i < data->size(); i++)
            {
                pipae->add_data(&data->at(i));
            }
            pipae->build_dim_reduction(data->at(0).size());
        }
        else
        {
            for(unsigned int i = 0; i < data->size(); i++)
            {
                pip->add_data(&data->at(i));
            }
            pip->build_dim_reduction(0, data->at(0).size());
        }


        //        pca = new stats::pca(data->at(0).size());
        //        for(vector<vector<double> >::iterator it = data->begin(); it != data->end(); it++)
        //        {
        //            pca->add_record(*it);
        //        }
        //        pca->set_do_normalize(false);
        //        pca->solve();
        return;
    }
    if(use_ros) return;
    //    DenseMatrix a(data->at(0).size(), 5000);
    //    for(unsigned int i = 0; i < 5000; i++)
    //    {
    //        for(unsigned int j = 0; j < data->at(i).size(); j++)
    //        {
    //            a(j,i) = data->at(i).at(j);
    //        }
    //    }

    //    TapkeeOutput output = tapkee::initialize().withParameters((method=tapkee::PCA, target_dimension=1, eigen_method=tapkee::Randomized)).embedUsing(a);


    //    int dim = data->at(0).size();
    //    max.assign(dim,0);
    //    min.assign(dim,0);
    //    for(vector<vector<double> >::iterator it = data->begin(); it != data->end(); it++)
    //    {
    //        vector<double> cur_data = *it;
    //        if(it == data->begin())
    //        {
    //            for(int i = 0; i < dim; i++)
    //            {
    //                max[i] = cur_data[i];
    //                min[i] = cur_data[i];
    //            }
    //        }
    //        else
    //        {
    //            for(int i = 0; i < dim; i++)
    //            {
    //                if(cur_data[i] > max[i]) max[i] = cur_data[i];
    //                if(cur_data[i] < min[i]) min[i] = cur_data[i];
    //            }
    //        }
    //    }
    //    for(vector<vector<double> >::iterator it = data->begin(); it != data->end(); it++)
    //    {
    //        for(unsigned int i = 0; i < dim; i++)
    //        {
    //             it->at(i) = (it->at(i) - min[i])/(max[i] - min[i]);
    //        }
    //    }
    //    for(vector<vector<double> >::iterator it = data->begin(); it != data->end(); it++)
    //    {
    //        cout << it->at(0) << "," << it->at(1) << "," << it->at(2) << endl;
    //    }
    //    vector<vector<double> > temp_data;
    //    for(unsigned int i = 0; i < data->size(); i++)
    //    {
    //        vector<double> cur_vec = data->at(i);
    //        double min_dist = 999999999;
    //        for(unsigned int j = 0; j < temp_data.size(); j++)
    //        {
    //            vector<double> vec_compare = temp_data.at(j);
    //            double dist = 0;
    //            for(unsigned int k = 0; k < vec_compare.size(); k++)
    //            {
    //                dist += abs(cur_vec[k] - vec_compare[k]) * abs(cur_vec[k] - vec_compare[k]);
    //            }
    //            dist = sqrt(dist);
    //            if(dist < min_dist) min_dist = dist;
    //        }
    //        if(min_dist > 0.01) temp_data.push_back(cur_vec);
    //    }

    //    for(unsigned int i = 0; i < temp_data.size(); i++)
    //    {
    //        temp_data[i][4] = wrap(temp_data[i][4]);
    //        temp_data[i][10] = wrap(temp_data[i][10]);
    //    }
    //    for(vector<vector<double> >::iterator it = temp_data.begin(); it != temp_data.end(); it++)
    //    {
    //        pca->add_record(*it);
    //    }

    if(use_pca)
    {
        for(vector<vector<double> >::iterator it = data->begin(); it != data->end(); it++)
        {
            pca->add_record(*it);
        }
    }

    if(use_lle)
    {

        Matrix<double> M(data->at(0).size(),data->size()/100.0);
        training_data.reallocate(data->at(0).size(),data->size()/100.0);
        for(int i=0; i<data->at(0).size(); i++)
            for(int j=0; j<data->size()/100.0; j++)
                M(i,j) = data->at(j).at(i);

        training_data = M;
        for(unsigned int i = 1; i <= data->at(0).size()-1; i++)
        {
            LLE* temp = new LLE(training_data);
            temp->set_d_out(i);
            temp->set_k(i*2);
            temp->compute_projection();
            temp->set_quiet(true);
            all_LLEobj.push_back(temp);
        }
    }


    //    PCAobj->set_training_data(M);
    //    PCAobj->set_d_out(6);
    //    PCAobj->compute();




    //LLEobj->set_quiet(true);
    //    solve();
    //    set_projection_dimension(1);
    //    vector<double> out_old = transform_down(&data->at(0));
    //    DenseMatrix test = DenseMatrix(6,1);
    //    for(unsigned int i = 0; i < 6; i++)
    //    {
    //        test(i,0) = a(i,0);
    //    }
    //    DenseMatrix out = output.projection.implementation->project(test);
    //    pca->save("Swimmers");
    //    exit(1);
    saved_data = *data;

    if(use_shogun)
    {

        shogun::init_shogun_with_defaults();

        shogun::CDenseFeatures<float64_t>* cf = new shogun::CDenseFeatures<float64_t>();
        shogun::SGMatrix<float64_t> sg_data(data->at(0).size(), data->size()/50.0);
        SG_REF(cf);

        for(int32_t i = 0; i < data->at(0).size(); i++)
        {
            for(int32_t j = 0; j < data->size()/50.0; j++)
            {
                sg_data(i,j) = data->at(j).at(i);
            }
        }
        cf->set_feature_matrix(sg_data);

        if(use_pca)
        {
            for(unsigned int i = 1; i <= data->at(0).size(); i++)
            {
                s_pca = new shogun::CPCA();
                s_pca->set_target_dim(i);
                s_pca->init(cf);
                list_spca.push_back(s_pca);
            }
            s_pca = list_spca[0];
        }
        if(use_kpca)
        {
            cout << "PCA" << endl;
            //shogun::CGaussianKernel* gk = new shogun::CGaussianKernel(&cf, &cf, 1);
            gk = new shogun::CGaussianKernel();
            s_kpca = new shogun::CKernelPCA(gk);
            SG_REF(s_kpca);
            SG_REF(gk);
            s_kpca->init(cf);
        }
        if(use_iso)
        {
            iso = new shogun::CIsomap();
            SG_REF(iso);
            iso->embed(cf);
        }
    }
}

void PCAInterface::add_weights(vector<double> *weights)
{
    pca->set_weights(*weights);
}

void PCAInterface::set_projection_dimension(int projection_dimension)
{
    m_projection_dimension = projection_dimension;
    if(use_keras || use_keras_single)
    {
        //cur_model = models.at(projection_dimension-1);
        return;
    }
    if(use_lle)
    {
        LLEobj = all_LLEobj[projection_dimension-1];
        //        LLEobj->set_d_out(projection_dimension);
        //        if(LLEobj->has_training_proj())
        //            LLEobj->clear_training_proj();
        //        LLEobj->compute_projection();
    }
    if(use_pca)
    {
        pca->set_num_retained(projection_dimension);
    }

    if(use_ros) pir->set_projection_dimension(projection_dimension);
    if(use_shogun)
    {
        if(use_pca) s_pca = list_spca[projection_dimension-1];
        //        if(use_kpca) s_kpca = list_skpca[projection_dimension-1];
        //        if(use_kpca) s_kpca->set_target_dim(projection_dimension);
        //        s_pca->cleanup();
        //        s_pca->set_target_dim(projection_dimension);
        //        shogun::CDenseFeatures<float64_t> cf;
        //        shogun::SGMatrix<float64_t> sg_data(saved_data.at(0).size(), saved_data.size());
        //        SG_REF(&cf);

        //        for(int32_t i = 0; i < saved_data.at(0).size(); i++)
        //        {
        //            for(int32_t j = 0; j < saved_data.size(); j++)
        //            {
        //                sg_data(i,j) = saved_data.at(j).at(i);
        //            }
        //        }
        //        cf.set_feature_matrix(sg_data);
        //        s_pca->init(&cf);
    }
    //    else if (use_kpca)
    //    {
    //        kpca->set_components(projection_dimension);
    //    }
}

vector<double> PCAInterface::transform_down(vector<double>* data)
{
    if(use_keras)
    {
        vector<double> temp;
        for(unsigned int i = 0; i < m_projection_dimension; i++)
        {
            Tensor in(1);
            in.data_ = vector<float>(data->begin(), data->end());
            Tensor out(1);
            models[i]->Apply(&in, &out);
            temp.push_back(out.data_[0]);
        }
        return temp;

    }
    if(use_keras_single)
    {
        vector<double> temp;
        Tensor in(1);
        in.data_ = vector<float>(data->begin(), data->end());
        Tensor out(1);
        models[0]->Apply(&in, &out);
        for(unsigned int i = 0; i < m_projection_dimension; i++)
        {
            temp.push_back(out.data_[i]);
        }
        return temp;
    }
    static int num_times = 0;
    static double avg_time = 0;
    static double tot_time = 0;
    clock_t begin = clock();
    if(use_python)
    {
        vector<double> temp;
        if(use_ae)
        {
            temp = pipae->transform_down(data, m_projection_dimension);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            tot_time += elapsed_secs;
            num_times++;
            avg_time = tot_time/double(num_times);
            if(num_times % 100000 == 0)
            {
                cout << avg_time << endl;
                num_times = 0;
                tot_time = 0;
            }
        }
        else
        {
            temp = pip->transform_down(data, m_projection_dimension);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            tot_time += elapsed_secs;
            num_times++;
            avg_time = tot_time/double(num_times);
            if(num_times % 100000 == 0)
            {
                cout << avg_time << endl;
                num_times = 0;
                tot_time = 0;
            }
        }
        return temp;
    }
    if(use_lle)
    {
        Matrix<double> M(data->size(), 1);
        for(unsigned int i = 0; i < data->size(); i++)
        {
            M(i,0) = data->at(i);
        }
        if(LLEobj->has_test_proj())
        {
            LLEobj->clear_test_data();
        }
        LLEobj->compute_projection(M);
        const Matrix<double> output_M = LLEobj->test_projection();

        vector<double> output;
        for(unsigned int i = 0; i < output_M.nrows(); i++)
        {
            output.push_back(output_M(i,0));
        }
        return output;
    }
    if(use_pca && !use_shogun)
    {
        return pca->to_principal_space(*data);
    }
    if(use_ros) return pir->transform_down(data);
    if(use_shogun)
    {
        shogun::SGVector<float64_t>* sg_data = new shogun::SGVector<float64_t>(data->size());
        for(unsigned int i = 0; i < data->size(); i++)
        {
            sg_data->set_element(data->at(i), i);
        }
        shogun::SGMatrix<float64_t> test_result;
        if(use_pca) test_result =s_pca->apply_to_feature_vector(*sg_data);
        if(use_kpca) test_result =s_kpca->apply_to_feature_vector(*sg_data);
        float64_t* ft = test_result.data();
        vector<double> v(m_projection_dimension);
        for(unsigned int i = 0; i < m_projection_dimension; i++)
        {
            v[i] = ft[i];
        }


        //        s_kpca->set_target_dim(2);
        //        if(use_pca) test_result =s_pca->apply_to_feature_vector(sg_data);
        //        if(use_kpca) test_result =s_kpca->apply_to_feature_vector(sg_data);
        //        ft = test_result.data();
        //        for(unsigned int i = 0; i < 2; i++)
        //        {
        //            v[i] = ft[i];
        //        }



        return v;
    }
    //    else if(use_kpca)
    //    {
    //        return kpca->project_kPCA(*data);
    //    }

}

vector<double> PCAInterface::transform_up(vector<double>* data)
{
    return pca->to_variable_space(*data);
}

vector<double> PCAInterface::get_error()
{
    vector<vector<double> > pca_position_data = utils::read_csv<double>("all_states_mountaincar_3d_1.csv");
    for(unsigned int retained = 1; retained <= 4; retained++)
    {
        pca->set_num_retained(retained);
//        int total_records = pca->get_num_records();
        int total_records = pca_position_data.size();
        double all_error = 0;
        for(unsigned int i = 0; i < total_records; i++)
        {
//            vector<double> record = pca->get_record(i);
            vector<double> record = pca_position_data[i];
            vector<double> down_record = pca->to_principal_space(record);
            vector<double> up_record = pca->to_variable_space(down_record);
            double err = 0;
            for(unsigned int i = 0; i < up_record.size(); i++)
            {
                err += (abs(record[i] - up_record[i])) * (abs(record[i] - up_record[i]));
            }
            all_error += err;
        }

        all_error = all_error/double(total_records);
        all_error = sqrt(all_error);
        cout << all_error << endl;
    }
    //pca->set_do_normalize(true);
    exit(1);
    return vector<double>();
}

double PCAInterface::get_neighbor_distance()
{
    //pca->set_do_normalize(false);
    int total_records = saved_data.size();
    double min_diff = 9999999;
    for(unsigned int i = 0; i < 10; i++)
    {
        vector<double> record = saved_data.at(i);
        vector<double> down_record = pca->to_principal_space(record);
        cout << down_record[0] << "," << down_record[1] << "," << down_record[2] << "," << down_record[3] << "," << down_record[4] << endl;
        exit(1);
        for(unsigned int j = 0; j < 10; j++)
        {
            if(i == j) continue;
            vector<double> record = saved_data.at(j);
            vector<double> down_record_compare = pca->to_principal_space(record);
            double diff = 0;
            for(unsigned int k = 0; k < down_record_compare.size(); k++)
            {
                diff += pow((abs(down_record[i] - down_record_compare[i])),2);
            }
            diff = sqrt(diff);
            if(diff < 0.0001) continue;
            if(diff < min_diff) min_diff = diff;
        }
    }
    //pca->set_do_normalize(true);
    return min_diff;

    //    int total_records = pca->get_num_records();
    //    double min_diff = 9999999;
    //    for(unsigned int i = 0; i < total_records/4; i++)
    //    {
    //        vector<double> record = pca->get_record(i);
    //        vector<double> down_record = pca->to_principal_space(record);
    //        for(unsigned int j = 0; j < total_records/4; j++)
    //        {
    //            if(i == j) continue;
    //            vector<double> record = pca->get_record(j);
    //            vector<double> down_record_compare = pca->to_principal_space(record);
    //            double diff = 0;
    //            for(unsigned int k = 0; k < down_record_compare.size(); k++)
    //            {
    //                diff += sqrt(pow((abs(down_record[i] - down_record_compare[i])),2));
    //            }
    //            if(diff < 0.0001) continue;
    //            if(diff < min_diff) min_diff = diff;
    //        }
    //    }
    //    return min_diff;
}

void PCAInterface::normalize(vector<double> *data)
{
    for(unsigned int i = 0; i < data->size(); i++)
    {
        data->at(i) = (data->at(i) - min[i])/(max[i] - min[i]);
    }
}

double PCAInterface::wrap(double rad)
{
    rad = fmod(rad, 2.0 * 3.14159);
    if (rad < 0.0)
        rad += 2.0 * 3.14159;
    return rad;
}
