#include <PCAInterface.h>

PCAInterface::PCAInterface(int num_variables)
{
    pca = new stats::pca(num_variables);
}

PCAInterface::~PCAInterface()
{
    delete pca;
}

void PCAInterface::solve()
{
    pca->set_do_normalize(true);
    pca->solve();
    //pca->set_do_normalize(false);
}

void PCAInterface::add_data(vector<double>* data)
{
    pca->add_record(*data);
}

void PCAInterface::add_data(vector<vector<double> >* data)
{
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
    for(vector<vector<double> >::iterator it = data->begin(); it != data->end(); it++)
    {
        pca->add_record(*it);
    }
    saved_data = *data;
}

void PCAInterface::set_projection_dimension(int projection_dimension)
{
    pca->set_num_retained(projection_dimension);
}

vector<double> PCAInterface::transform_down(vector<double>* data)
{
//    data->at(0) = -0.499177;
//    data->at(1) = 0.997317;
//    data->at(2) = 0.000823157;
//    pca->set_num_retained(1);
    return pca->to_principal_space(*data);
}

vector<double> PCAInterface::transform_up(vector<double>* data)
{
    return pca->to_variable_space(*data);
}

vector<double> PCAInterface::get_error()
{
    pca->set_do_normalize(false);
    int original_dimension = pca->get_record(0).size();
    int total_records = pca->get_num_records();
    vector<double> all_errors(original_dimension);
    for(unsigned int i = 0; i < total_records; i++)
    {
        vector<double> record = pca->get_record(i);
        vector<double> down_record = pca->to_principal_space(record);
        vector<double> up_record = pca->to_variable_space(down_record);
        for(unsigned int i = 0; i < up_record.size(); i++)
        {
            double err = (abs(record[i] - up_record[i]));
            err = err * err;
            all_errors[i] += err;
        }
    }

    for(unsigned int i = 0; i < all_errors.size(); i++)
    {
        all_errors[i] = all_errors[i]/total_records;
    }
    //pca->set_do_normalize(true);

    return all_errors;
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
