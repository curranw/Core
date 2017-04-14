#include <PCAInterfaceROS.h>


PCAInterfaceROS::PCAInterfaceROS()
{
    vector<pair<string, string> > temp;
    ros::init(temp, "dim_reduction_client");

    node_handler = new ros::NodeHandle();

    sklearn_interface::BuildDimensionalityReduction bdr;
    bdr.request.dimensionality_reduction = "RANDOM";
    bdr.request.filename = "/nfs/attic/smartw/users/curranw/ManifoldLearningNew/ManifoldLearning/build/Core/converged_state_data_swimmers_good_large_NEWEST_2.csv";
    ros::service::waitForService("/build_dim_reduction");
    bdr_client = node_handler->serviceClient<sklearn_interface::BuildDimensionalityReduction>("/build_dim_reduction");
    bdr_client.call(bdr);

    ros::service::waitForService("/dim_reduction");
    dr_client = node_handler->serviceClient<sklearn_interface::DimensionalityReduction>("/dim_reduction", true);

//    ros::Rate poll_rate(10);
//    ros::spinOnce();
//    poll_rate.sleep();
}

PCAInterfaceROS::~PCAInterfaceROS()
{

}

void PCAInterfaceROS::set_projection_dimension(int projection_dimension)
{
    m_projection_dimension = projection_dimension;
}

vector<double> PCAInterfaceROS::transform_down(vector<double>* data)
{
    sklearn_interface::DimensionalityReduction dr;

    dr.request.dimension = m_projection_dimension;
    for(unsigned int i = 0; i < data->size(); i++)
    {
        dr.request.input.push_back(data->at(i));
    }
    dr_client.call(dr);

    vector<double> return_val;
    for(unsigned int i = 0; i < dr.response.output.size(); i++)
    {
        return_val.push_back(dr.response.output.at(i));
    }
    return return_val;
}
