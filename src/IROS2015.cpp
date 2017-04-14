#include <IROS2015.h>
void IROS2015::setup_experiments()
{
    //normal_learning_swimmers("Large");
    //normal_learning_swimmers("shitty");

    //mountain_car_3d_delayed("Mountain_Car_3d_normal_8");
    //normal_learning_swimmers("Swimmers_3d_normal_TILES_TEST_4res");
    for(unsigned int i = 0; i < 10; i++)
    {
        srand(time(NULL));
        normal_learning_swimmers_large("Swimmers_6d_normal_4res");
        //mountain_car_3d_delayed("Mountain_Car_3d_normal_8");
        //mountain_car_3d_pca("mountain_car_3d_good_1.csv", -1, "Mountaincar_3d_PCA_PYTHON_fixed", 1, true);
        //normal_learning_swimmers("Swimmers_3d_normal_TILES_TEST");
        //normal_learning_swimmers("Swimmers_3d_normal_small_actions_dx_small_r_nn_1_200_l_01_linear_incremental_layered_minibatch_1_symmsw");
        //pca_learning_swimmers("converged_state_data_merge_Swimmers_3d_normal_full_actions_dx_small_r_1.csv", -1, "Swimmers_3d_PCA_full_actions_equalsize_dx_small_merge_r_250", 1, true);
        //pca_learning_swimmers("converged_state_data_swimmers_good_3d_dx.csv", -1, "Swimmers_3d_PCA_full_actions_equalsize_dx_good_250", 1, true);
        //pca_learning_swimmers("converged_state_data_swimmers_good_large_NEWEST_7.csv", -1, "Swimmers_3d_PCA_full_actions_equalsize_seed_1000", 1, true);


            //pca_learning_swimmers("converged_state_data_bad_Swimmers_3d_normal_small_actions_dx_small_r_2.csv", -1, "Swimmers_3d_PCA_small_actions_dx_bad_small_r_3_250", 1, true);




            //pca_learning_swimmers("converged_state_data_merge_Swimmers_3d_normal_small_actions_dx_small_r_1.csv", -1, "Swimmers_3d_PCA_small_actions_dx_merge_small_r_force_25_250_NN_200_static_l_01_PCA", 2, true);

            //pca_learning_swimmers("converged_state_data_merge_Swimmers_3d_normal_small_actions_dx_small_r_1.csv", -1, "Swimmers_3d_PCA_LINEAR_1_LAYERED_200_STATIC_INCREMENTAL_01_LR", 2, true);

            //pca_learning_swimmers("converged_state_data_merge_Swimmers_3d_normal_small_actions_dx_small_r_1.csv", -1, "Swimmers_3d_PCA_25_5_full_actions_4_res", 1, true);
            //pca_learning_swimmers("converged_state_data_merge_Swimmers_3d_normal_small_actions_dx_small_r_1.csv", -1, "Swimmers_3d_AE_250_50_full_actions_24_res_2048_moredata", 1, true);

        //pca_learning_swimmers("converged_state_data_merge_Swimmers_3d_normal_small_actions_dx_small_r_1.csv", -1, "Swimmers_3d_PCA_TESTING_4", 1, true);

        //pca_learning_swimmers("converged_state_data_merge_Swimmers_3d_normal_small_actions_dx_small_r_1.csv", -1, "Swimmers_3d_PCA_small_actions_dx_merge_small_r_force_25_250_LLE", 1, true);





        //normal_learning_swimmers_large("Swimmers_6d_normal_small_actions_dx_small_r");
        //pca_learning_swimmers_large("converged_state_data_swimmers_bad_6d_dx_1.csv", -1, "Swimmers_6d_PCA_full_actions_equalsize_dx_small_r_1000", 1, true);
              //pca_learning_swimmers_large("converged_state_data_bad_Swimmers_6d_normal_small_actions_dx_small_r_1.csv", -1, "Swimmers_6d_PCA_LARGE_8", 1, true);

              //Next: Start at 85% variance.

        //normal_learning_swimmers_12d("Swimmers_12d_normal_small_actions");
        //pca_learning_swimmers_12d("converged_state_data_swimmers_bad_12d_2.csv", -1, "Swimmers_12d_PCA_small_actions", 1, true);
    }
    //normal_learning_fitted_rmax("RobotChasing_RMAX.csv");
}

void IROS2015::mountain_car_3d_pca(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 4; i++)
    {
        QTilesArguments* learning_tiles_args = new QTilesArguments();
        learning_tiles_args->alpha = 0.1;
        learning_tiles_args->gamma = 0.99;
        learning_tiles_args->eligability = false;
        learning_tiles_args->num_tiles = 8;
        QTiles* learning_algorithm = new QTiles(learning_tiles_args);

        learning_args->learning_algorithms.push_back(learning_algorithm);
    }
    learning_args->pca_file = pca_file;
    learning_args->amount = amount;

    vector<double> resolution;
//    resolution.push_back(0.45);
//    resolution.push_back(0.45);
//    resolution.push_back(0.035);
//    resolution.push_back(0.035);
    resolution.push_back(0.2);
    resolution.push_back(0.2);
    resolution.push_back(0.00875);
    resolution.push_back(0.00875);
    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(4, learning_args);


    MountainCar3D* domain = new MountainCar3D();

    MountainCarExperimentPCAArgs* experiment_args = new MountainCarExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 10000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    MountainCarExperimentPCA* exp = new MountainCarExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;

}

void IROS2015::mountain_car_3d_delayed(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 32;

    vector<double> resolution;
    resolution.push_back(0.2);
    resolution.push_back(0.2);
    resolution.push_back(0.00875);
    resolution.push_back(0.00875);
    learning_args->resolution = resolution;

    MountainCar3D* domain = new MountainCar3D();

    MountainCarExperimentArgs* experiment_args = new MountainCarExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 10000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    MountainCarExperiment* exp = new MountainCarExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::normal_learning(std::string save_file)
{
    ValueIterationArgs* learning_args = new ValueIterationArgs();
    learning_args->gamma = .99;
    vector<double> resolution;

    resolution.push_back(1.0);
    resolution.push_back(1.0);
    resolution.push_back(1.0);
    resolution.push_back(1.0);
    learning_args->resolution = resolution;

    ValueIteration* learning_algorithm = new ValueIteration(learning_args);

    RobotChasing* domain = new RobotChasing();

    RobotChasingExperimentArgs* experiment_args = new RobotChasingExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 500;
    experiment_args->num_epochs = 500;
    experiment_args->save_file = save_file;

    RobotChasingExperiment* exp = new RobotChasingExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::normal_learning_swimmers_large(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 32;

    vector<double> resolution;

    resolution.push_back(0.5);
    resolution.push_back(0.5);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);

    learning_args->resolution = resolution;
    QTiles* learning_algorithm = new QTiles(learning_args);

    Swimmers* domain = new Swimmers(6);

    SwimmersExperimentArgs* experiment_args = new SwimmersExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 1500;
    experiment_args->num_epochs = 100000;
    experiment_args->save_file = save_file;

    SwimmersExperiment* exp = new SwimmersExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::normal_learning_swimmers(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 32;

    vector<double> resolution;

    //4-Resolution
    resolution.push_back(0.5);
    resolution.push_back(0.5);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
//        resolution.push_back(0.25);
//        resolution.push_back(0.25);
//        resolution.push_back(0.8);
//        resolution.push_back(2);
//        resolution.push_back(0.8);
//        resolution.push_back(2);
//        resolution.push_back(0.8);
//        resolution.push_back(2);

    learning_args->resolution = resolution;
    QTiles* learning_algorithm = new QTiles(learning_args);

    Swimmers* domain = new Swimmers(3);

    SwimmersExperimentArgs* experiment_args = new SwimmersExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 1500;
    experiment_args->num_epochs = 50000;
    experiment_args->save_file = save_file;

    SwimmersExperiment* exp = new SwimmersExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::normal_learning_swimmers_12d(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 64;

    vector<double> resolution;



    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);

    learning_args->resolution = resolution;
    QTiles* learning_algorithm = new QTiles(learning_args);

    Swimmers* domain = new Swimmers(12);

    SwimmersExperimentArgs* experiment_args = new SwimmersExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 1500;
    experiment_args->num_epochs = 200000;
    experiment_args->save_file = save_file;

    SwimmersExperiment* exp = new SwimmersExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::pca_learning_swimmers_12d(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 24; i++)
    {
        QTilesArguments* learning_tiles_args = new QTilesArguments();
        learning_tiles_args->alpha = 0.1;
        learning_tiles_args->gamma = 0.99;
        learning_tiles_args->eligability = false;
        learning_tiles_args->num_tiles = 32;
        QTiles* learning_algorithm = new QTiles(learning_tiles_args);

        learning_args->learning_algorithms.push_back(learning_algorithm);
    }
    learning_args->pca_file = pca_file;
    learning_args->weight_file = "converged_state_rewards_swimmers_good_large.csv";
    learning_args->amount = amount;

    vector<double> resolution;
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);
    resolution.push_back(3.2);
    resolution.push_back(8);

    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(24, learning_args);



    Swimmers* domain = new Swimmers(12);

    SwimmersExperimentPCAArgs* experiment_args = new SwimmersExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 1500;
    experiment_args->num_epochs = 200000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    SwimmersExperimentPCA* exp = new SwimmersExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::pca_learning_swimmers_large(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 14; i++)
    {
        QTilesArguments* learning_tiles_args = new QTilesArguments();
        learning_tiles_args->alpha = 0.1;
        learning_tiles_args->gamma = 0.99;
        learning_tiles_args->eligability = false;
        learning_tiles_args->num_tiles = 32;
        QTiles* learning_algorithm = new QTiles(learning_tiles_args);

        learning_args->learning_algorithms.push_back(learning_algorithm);
    }
    learning_args->pca_file = pca_file;
    learning_args->amount = amount;

    vector<double> resolution;
    resolution.push_back(0.5);
    resolution.push_back(0.5);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);

    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(14, learning_args);



    Swimmers* domain = new Swimmers(6);

    SwimmersExperimentPCAArgs* experiment_args = new SwimmersExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 1500;
    experiment_args->num_epochs = 100000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    SwimmersExperimentPCA* exp = new SwimmersExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::pca_learning_swimmers(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 8; i++)
    {
        QTilesArguments* learning_tiles_args = new QTilesArguments();
        learning_tiles_args->alpha = 0.1;
        learning_tiles_args->gamma = 0.99;
        learning_tiles_args->eligability = false;
        learning_tiles_args->num_tiles = 32;
        QTiles* learning_algorithm = new QTiles(learning_tiles_args);

        learning_args->learning_algorithms.push_back(learning_algorithm);
    }
    learning_args->pca_file = pca_file;
    //learning_args->weight_file = "converged_state_rewards_swimmers_good_1.csv";
    learning_args->amount = amount;

    vector<double> resolution;
    //2-Resolution
//    resolution.push_back(1);
//    resolution.push_back(1);
//    resolution.push_back(3.2);
//    resolution.push_back(8);
//    resolution.push_back(3.2);
//    resolution.push_back(8);
//    resolution.push_back(3.2);
//    resolution.push_back(8);

    //4-Resolution
    resolution.push_back(0.5);
    resolution.push_back(0.5);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);
    resolution.push_back(1.6);
    resolution.push_back(4);

    //8-Resolution
//    resolution.push_back(0.25);
//    resolution.push_back(0.25);
//    resolution.push_back(0.8);
//    resolution.push_back(2);
//    resolution.push_back(0.8);
//    resolution.push_back(2);
//    resolution.push_back(0.8);
//    resolution.push_back(2);

    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(8, learning_args);



    Swimmers* domain = new Swimmers(3);

    SwimmersExperimentPCAArgs* experiment_args = new SwimmersExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 1500;
    experiment_args->num_epochs = 50000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    SwimmersExperimentPCA* exp = new SwimmersExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::normal_learning_rmax(std::string save_file)
{
    RMaxArgs* learning_args = new RMaxArgs();
    learning_args->gamma = .99;
    vector<double> resolution;

    resolution.push_back(1.0);
    resolution.push_back(1.0);
    resolution.push_back(1.0);
    resolution.push_back(1.0);
    learning_args->resolution = resolution;
    RMax* learning_algorithm = new RMax(learning_args);

    RobotChasing* domain = new RobotChasing();

    RobotChasingExperimentArgs* experiment_args = new RobotChasingExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 500;
    experiment_args->num_epochs = 200;
    experiment_args->save_file = save_file;

    RobotChasingExperiment* exp = new RobotChasingExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::normal_learning_fitted_rmax(string save_file)
{
    FittedRMaxArgs* learning_args = new FittedRMaxArgs();
    learning_args->gamma = .99;
    vector<double> resolution;

    resolution.push_back(2.0);
    resolution.push_back(2.0);
    //resolution.push_back(4.0);
    //resolution.push_back(4.0);
    learning_args->resolution = resolution;
    FittedRMax* learning_algorithm = new FittedRMax(learning_args);

    RobotChasing* domain = new RobotChasing();

    RobotChasingExperimentArgs* experiment_args = new RobotChasingExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 15;
    experiment_args->num_epochs = 1000;
    experiment_args->save_file = save_file;

    RobotChasingExperiment* exp = new RobotChasingExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2015::pca_learning(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 4; i++)
    {
        QTilesArguments* learning_tiles_args = new QTilesArguments();
        learning_tiles_args->alpha = 0.1;
        learning_tiles_args->gamma = 0.99;
        learning_tiles_args->eligability = false;
        learning_tiles_args->num_tiles = 10;
        QTiles* learning_algorithm = new QTiles(learning_tiles_args);

        learning_args->learning_algorithms.push_back(learning_algorithm);
    }
    learning_args->pca_file = pca_file;
    learning_args->amount = amount;

    vector<double> resolution;
    resolution.push_back(0.072);
    resolution.push_back(0.072);
    resolution.push_back(0.0056);
    resolution.push_back(0.0056);
    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(6, learning_args);



    NArm* domain = new NArm();

    NArmExperimentPCAArgs* experiment_args = new NArmExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 50000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    NArmExperimentPCA* exp = new NArmExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;

}
