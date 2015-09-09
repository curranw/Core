#include <AAAI2015.h>

void AAAI2015::setup_experiments()
{
    normal_learning_pole("Pole_Test");
//    for(unsigned int i = 0; i < 3; i++)
//    {
//        normal_learning4d("Normal_6");
//    }
//    for(unsigned int i = 0; i < 10; i++)
//    {
//        srand(time(NULL));
//        pca_learning_4d("8/converged_state_data_4d_good.csv", -1, "test_pca_6i_1_good");
//    }
    //normal_learning4d("Normal_6");
    //pca_learning("4/converged_state_data_3d_good.csv", -1, "test_pca_4i_1");
    //        for(unsigned int i = 0; i < 10; i++)
    //        {
    //            srand(time(NULL));
    //            normal_learning("Normal_4");
    //        }
    //        for(unsigned int i = 0; i < 10; i++)
    //        {
    //            srand(time(NULL));
    //            pca_learning("4/converged_state_data_3d_good.csv", -1, "test_pca_4_1",1, false);
    //        }
    //        for(unsigned int i = 0; i < 10; i++)
    //        {
    //            srand(time(NULL));
    //            pca_learning("4/converged_state_data_3d_good.csv", -1, "test_pca_4_2",2, false);
    //        }
    //        for(unsigned int i = 0; i < 10; i++)
    //        {
    //            srand(time(NULL));
    //            pca_learning("4/converged_state_data_3d_good.csv", -1, "test_pca_4_3",3, false);
    //        }


    //    for(unsigned int i = 0; i < 10; i++)
    //    {
    //        srand(time(NULL));
    //        pca_learning("4/converged_state_data_3d_bad.csv", -1, "test_pca_4i_1_bad",1, true);
    //    }
    //    for(unsigned int i = 0; i < 10; i++)
    //    {
    //        srand(time(NULL));
    //        pca_learning("4/converged_state_data_3d_bad.csv", 25000, "test_pca_4i_1_bad_25000",1, true);
    //    }
    //    for(unsigned int i = 0; i < 10; i++)
    //    {
    //        srand(time(NULL));
    //        pca_learning("4/converged_state_data_3d_random.csv", 1000, "test_pca_4i_1_random_1000",1, true);
    //    }
    
    //    for(unsigned int i = 0; i < 10; i++)
    //     {
    //            srand(time(NULL));
    //            pca_learning("4/converged_state_data_3d_good.csv", 1000, "test_pca_4i_1_good_1000",1, true);
    //     }

    //    for(unsigned int i = 0; i < 10; i++)
    //    {
    //        srand(time(NULL));
    //        pca_learning("4/converged_state_data_3d_random.csv", 1000, "test_pca_4i_1_random_1000",1, true);
    //    }

    /*
    for(unsigned int i = 0; i < 10; i++)
    {
        srand(time(NULL));
        pca_learning("4/converged_state_data_3d_random.csv", 10000, "test_pca_4i_1_random_10000",1, true);
    }
    for(unsigned int i = 0; i < 10; i++)
    {
        srand(time(NULL));
        pca_learning("4/converged_state_data_3d_random.csv", 1000, "test_pca_4i_1_random_1000",1, true);
    }
*/
//        for(int i = 0; i < 10; i++)
//        {
//            srand(time(NULL));
//            pca_learning_4d("converged_state_data_4d_good_1.csv", -1, "test_pca_6i_1");
//        }
//        for(int i = 0; i < 10; i++)
//        {
//            srand(time(NULL));
//            pca_learning_4d("converged_state_data_4d_bad_1.csv", -1, "test_pca_6i_1");
//        }
    //    for(int i = 0; i < 10; i++)
    //    {
    //        srand(time(NULL));
    //        pca_learning("4/converged_state_data_3d_good.csv", -1, "test_pca_4_3");
    //    }
//    for(int i = 0; i < 5; i++)
//    {
//        srand(time(NULL));
//        pca_learning_scaled("8/converged_state_data_3d_good.csv", -1, "test_pca_8");
//    }
}

void AAAI2015::normal_learning(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 10;

    vector<double> resolution;
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    learning_args->resolution = resolution;

    MountainCar3D* domain = new MountainCar3D();

    MountainCarExperimentArgs* experiment_args = new MountainCarExperimentArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 50000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    MountainCarExperiment* exp = new MountainCarExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void AAAI2015::normal_learning4d(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 10;

    vector<double> resolution;
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    learning_args->resolution = resolution;

    MountainCar4D* domain = new MountainCar4D();

    MountainCarExperimentArgs* experiment_args = new MountainCarExperimentArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 200000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    MountainCarExperiment* exp = new MountainCarExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void AAAI2015::normal_learning_scaled(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 10;

    vector<double> resolution;
    resolution.push_back(0.04);
    resolution.push_back(0.04);
    resolution.push_back(0.04);
    resolution.push_back(0.04);
    resolution.push_back(0.04);
    resolution.push_back(0.04);
    resolution.push_back(0.04);
    resolution.push_back(0.04);
    learning_args->resolution = resolution;

    MountainCar3DScaled* domain = new MountainCar3DScaled();

    MountainCarExperimentArgs* experiment_args = new MountainCarExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 200000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    MountainCarExperiment* exp = new MountainCarExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void AAAI2015::pca_learning(string pca_file, int amount, string save_file, int dim, bool iterative)
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

    QTilesReuse* learning_algorithm = new QTilesReuse(4, learning_args);


    MountainCar3D* domain = new MountainCar3D();

    MountainCarExperimentPCAArgs* experiment_args = new MountainCarExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 50000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    MountainCarExperimentPCA* exp = new MountainCarExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;

}

void AAAI2015::pca_learning_4d(string pca_file, int amount, string save_file)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 6; i++)
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
    resolution.push_back(0.09);
    resolution.push_back(0.09);
    resolution.push_back(0.09);
    resolution.push_back(0.007);
    resolution.push_back(0.007);
    resolution.push_back(0.007);
    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(6, learning_args);



    MountainCar4D* domain = new MountainCar4D();

    MountainCarExperimentPCAArgs* experiment_args = new MountainCarExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 200000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = true;
    experiment_args->start_dimension = 1;

    MountainCarExperimentPCA* exp = new MountainCarExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;

}

void AAAI2015::pca_learning_scaled(string pca_file, int amount, string save_file)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 8; i++)
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
    resolution.push_back(0.144);
    resolution.push_back(0.0112);
    resolution.push_back(0.0864);
    resolution.push_back(0.000392);
    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(8, learning_args);



    MountainCar3DScaled* domain = new MountainCar3DScaled();

    MountainCarExperimentPCAArgs* experiment_args = new MountainCarExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 200000;
    experiment_args->save_file = save_file;


    MountainCarExperimentPCA* exp = new MountainCarExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;

}

void AAAI2015::normal_learning_pole(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 10;

    vector<double> resolution;
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    learning_args->resolution = resolution;

    NPole* domain = new NPole();

    NPoleExperimentArgs* experiment_args = new NPoleExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 4000;
    experiment_args->num_epochs = 10000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    NPoleExperiment* exp = new NPoleExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;

}
