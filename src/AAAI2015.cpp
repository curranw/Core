#include <AAAI2015.h>
void AAAI2015::setup_experiments()
{
    normal_learning("Normal_4");
//    for(unsigned int i = 0; i < 20; i++)
//    {
//        srand(time(NULL));
//        normal_learning("Normal_4");
//        //pca_learning("mountain_car_3d_good.csv", -1, "best_mountain_car",2, true);
//    }
//    //normal_learning("Normal_4");
//    pca_learning("mountain_car_3d_good.csv", -1, "blah",2, true);
    //normal_learning("Mountain_MounCar_3D");
    //normal_learning_fitted_rmax("Mountain Car");
    //normal_learning4d_fitted_rmax("Mountain Car");
    //    for(unsigned int i = 0; i < 19; i++)
//    {
//        normal_learning_blackjack("Blackjack_Normal");
//    }
//    for(unsigned int i = 0; i < 20; i++)
//    {
//        pca_learning_blackjack("converged_state_data_blackjack_good_2.csv", -1, "test_blackjack_new",1, true);
//    }
    //pca_learning_pole("converged_state_data_pole_good.csv", -1, "test_pole",1, true);
    //normal_learning_pole("Pole_Test");
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
    learning_args->num_tiles = 32;

    vector<double> resolution;
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    learning_args->resolution = resolution;

    MountainCar3D* domain = new MountainCar3D();

    MountainCarExperimentArgs* experiment_args = new MountainCarExperimentArgs();
    experiment_args->demonstrations = true;
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

void AAAI2015::normal_learning_fitted_rmax(string save_file)
{
    FittedRMaxArgs* learning_args = new FittedRMaxArgs();
    learning_args->gamma = .99;
    vector<double> resolution;
    resolution.push_back(0.12);
    resolution.push_back(0.0140);
    learning_args->resolution = resolution;
    FittedRMax* learning_algorithm = new FittedRMax(learning_args);

    MountainCar* domain = new MountainCar();

    MountainCarExperimentArgs* experiment_args = new MountainCarExperimentArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 300;
    experiment_args->num_epochs = 500;
    experiment_args->save_file = save_file;


    MountainCarExperiment* exp = new MountainCarExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void AAAI2015::normal_learning4d_fitted_rmax(string save_file)
{
    FittedRMaxArgs* learning_args = new FittedRMaxArgs();
    learning_args->gamma = .99;
    vector<double> resolution;
    //resolution.push_back(0.1);
    //resolution.push_back(0.01);
    resolution.push_back(0.12);
    resolution.push_back(0.12);
    resolution.push_back(0.035);
    resolution.push_back(0.035);
    learning_args->resolution = resolution;
    FittedRMax* learning_algorithm = new FittedRMax(learning_args);

    MountainCar3D* domain = new MountainCar3D();

    MountainCarExperimentArgs* experiment_args = new MountainCarExperimentArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 300;
    experiment_args->num_epochs = 500;
    experiment_args->save_file = save_file;


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
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
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
    resolution.push_back(0.05);
    resolution.push_back(0.05);
    resolution.push_back(0.05);
    resolution.push_back(0.05);
//    resolution.push_back(0.05);
//    resolution.push_back(0.05);
//    resolution.push_back(0.1);
//    resolution.push_back(0.1);
//    resolution.push_back(0.05);
//    resolution.push_back(0.05);
//    resolution.push_back(0.05);
//    resolution.push_back(0.05);
    learning_args->resolution = resolution;

    NPole* domain = new NPole();

    NPoleExperimentArgs* experiment_args = new NPoleExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 1000;
    experiment_args->num_epochs = 100000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    NPoleExperiment* exp = new NPoleExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;

}

void AAAI2015::pca_learning_pole(string pca_file, int amount, string save_file, int dim, bool iterative)
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
    resolution.push_back(0.05);
    resolution.push_back(0.05);
    resolution.push_back(0.05);
    resolution.push_back(0.05);
//    resolution.push_back(0.1);
//    resolution.push_back(0.1);
    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(resolution.size(), learning_args);


    NPole* domain = new NPole();

    NPoleExperimentPCAArgs* experiment_args = new NPoleExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 1000;
    experiment_args->num_epochs = 100000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    NPoleExperimentPCA* exp = new NPoleExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;

}

void AAAI2015::normal_learning_blackjack(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 16;

    vector<double> resolution;
    resolution.push_back(0.05);
    resolution.push_back(0.1);
    resolution.push_back(0.5);

    //Digits
//    resolution.push_back(0.1);
//    resolution.push_back(0.1);
//    resolution.push_back(0.1);
//    resolution.push_back(0.1);

    learning_args->resolution = resolution;

    Blackjack* domain = new Blackjack();

    BlackjackExperimentArgs* experiment_args = new BlackjackExperimentArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 1000;
    experiment_args->num_epochs = 100000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    BlackjackExperiment* exp = new BlackjackExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;

}

void AAAI2015::pca_learning_blackjack(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 7; i++)
    {
        QTilesArguments* learning_tiles_args = new QTilesArguments();
        learning_tiles_args->alpha = 0.1;
        learning_tiles_args->gamma = 1;
        learning_tiles_args->eligability = false;
        learning_tiles_args->num_tiles = 16;
        QTiles* learning_algorithm = new QTiles(learning_tiles_args);

        learning_args->learning_algorithms.push_back(learning_algorithm);
    }
    learning_args->pca_file = pca_file;
    learning_args->amount = amount;

    vector<double> resolution;
    resolution.push_back(0.05);
    resolution.push_back(0.1);
    resolution.push_back(0.5);

    //Digits
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    resolution.push_back(0.1);
    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(resolution.size(), learning_args);


    Blackjack* domain = new Blackjack();

    BlackjackExperimentPCAArgs* experiment_args = new BlackjackExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 1000;
    experiment_args->num_epochs = 100000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    BlackjackExperimentPCA* exp = new BlackjackExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;

}

