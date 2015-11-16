#include <IROS2015.h>
void IROS2015::setup_experiments()
{
    normal_learning_fitted_rmax("RobotChasing_RMAX.csv");
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
