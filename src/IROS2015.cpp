#include <IROS2015.h>
void IROS2015::setup_experiments()
{
    pca_learning("converged_state_data_arm_2_good_1.csv", -1, "Arm_PCA", 1, true);
}

void IROS2015::normal_learning(std::string save_file)
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
    learning_args->resolution = resolution;

    NArm* domain = new NArm();

    NArmExperimentArgs* experiment_args = new NArmExperimentArgs();
    experiment_args->demonstrations = true;
    experiment_args->num_steps = 2000;
    experiment_args->num_epochs = 200000;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    NArmExperiment* exp = new NArmExperiment(domain, learning_algorithm, experiment_args);
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
