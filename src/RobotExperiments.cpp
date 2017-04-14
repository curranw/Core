#include <RobotExperiments.h>
void RobotExperiments::setup_experiments()
{
    //    normal_learning_turtle_stairs("Stairs_Test");
    for(unsigned int i = 0; i < 1; i++)
    {
        srand(time(NULL));
        normal_learning_ball_balance("BallBalance");
    }
}

void RobotExperiments::normal_learning_ball_balance(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 32;

    vector<double> resolution;
//    resolution.push_back(0.05);
    resolution.push_back(0.025);
    resolution.push_back(0.025);
    resolution.push_back(0.025);
    resolution.push_back(0.025);
    resolution.push_back(0.025);
    resolution.push_back(0.03125);
    resolution.push_back(0.03125);
    resolution.push_back(0.03125);
    resolution.push_back(0.03125);
    learning_args->resolution = resolution;

    BallBalance* domain = new BallBalance();

    domain->m_accumulate_data = true;
    BallBalanceExperimentArgs* experiment_args = new BallBalanceExperimentArgs();
    experiment_args->num_steps = 10000;
    experiment_args->num_epochs = 100000;
    experiment_args->save_file = save_file;
    QTiles* learning_algorithm = new QTiles(learning_args);

    BallBalanceExperiment* exp = new BallBalanceExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}
