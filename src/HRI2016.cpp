#include <HRI2016.h>
void HRI2016::setup_experiments()
{
    for(unsigned int i = 0; i < 10; i++)
    {
        normal_learning_turtle_maze("Testing_no_reward");
    }
}


void HRI2016::normal_learning_turtle_maze(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = true;
    learning_args->num_tiles = 32;

    vector<double> resolution;
    resolution.push_back(0.05);
    resolution.push_back(0.05);
    resolution.push_back(0.1);
    learning_args->resolution = resolution;

    TurtleMaze* domain = new TurtleMaze();

    TurtleMazeExperimentArgs* experiment_args = new TurtleMazeExperimentArgs();
    experiment_args->num_steps = 1000;
    experiment_args->num_epochs = 200;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    TurtleMazeExperiment* exp = new TurtleMazeExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}
