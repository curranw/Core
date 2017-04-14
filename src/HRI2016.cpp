#include <HRI2016.h>
void HRI2016::setup_experiments()
{
//    normal_learning_turtle_stairs("Stairs_Test");
    for(unsigned int i = 0; i < 20; i++)
    {
        srand(time(NULL));
        normal_learning_turtle_maze("Maze_Rewarded");
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

void HRI2016::normal_learning_turtle_stairs(string save_file)
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

    TurtleStairs* domain = new TurtleStairs();

    TurtleStairsExperimentArgs* experiment_args = new TurtleStairsExperimentArgs();
    experiment_args->num_steps = 1000;
    experiment_args->num_epochs = 200;
    experiment_args->save_file = save_file;

    QTiles* learning_algorithm = new QTiles(learning_args);

    TurtleStairsExperiment* exp = new TurtleStairsExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}
