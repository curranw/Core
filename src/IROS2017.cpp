#include <IROS2017.h>
void IROS2017::setup_experiments()
{
    //    normal_learning_turtle_stairs("Stairs_Test");
    for(unsigned int i = 0; i < 1; i++)
    {
        srand(time(NULL));
        normal_learning_arm_touch("Arm_Touch_Together");
        //pca_learning("filep.csv", -1, "Arm_Touch_PCA",6, true);
        //normal_learning_dual_arm_touch("Arm_Touch_Dual");
        //pca_learning_dual("converged_state_data_ArmTouch_bad_9.csv", -1, "Arm_Touch_Dual_PCA_Not_Norm", 3, true);
        //pca_learning_dual("converged_state_data_ArmTouch_good_3.csv", -1, "Arm_Touch_Dual_PCA_Not_Norm", 3, true);
    }
}
//TODO: Fix reward by making smaller.
//Add more tiles/more generalization?
//Longer steps, shorter episodes

void IROS2017::normal_learning_arm_touch(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.99;
    learning_args->eligability = false;
    learning_args->num_tiles = 32;

    vector<double> resolution;
    resolution.push_back(1.75);
    resolution.push_back(1.0);
    resolution.push_back(2.5);
    resolution.push_back(1.3);
    resolution.push_back(3.55);
    resolution.push_back(1.3);

    resolution.push_back(1.75);
    resolution.push_back(1.0);
    resolution.push_back(2.5);
    resolution.push_back(1.3);
    resolution.push_back(3.55);
    resolution.push_back(1.3);

    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    learning_args->resolution = resolution;

    ArmTouch* domain = new ArmTouch();

    domain->m_accumulate_data = true;
    ArmTouchExperimentArgs* experiment_args = new ArmTouchExperimentArgs();
    experiment_args->num_steps = 100;
    experiment_args->num_epochs = 500;
    experiment_args->save_file = save_file;
    QTiles* learning_algorithm = new QTiles(learning_args);

    ArmTouchExperiment* exp = new ArmTouchExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2017::normal_learning_dual_arm_touch(string save_file)
{
    QTilesArguments* learning_args = new QTilesArguments();
    learning_args->alpha = 0.1;
    learning_args->gamma = 0.1;
    learning_args->eligability = false;
    learning_args->num_tiles = 64;

    vector<double> resolution;
    for(unsigned int i = 0; i < 12; i++)
    {
        resolution.push_back(0.05);
    }

    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);
    //    resolution.push_back(0.05);

    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    //    resolution.push_back(0.25);
    learning_args->resolution = resolution;

    ArmTouch* domain = new ArmTouch();

    domain->m_accumulate_data = true;
    ArmTouchExperimentArgs* experiment_args = new ArmTouchExperimentArgs();
    experiment_args->num_steps = 100;
    experiment_args->num_epochs = 500;
    experiment_args->save_file = save_file;
    QTiles* learning_algorithm = new QTiles(learning_args);

    ArmTouchExperiment* exp = new ArmTouchExperiment(domain, learning_algorithm, experiment_args);
    exp->run_experiment();
    delete exp;
}

void IROS2017::pca_learning(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 6; i++)
    {
        QTilesArguments* learning_tiles_args = new QTilesArguments();
        learning_tiles_args->alpha = 0.1;
        learning_tiles_args->gamma = 0.1;
        learning_tiles_args->eligability = false;
        learning_tiles_args->num_tiles = 64;
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
    resolution.push_back(0.05);
    resolution.push_back(0.05);
    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(6, learning_args);


    ArmTouch* domain = new ArmTouch();

    ArmTouchExperimentPCAArgs* experiment_args = new ArmTouchExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 100;
    experiment_args->num_epochs = 2000;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    ArmTouchExperimentPCA* exp = new ArmTouchExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;

}

void IROS2017::pca_learning_dual(string pca_file, int amount, string save_file, int dim, bool iterative)
{
    QTilesReuseArgs* learning_args = new QTilesReuseArgs();
    for(int i = 0; i < 12; i++)
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




    resolution.push_back(1.75);
    resolution.push_back(1.0);
    resolution.push_back(2.5);
    resolution.push_back(1.3);
    resolution.push_back(3.55);
    resolution.push_back(1.3);

    resolution.push_back(1.75);
    resolution.push_back(1.0);
    resolution.push_back(2.5);
    resolution.push_back(1.3);
    resolution.push_back(3.55);
    resolution.push_back(1.3);



    learning_args->resolution = resolution;

    QTilesReuse* learning_algorithm = new QTilesReuse(12, learning_args);


    ArmTouch* domain = new ArmTouch();

    ArmTouchExperimentPCAArgs* experiment_args = new ArmTouchExperimentPCAArgs();
    experiment_args->demonstrations = false;
    experiment_args->num_steps = 100;
    experiment_args->num_epochs = 500;
    experiment_args->save_file = save_file;
    experiment_args->iterative = iterative;
    experiment_args->start_dimension = dim;

    ArmTouchExperimentPCA* exp = new ArmTouchExperimentPCA(domain, learning_algorithm, experiment_args);
    exp->run_experiment();

    delete exp;
}

