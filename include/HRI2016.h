#pragma once
#include <TurtleMazeExperiment.h>
#include <TurtleMaze.h>
#include <TurtleStairsExperiment.h>
#include <TurtleStairs.h>
class HRI2016
{
public:
    void setup_experiments();
    void normal_learning_turtle_maze(string save_file);
    void normal_learning_turtle_stairs(string save_file);
};
