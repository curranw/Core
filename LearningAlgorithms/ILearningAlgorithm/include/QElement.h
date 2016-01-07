#pragma once
#ifndef QELEMENT_H
#define QELEMENT_H

#include <vector>
#include <unordered_map>
using namespace std;


class QElement
{
public:
    typedef vector<double> State;
    typedef int Action;
    QElement::State s;
    vector<QElement::Action> a;
    vector<double> v;
};

class QUpdate
{
public:
    double reward;
    QElement::State state, next_state;
    QElement::Action action;
    unordered_map<int, double> next_state_action_values;
    double old_value;
    vector<QElement*> states_to_update;
    vector<double> old_probabilities;
};


#endif
