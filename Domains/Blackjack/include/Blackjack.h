#pragma once

#include <cmath>
#include <vector>
#include <iostream>
#include <IDomain.h>
#include <algorithm>
using namespace std;
class Blackjack : public IDomain{
public:
    Blackjack();
    void init();
    void step(int action);
    bool end_of_episode();
    vector<double> get_state();
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    double get_reward();
    void compute_possible_actions();

    virtual ~Blackjack();

private:
    double calculate_hand_value(int player);
    void draw_cards(int player);
    vector<int> deck;
    int num_cards;
    bool done;
    vector<int> player_hand;
    vector<int> dealer_hand;
    int player_value;
    int dealer_value;
    int games_won;
    bool usable_ace;
};
