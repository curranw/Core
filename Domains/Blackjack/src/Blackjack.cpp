#include <Blackjack.h>
#include <time.h>
Blackjack::Blackjack()
{
    srand(time(NULL));
    num_cards = 52;
    //Number Cards and A
    for(unsigned int i = 0; i < 4; i++)
    {
        for(unsigned int j = 1; j < 11; j++)
        {
            deck.push_back(j);
        }
    }
    //Face Cards
    for(unsigned int i = 0; i < 12; i++)
    {
        deck.push_back(10);
    }
    cout << deck.size() << endl;
    num_cards = deck.size();

}

void Blackjack::init()
{
    done = false;
    usable_ace = false;
    draw_cards(0);
    draw_cards(0);
    draw_cards(1);

    player_value = calculate_hand_value(0);
    dealer_value = calculate_hand_value(1);
    games_won = 0;
}

void Blackjack::step(int action)
{

    vector<double> a = m_action_mapping[action];
    //Stay
    if(a[0] == 0)
    {
        //cout << "Stay" << endl;
        done = true;
    }
    //Hit
    if(a[0] == 1)
    {
        //cout << "Hit" << endl;
        draw_cards(0);
        player_value = calculate_hand_value(0);
    }
}


bool Blackjack::end_of_episode()
{

    if(deck.empty())
    {
        deck.clear();
        //cout << "Deck Small " << endl;
        for(unsigned int i = 0; i < 4; i++)
        {
            for(unsigned int j = 1; j < 11; j++)
            {
                deck.push_back(j);
            }
        }
        //Face Cards
        for(unsigned int i = 0; i < 12; i++)
        {
            deck.push_back(10);
        }
        //cout << games_won << endl;
        //cout << "Deck Size: " << deck.size() << endl;
        player_hand.clear();
        dealer_hand.clear();
        return true;
    }
    return false;
}


vector<double> Blackjack::get_state()
{

    for(unsigned int i = 0; i < player_hand.size(); i++)
    {
        //cout << player_hand[i] << endl;
    }
    //cout << endl;

    for(unsigned int i = 0; i < dealer_hand.size(); i++)
    {
        //cout << dealer_hand[i] << endl;
    }
    //cout << endl;
    vector<double> state;
    state.push_back(player_value);
    state.push_back(dealer_value);

    state.push_back(usable_ace);
        vector<int> values(10);
        for(unsigned int i = 0; i < deck.size(); i++)
        {
            values[deck[i]-1]++;
        }

        state.push_back(values[0]);
        state.push_back(values[1] + values[2] + values[3] + values[4]);
        state.push_back(values[5] + values[6] + values[7] + values[8]);
        state.push_back(values[9]);

        //cout << values[9] << endl;
        //state.push_back(values[0]);
        for(unsigned int i = 0; i < values.size(); i++)
        {
            //state.push_back(values[i]);
        }
    //cout << "Deck Size: " << deck.size() << endl;
    return state;
}


vector<double> Blackjack::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(2);
    min_ranges.push_back(1);
    min_ranges.push_back(0);
//        //A & Number Cards
//        for(unsigned int i = 0; i < 9; i++)
//        {
//            min_ranges.push_back(0);
//        }
        //10/Face Card
        min_ranges.push_back(0);
        min_ranges.push_back(0);
        min_ranges.push_back(0);
        min_ranges.push_back(0);

        //min_ranges.push_back(0);
    return min_ranges;
}


vector<double> Blackjack::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(31);
    max_ranges.push_back(11);
    max_ranges.push_back(1);
    //A & Number Cards

//        for(unsigned int i = 0; i < 9; i++)
//        {
//            max_ranges.push_back(4);
//        }
        //10/Face Card
        //max_ranges.push_back(4);
    max_ranges.push_back(16);
    max_ranges.push_back(16);
    max_ranges.push_back(16);
    max_ranges.push_back(16);
    return max_ranges;

}


double Blackjack::get_reward()
{
    if(deck.empty()) return 0;
    if(player_value > 21) done = true;
    if(!done) return 0;
    double reward;
    while( dealer_value < 17)
    {
        if(deck.empty()) return 0;
        draw_cards(1);

        //        for(unsigned int i = 0; i < dealer_hand.size(); i++)
        //        {
        //            cout << dealer_hand[i] << endl;
        //        }
        //        cout << endl;
        dealer_value = calculate_hand_value(1);
    }

    //cout << player_value << endl;
    //cout << dealer_value << endl;

    if(player_value > dealer_value || dealer_value > 21) reward = 1;
    if(player_value == dealer_value) reward = 0;
    if(player_value > 21 || (dealer_value > player_value && dealer_value <= 21)) reward = -1;
    if(player_value > 21 && dealer_value > 21) reward = 0;

    player_hand.clear();
    dealer_hand.clear();
    done = false;
    draw_cards(0);
    draw_cards(0);
    draw_cards(1);

    player_value = calculate_hand_value(0);
    dealer_value = calculate_hand_value(1);

    if(reward == 1) games_won++;
    //cout << "Reward: " << reward << endl;
    return reward;
}

void Blackjack::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    vector<double> action;
    int it = 0;
    action.push_back(0);
    possible_actions.push_back(it);
    action_mapping.insert(make_pair(it++, action));

    action.clear();
    action.push_back(1);
    possible_actions.push_back(it);
    action_mapping.insert(make_pair(it++, action));

    set_possible_actions(possible_actions, action_mapping);
}


Blackjack::~Blackjack()
{
}

double Blackjack::calculate_hand_value(int player)
{
    vector<int> hand;
    if(player == 0) hand = player_hand;
    if(player == 1) hand = dealer_hand;
    int hand_val = 0;
    int num_aces = 0;
    for(unsigned int i = 0; i < hand.size(); i++)
    {
        if(hand[i] == 1) num_aces++;
        else hand_val += hand[i];
    }
    usable_ace = false;
    // cout << "NUM ACES: " << num_aces << endl;
    for(unsigned int i = 0; i < num_aces; i++)
    {
        if(hand_val + 11 > 21)
        {
            hand_val += 1;
        }
        else
        {
            if(player == 0) usable_ace = true;
            hand_val += 11;
        }
    }
    return hand_val;
}

void Blackjack::draw_cards(int player)
{
    if(deck.empty()) return;
    int rand_card = rand() % deck.size();
    int card = deck.at(rand_card);
    std::vector<int>::iterator position = (std::find(deck.begin(), deck.end(), card));
    if (position != deck.end())
    {
        deck.erase(position);
    }
    if(player == 0)
    {
        player_hand.push_back(card);
    }
    if(player == 1)
    {
        dealer_hand.push_back(card);
    }
}


