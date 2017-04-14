#include <Swimmers.h>

Swimmers::Swimmers(int segments)
{
    swimmer = new CSwimmer(segments);
    swimmerwin = new CXSwimmer(*swimmer, *this);
    viz = false;
    //swimmerwin->Animate();
}

void Swimmers::init()
{
    last_v = 0;
    swimmer->reset();
    swimmerwin->Reset();
}

vector<double> Swimmers::get_state()
{
    int num_segments = swimmer->GetSegments();
    vector<double> state;
    CVector swim_state = swimmerwin->GetState();
    state.push_back(swim_state[0]);
    state.push_back(swim_state[1]);
    for(int i = 0; i < num_segments; i++)
    {
        double theta = swim_state[i*2+2];
        double v = swim_state[i*2+3];
        state.push_back(theta);
        state.push_back(v);
    }
    return state;
}

void Swimmers::step(int action)
{
    //Set Action
    action_vec = m_action_mapping[action];
    swimmerwin->Step();
    if(viz)
    {
        static int i = 0;
        if(i == 0)
        {
            swimmerwin->xanim.Events();
            swimmerwin->xanim.StartTime();
            swimmerwin->xanim.ResetStopFlag();
        }
        cout << i++ << endl;
        swimmerwin->Draw();
        swimmerwin->xanim.Events();
        swimmerwin->xanim.NewFrame();
    }
}

vector<double> Swimmers::get_min_ranges()
{
    int num_segments = swimmer->GetSegments();
    vector<double> min_ranges;
    min_ranges.push_back(-1);
    min_ranges.push_back(-1);
    for(int i = 0; i < num_segments; i++)
    {
        min_ranges.push_back(-3.2);
        min_ranges.push_back(-8);
        //min_ranges.push_back(-8);
    }
    return min_ranges;
}

vector<double> Swimmers::get_max_ranges()
{
    int num_segments = swimmer->GetSegments();
    vector<double> max_ranges;
    max_ranges.push_back(1);
    max_ranges.push_back(1);
    for(int i = 0; i < num_segments; i++)
    {
        max_ranges.push_back(3.2);
        max_ranges.push_back(8);
        //max_ranges.push_back(8);
    }
    return max_ranges;
}

bool Swimmers::end_of_episode()
{
    return false;
}

double Swimmers::get_reward()
{
    CVector swim_state = swimmerwin->GetState();
    return swim_state[0];
}

double Swimmers::get_potential()
{
    CVector swim_state = swimmerwin->GetState();
    double cur_v = swim_state[0];
    double accel = cur_v - last_v;
    last_v = swim_state[0];
    return accel;
}

void Swimmers::compute_possible_actions()
{
    int num_dimensions = swimmer->GetControlDimension();
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
//    vector<double> a;
//    a.push_back(-3);
//    a.push_back(0);
//    a.push_back(3);
//    vector<int> totals(num_dimensions);
//    std::fill(totals.begin(), totals.end(), 0);

//    int cur_a = 0;
//    while(true)
//    {
//        vector<double> possible_a;
//        for(unsigned int i = 0; i < num_dimensions; i++)
//        {
//            possible_a.push_back(a[totals[i]]);
//        }

//        totals.back()++;
//        for(int cur_it = totals.size()-1; cur_it >= 0; cur_it--)
//        {
//            if(totals[cur_it] > a.size()-1 && cur_it != 0)
//            {
//                totals[cur_it] = 0;
//                totals[cur_it-1]++;
//            }
//        }
//        possible_actions.push_back(cur_a);
//        action_mapping.insert(make_pair(cur_a++, possible_a));
//        if(totals[0] > a.size()-1) break;
//    }
//    set_possible_actions(possible_actions, action_mapping);

        vector<double> a;
        int vsize = num_dimensions;
        //    for (int i=0; i<vsize*2; i++){
        for (int i=0; i<vsize*2; i++){
            double val = 3;
            possible_actions.push_back(i);
            a = std::vector<double>(vsize,0);
            a[i/2] = val*(pow(-1.0, i));

            action_mapping.insert(make_pair(i, a));
            a.clear();
        }
        possible_actions.push_back(vsize*2);
        a.assign(vsize,0);
        action_mapping.insert(make_pair(vsize*2, a));
        set_possible_actions(possible_actions, action_mapping);
}




//This should be step
void Swimmers::GetControl(const double *px, double *pu) const
{
    for (int i = swimmer->GetControlDimension(); --i >= 0;)
        pu[i] = action_vec[i];
    //pu[i] = (i & 1) ? 0 : swimmer->GetUMax(i);
}

Swimmers::~Swimmers()
{
    delete swimmer;
    delete swimmerwin;
}

