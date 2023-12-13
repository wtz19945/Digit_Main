#include "State_Machine.hpp"

State_Machine::State_Machine(){
    state = st;
    contact1 = 1;
    contact2 = 1;
}

void State_Machine::Update_state(double time, double step_time, string cmd){
    switch(state){
        case st:
            if(cmd == "standing")
                state = st;
            if(cmd == "stepping")
                state = st2sw;
            break;
        case st2sw:
            
        case sw:
            state = sw;
    }
}

