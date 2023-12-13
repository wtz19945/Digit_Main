#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <unistd.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <memory>
#include <Eigen/Dense>
#include <string>
typedef enum
{
    st = 0,
    st2sw = 1,
    sw = 2
} digit_state;


class State_Machine{
    public:
    State_Machine();
    void Update_state(double time, double step_time, int cmd);

    private:
    static digit_state state;
    int contact1;
    int contact2;
};

#endif //SAFETY_MANAGER_H_

