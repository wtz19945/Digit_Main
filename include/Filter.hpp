#ifndef FILTER_H
#define FILTER_H

#include <queue>
#include <assert.h> 
class MovingAverageFilter{
public:
    MovingAverageFilter(int window_sz);
    MovingAverageFilter() : MovingAverageFilter(3){};
    double getData(double new_data); 

private:    
    std::queue<double> data;
    int period;
    double total;
};

class FirstOrderFilter{
public:
    FirstOrderFilter(double value_0, double param) : prev_value(value_0), param(param){assert(param>=0 && param <=1);};
    FirstOrderFilter() : FirstOrderFilter(0,1){};
    double getData(double new_data); 

private:
    double prev_value;
    double param;
};

#endif //FILTER_H

