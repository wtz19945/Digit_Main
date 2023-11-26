#include "Filter.hpp"

MovingAverageFilter::MovingAverageFilter(int window_sz) : 
    period(window_sz),
    total(0) {
        assert(period >= 1);
}

double MovingAverageFilter::getData(double new_data){
    if(data.size() == period){
        total -= data.front();
        data.pop();
    }

    data.push(new_data);
    total += new_data;

    if(data.size() == 0){
        return 0;
    }
    else{
        return total / (double) data.size();
    }
}

double FirstOrderFilter::getData(double new_data){
    double result = param * new_data + (1 - param) * prev_value;
    prev_value = new_data;
    return result;
}



