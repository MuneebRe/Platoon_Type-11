#pragma once
#include "Input.h"

class Hidden
{
private:
    double value;
    int nb_hidden;
    double* weight;

public:
    Hidden();
    double& get_value() { return value; }
    void set_value(double val) { value = val; }
    void calculate_from_input(Input input[], int nb_input, int nb_hidden);
    void out_nodes(int nb_output);
    double& get_weight(int index);
    void sigmoid();
};