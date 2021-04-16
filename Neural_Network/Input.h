#pragma once

class Input
{
private:
    double value;
    double* weight;

public:
    Input();
    //void set_weight(double val) { weight = val; }
    void set_value(double val) { value = val; }
    void out_nodes(int nb_hidden);
    double get_value() { return value; }
    double& get_weight(int index);

    ~Input();
};