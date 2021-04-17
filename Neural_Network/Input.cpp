#include "Input.h"

#include <iostream>

using namespace std;

Input::Input()
{
    value = 0;
}

void Input::out_nodes(int nb_hidden)
{
    weight = new double[nb_hidden];

    for (int i = 0; i < nb_hidden; i++)
    {
        weight[i] = 0.0;
    }
    
}

double& Input::get_weight(int index)
{
    return weight[index];
}

Input::~Input()
{
    delete[] weight;
}