#include "Hidden.h"

#include <iostream>
using namespace std;

Hidden::Hidden()
{
    value = 0;
}

void Hidden::out_nodes(int nb_output)
{
    weight = new double[nb_output];

    for (int i = 0; i < nb_output; i++)
    {
        weight[i] = 1.0;
    }

}

double& Hidden::get_weight(int index)
{
    return weight[index];
}

void Hidden::sigmoid()
{
    value = 1.0 / (1.0 + exp(-value));
}