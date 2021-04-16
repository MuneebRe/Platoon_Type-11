#include <iostream>
using namespace std;

#include "NeuroNet.h"
#include "Input.h"
#include "Hidden.h"
#include "Output.h"

int main()
{
    Neural_Net topology(4, 3, 1);

    topology.input[0].set_value(0.25);
    topology.input[1].set_value(0.50);
    topology.input[2].set_value(0.65);

    topology.randomize_weights();
    topology.calculate_hidden();
    topology.calculate_output();
    topology.print_inputs();
    topology.print_hidden();
    topology.print_output();
    topology.save_weights();
}





