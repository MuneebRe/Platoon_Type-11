#pragma once

#include "Input.h"
#include "Hidden.h"
#include "Output.h"

class Neural_Net
{
private:
    int nb_input;
    int nb_hidden;
    int nb_output;
    int trial_number;
    int fitness_number;
    int generation;
public:
    Input* input;
    Hidden* hidden;
    Output* output;

    Neural_Net(int nb_input, int nb_hidden, int nb_output);
    void calculate_hidden();
    void calculate_output();
    void print_inputs();
    void print_hidden();
    void print_output();
    void bias();
    void save_weights();
    void randomize_weights();
    void randomize_weights_again();
    void set_trial_number(int trial_number);
    void set_finess_number(int fitness_number);
    void find_best();
    void load_best();
    char int_to_char(int number);
};