#pragma once

#include "Input.h"
#include "Hidden.h"
#include "Output.h"

class Neural_Net
{
private:
    int nb_input;
    int nb_hidden;
    int nb_hidden2;
    int nb_output;
    int trial_number;
    int fitness_number;
    int generation;
    bool always_reset_fitness;
    int trial_number_limit;
    bool is_bias;

public:
    Input* input;
    Hidden* hidden;
    Hidden* hidden2;
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
    void randomize_just_one_weight();
    void randomize_weights_again();
    void set_trial_number(int trial_number);
    void set_finess_number(int fitness_number);
    void find_best();
    void load_best();
    char int_to_char(int number);
    void zero_to_hundred(int number, char& index1, char& index2);
    void set_trial_nb_limit(int trial_number_limit) { this->trial_number_limit = trial_number_limit; }
    int get_trial_nb_limit() { return trial_number_limit; }
    void set_bias(bool bias) { is_bias = bias; }
};