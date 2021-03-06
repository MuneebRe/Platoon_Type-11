#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

#include "NeuroNet.h"
#include "Input.h"
#include "Hidden.h"
#include "Output.h"
#include <cstdlib>

Neural_Net::Neural_Net(int nb_input, int nb_hidden, int nb_output)
{
    this->nb_input = nb_input;
    input = new Input[nb_input];

    for (int i = 0; i < nb_input; i++)
    {
        input[i].out_nodes(nb_hidden);
    }

    this->nb_hidden = nb_hidden;
    hidden = new Hidden[nb_hidden];

    for (int i = 0; i < nb_hidden; i++)
    {
        hidden[i].out_nodes(nb_output);
    }

    this->nb_output = nb_output;
    output = new Output[nb_output];

}

void Neural_Net::bias()
{
    input[nb_input - 1].set_value(1.0);
    input[nb_input - 1].get_weight(nb_hidden - 1) = 0;
    hidden[nb_hidden - 1].set_value(1.0);
    //hidden[nb_hidden - 1].get_weight(nb_output - 1) = 0;
}

void Neural_Net::calculate_hidden()
{
    if(is_bias == 1) bias();

    for (int i = 0; i < nb_hidden; i++)
    {
        for (int j = 0; j < nb_input; j++)
        {
            hidden[i].get_value() += input[j].get_value() * input[j].get_weight(i);
        }
        hidden[i].sigmoid();
    }
}

void Neural_Net::calculate_output()
{
    if (is_bias == 1) bias();

    for (int i = 0; i < nb_output; i++)
    {
        for (int j = 0; j < nb_hidden; j++)
        {
            output[i].get_value() += hidden[j].get_value() * hidden[j].get_weight(i);
        }
        output[i].sigmoid();
    }
}

void Neural_Net::print_inputs()
{
    for (int i = 0; i < nb_input; i++)
    {
        cout << input[i].get_value() << endl;
    }
}

void Neural_Net::print_hidden()
{
    for (int i = 0; i < nb_hidden; i++)
    {
        cout << hidden[i].get_value() << endl;
    }
}

void Neural_Net::print_output()
{
    for (int i = 0; i < nb_output; i++)
    {
        cout << output[i].get_value() << endl;
    }
}

void Neural_Net::save_weights()
{

    ofstream fout;
    
    char string[50] = "Fitness_Logs/trial00.txt";

    for (int i = 0; i < 50; i++)
    {
        if (string[i] == '\0')
        {   
            //string[i - 5] = int_to_char(trial_number);

            zero_to_hundred(trial_number, string[i - 5], string[i - 6]);

            break;
        }
    }

    //cout << string << endl;
    fout.open(string);
    //fout.open("Neural_Network/Fitness_Logs/trial.txt");
    
    fout << trial_number << endl;

    fout << fitness_number << endl;
    
    fout << fixed;
    fout << setprecision(4);
    
    for (int i = 0; i < nb_hidden; i++)
    {
        for (int j = 0; j < nb_input; j++)
        {
            fout << input[j].get_weight(i) << endl;
        }
    }

    for (int i = 0; i < nb_output; i++)
    {
        for (int j = 0; j < nb_hidden; j++)
        {
            fout << hidden[j].get_weight(i) << endl;
        }
    }

    if (is_bias == 1) bias();

    fout.close();
}

void Neural_Net::randomize_weights()
{
    //REF1-6
    always_reset_fitness = 0;
    
    bool flag0 = 0;     //flag0 is just to see if all the weights are initialized to zero
    bool flag1 = 0;     //flag1 is if you want randomization to occur relative to the weights stored in best.txt
    bool flag2 = 1;     //flag2 is if you want to test fully randomly unrelated to best.txt

    double _rando0 = 0.15;  //Good for resetting all weights, kind of
    double _rando1 = 0.30;  //if range desired is -0.25 to 0.25, just write 0.25. weights will add to weight recorded by best.txt with a value between [-0.25, +0.25] Useful for tuning species.
    double _rando2 = 1.00;  //If value 0.25, Weight will pick a value between[-0.25, +0.25].Useful to generate different species.
    
    double limit = 1.0000;     //Limits the weighting so it's kept between [1.0 - 1.0]. Also, bias( ) is included, if you know what I mean.

    int rando1 = _rando1 * 10000 * 2;     //Double to int convert
    int rando2 = _rando2 * 10000 * 2;     //Double to int convert 

    for (int i = 0; i < nb_hidden; i++)
    {
        for (int j = 0; j < nb_input; j++)
        {
            while (1)
            {
                if (flag0 == 1) input[j].get_weight(i) = _rando0;
                if (flag1 == 1) input[j].get_weight(i) = input[j].get_weight(i) + (double)(((rand() % rando1) - (rando1 / 2)) / 10000.000);
                if (flag2 == 1) input[j].get_weight(i) = (double)(((rand() % rando2) - (rando2 / 2)) / 10000.000);
                
                if (input[j].get_weight(i) < -limit) input[j].get_weight(i) = -limit;
                if (input[j].get_weight(i) > limit) input[j].get_weight(i) = limit;

                if (input[j].get_weight(i) < limit && input[j].get_weight(i) > -limit) break;
            }
        }
    }

    for (int i = 0; i < nb_output; i++)
    {
        for (int j = 0; j < nb_hidden; j++)
        {
            while ( 1 )
            {
                if (flag0 == 1) hidden[j].get_weight(i) = _rando0;
                if (flag1 == 1) hidden[j].get_weight(i) = hidden[j].get_weight(i) + (double)(((rand() % rando1) - (rando1 / 2)) / 10000.0);
                if (flag2 == 1) hidden[j].get_weight(i) = (double)(((rand() % rando2) - (rando2 / 2)) / 10000.0);
                
                if (hidden[j].get_weight(i) < -limit) hidden[j].get_weight(i) = -limit;
                if (hidden[j].get_weight(i) > limit) hidden[j].get_weight(i) = limit;
               
                if (hidden[j].get_weight(i) < limit && hidden[j].get_weight(i) > -limit) break;
            } 
        }
    }
    //bias();

}

void Neural_Net::randomize_just_one_weight()
{
    //REF1-6
    always_reset_fitness = 0;

    bool flag0 = 0;     //flag0 is just to see if all the weights are initialized to zero
    bool flag1 = 0;     //flag1 is if you want randomization to occur relative to the weights stored in best.txt
    bool flag2 = 1;     //flag2 is if you want to test fully randomly unrelated to best.txt

    double _rando0 = 0.00;  //Good for resetting all weights, kind of
    double _rando1 = 0.10;  //if range desired is -0.25 to 0.25, just write 0.25. weights will add to weight recorded by best.txt with a value between [-0.25, +0.25] Useful for tuning species.
    double _rando2 = 1.00;  //If value 0.25, Weight will pick a value between[-0.25, +0.25].Useful to generate different species.

    double limit = 1.00;     //Limits the weighting so it's kept between [1.0 - 1.0]. Also, bias( ) is included, if you know what I mean.

    int rando1 = _rando1 * 10000 * 2;     //Double to int convert
    int rando2 = _rando2 * 10000 * 2;     //Double to int convert

    int layer_selection = rand() % 2;

    if (layer_selection == 0)
    {
        int select_input;
        int select_weight;


            select_input = rand() & (nb_input - 1);
            select_weight = rand() & (nb_hidden-1);


        cout << "Selected input:  " << select_input << "  " << "Selected weight :" << select_weight << endl;

        while (1)
        {
            if (flag0 == 1) input[select_input].get_weight(select_weight) = _rando0;
            if (flag1 == 1) input[select_input].get_weight(select_weight) = input[select_input].get_weight(select_weight) + (double)(((rand() % rando1) - (rando1 / 2)) / 10000.0);
            if (flag2 == 1) input[select_input].get_weight(select_weight) = (double)(((rand() % rando2) - (rando2 / 2)) / 10000.0);

            if (input[select_input].get_weight(select_weight) < -limit) input[select_input].get_weight(select_weight) = -limit;
            if (input[select_input].get_weight(select_weight) > limit) input[select_input].get_weight(select_weight) = limit;

            if (input[select_input].get_weight(select_weight) < limit && input[select_input].get_weight(select_weight) > -limit) break;
        }
    }
    
    if (layer_selection == 1)
    {
        int select_hidden;
        int select_weight;


            select_hidden = rand() & (nb_hidden - 1);
            select_weight = rand() & (nb_output - 1);


        cout << "Selected hidden: " << select_hidden << "  " << "Selected weight: " << select_weight << endl;

        while (1)
        {
            if (flag0 == 1) hidden[select_hidden].get_weight(select_weight) = _rando0;
            if (flag1 == 1) hidden[select_hidden].get_weight(select_weight) = hidden[select_hidden].get_weight(select_weight) + (double)(((rand() % rando1) - (rando1 / 2)) / 10000.0);
            if (flag2 == 1) hidden[select_hidden].get_weight(select_weight) = (double)(((rand() % rando2) - (rando2 / 2)) / 10000.0);

            if (hidden[select_hidden].get_weight(select_weight) < -limit) hidden[select_hidden].get_weight(select_weight) = -limit;
            if (hidden[select_hidden].get_weight(select_weight) > limit) hidden[select_hidden].get_weight(select_weight) = limit;

            if (hidden[select_hidden].get_weight(select_weight) < limit && hidden[select_hidden].get_weight(select_weight) > -limit) break;
        }
    }

    //bias();

}

void Neural_Net::find_best()
{

    int trial_number_remember = 0;
    int fitness_number_max = 0;
    int *fitness_mem = new int[trial_number_limit];
    int *trial_mem = new int[trial_number_limit];
    //int fitness_mem[10];
    //int trial_mem[10];

    ifstream fin;
    ofstream fout;

    fin.open("Fitness_Logs/top_fitness.txt");
    fin >> fitness_number_max;

    if (always_reset_fitness == 1)
    {
        fitness_number_max = 0; //Helpful if species is not improving, useful in initial stages
    }

    fin.close();

    char string[50] = "Fitness_Logs/trial00.txt";

    for (int i = 0; i < trial_number_limit; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            if (string[j] == '\0')
            {
                //string[i - 5] = int_to_char(trial_mem[trial_number_remember]);
                zero_to_hundred(i, string[j - 5], string[j - 6]);
                break;
            }
        }

        fin.open(string);
        fin >> trial_number;
        fin >> fitness_number;
        fin.close();

        cout << "TRIAL " << trial_number << " -> Fitness: " << fitness_number << endl;

        //if (fitness_number >= fitness_number_max) trial_number_remember = trial_number;
        trial_mem[i] = trial_number;
        fitness_mem[i] = fitness_number;
        //cout << "Trial number recrod = " << trial_number_remember << endl;
    }

    for (int i = 0; i < trial_number_limit; i++)
    {
        //cout << trial_mem[i] << "\t" << fitness_mem[i] << endl;
        if (fitness_mem[i] > fitness_number_max)
        {
            fitness_number_max = fitness_mem[i];
            trial_number_remember = i;

            fout.open("Fitness_Logs/top_fitness.txt");
            fout << fitness_mem[trial_number_remember] << endl;
            fout.close();

            for (int i = 0; i < 50; i++)
            {
                if (string[i] == '\0')
                {
                    //string[i - 5] = int_to_char(trial_mem[trial_number_remember]);
                    zero_to_hundred(trial_mem[trial_number_remember], string[i - 5], string[i - 6]);
                    break;
                }
            }

            //cout << "BEST RECORD AT" << endl;
            //cout << "TRIAL: " << trial_number_remember << '\t' << "FITNESS: " << fitness_number << endl;

            ifstream inFile(string);

            ofstream outFile("Fitness_Logs/best.txt");

            outFile << inFile.rdbuf();
        }
    }

    cout << "BEST TRIAL " << trial_mem[trial_number_remember] << " -> Fitness: " << fitness_mem[trial_number_remember] << endl;

    delete[]fitness_mem;
    delete[]trial_mem;

    
    fin.open("Fitness_Logs/generation.txt");
    int generation;
    fin >> generation;
    fin.close();
    generation++;
    
    
    fout.open("Fitness_Logs/generation.txt");
    fout << generation;
    fout.close();
}

void Neural_Net::set_trial_number(int trial_number)
{
    this->trial_number = trial_number;
    cout << "Trial: " << trial_number << " saved!" << endl;
}

void Neural_Net::set_finess_number(int fitness_number)
{
    this->fitness_number = fitness_number;
    cout << fitness_number << endl;
}

void Neural_Net::load_best()
{
    ifstream fin;

    char string[50] = "Fitness_Logs/trial00.txt";

    for (int i = 0; i < 50; i++)
    {
        if (string[i] == '\0')
        {
            //string[i - 5] = int_to_char(trial_number);
            zero_to_hundred(trial_number, string[i - 5], string[i - 6]);
            break;
        }
    }

    //cout << string << endl;
    fin.open("Fitness_Logs/best.txt");
    //fout.open("Neural_Network/Fitness_Logs/trial.txt");

    fin >> trial_number;

    fin >> fitness_number;


    for (int i = 0; i < nb_hidden; i++)
    {
        for (int j = 0; j < nb_input; j++)
        {
            fin >> input[j].get_weight(i);
        }
    }

    for (int i = 0; i < nb_output; i++)
    {
        for (int j = 0; j < nb_hidden; j++)
        {
            fin >> hidden[j].get_weight(i);
        }
    }


    fin.close();
}

char Neural_Net::int_to_char(int number)
{
    char character;
    switch (number)
    {
    case 0:
        character = '0';
        break;
    case 1:
        character = '1';
        break;
    case 2:
        character = '2';
        break;
    case 3:
        character = '3';
        break;
    case 4:
        character = '4';
        break;
    case 5:
        character = '5';
        break;
    case 6:
        character = '6';
        break;
    case 7:
        character = '7';
        break;
    case 8:
        character = '8';
        break;
    case 9:
        character = '9';
        break;
    }
    return character;
}

void Neural_Net::zero_to_hundred(int number, char& index1, char& index2)
{
    int digit1 = number % 10;
    int digit2 = number / 10 % 10;

    //cout << "FIRST DIGIT IS " << int_to_char(digit1) << endl;
    //cout << "SECON DIGIT IS " << int_to_char(digit2) << endl;
    
    index1 = int_to_char(digit1);
    index2 = int_to_char(digit2);
}