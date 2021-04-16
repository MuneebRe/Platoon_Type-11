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
    bias();

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
    bias();

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
    
    char string[50] = "Fitness_Logs/trial0.txt";

    for (int i = 0; i < 50; i++)
    {
        if (string[i] == '\0')
        {   
            string[i - 5] = int_to_char(trial_number);;
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


    fout.close();
}

void Neural_Net::randomize_weights()
{
    for (int i = 0; i < nb_hidden; i++)
    {
        for (int j = 0; j < nb_input; j++)
        {
            input[j].get_weight(i) = (double)(((rand() % 300)-150)/100.0);
        }
    }

    for (int i = 0; i < nb_output; i++)
    {
        for (int j = 0; j < nb_hidden; j++)
        {
            hidden[j].get_weight(i) = (double)(((rand() % 300) - 150) / 100.0);
        }
    }
    bias();
}

void Neural_Net::find_best()
{
    int trial_number_remember = 0;
    int fitness_number_max = 0;

    ifstream fin;

    char string[50] = "Fitness_Logs/trial0.txt";

    for (int i = 0; i < 10; i++)
    {
        for (int i = 0; i < 50; i++)
        {
            if (string[i] == '\0')
            {
                string[i - 5] = int_to_char(i);
                break;
            }
        }

        fin.open(string);

        int temp;
        //fin >> generation;
        fin >> trial_number;
        fin >> fitness_number;

        if (fitness_number > fitness_number_max)
        {
            fitness_number_max = fitness_number;
            trial_number_remember = trial_number;
        }
    }

    fin.close();

    for (int i = 0; i < 50; i++)
    {
        if (string[i] == '\0')
        {
            string[i - 5] = int_to_char(trial_number_remember);
            break;
        }
    }

    ifstream inFile(string);

    ofstream outFile("Fitness_Logs/best.txt");
    
    outFile << inFile.rdbuf();

    

    
    
}

void Neural_Net::randomize_weights_again()
{
    for (int i = 0; i < nb_hidden; i++)
    {
        for (int j = 0; j < nb_input; j++)
        {
            input[j].get_weight(i) = (double)((rand() % 300) - 150) / 100;
        }
    }
    
    for (int i = 0; i < nb_output; i++)
    {
        for (int j = 0; j < nb_hidden; j++)
        {
            hidden[j].get_weight(i) = (double)((rand() % 300) - 150) / 100;
        }
    }
    bias();
}

void Neural_Net::set_trial_number(int trial_number)
{
    this->trial_number = trial_number;
    cout << "Trial: " << trial_number << endl;
}

void Neural_Net::set_finess_number(int fitness_number)
{
    this->fitness_number = fitness_number;
    cout << fitness_number << endl;
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
        character = '8';
        break;
    }
    return character;
}