#include "NeuralNet.h"

#include <cmath>
#include <iostream>

using namespace std;

void toTerminal();

void toTerminal2(bool net_mem[], double net_out[]);

//Original code from
//http://robotics.hobbizine.com/arduinoann.html
//I converted that Arduino code into something we could use.
//Defenitely did not make it, I would have if we knew how long we had for this project.

/******************************************************************
 * Network Configuration - customized per network
 ******************************************************************/

const int PatternCount = 17;
const int InputNodes = 7;
const int HiddenNodes = 25;
const int OutputNodes = 3;
const float LearningRate = 0.007;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;


const short int Input[PatternCount][InputNodes] = {
  { 0, 0, 0, 0, 0, 1, 0 },   // Enemy to your right
  { 0, 0, 0, 0, 1, 0, 0 },   // Enemy to your left
  { 0, 0, 0, 0, 0, 0, 1 },    // Enemy Detected
  { 1, 0, 0, 0, 1, 0, 0 },    // Front Collision - Enemy to the left
  { 1, 0, 0, 0, 0, 1, 0 },    // Front Collision - Enemy to the right
  { 0, 0, 0, 1, 1, 0, 0 },    // Left Collision - Enemy to the left
  { 0, 0, 0, 1, 0, 1, 0 },    // Left Collision - Enemy to the right
  { 1, 0, 0, 1, 1, 0, 0 },    // Front Left Collision - Enemy to the left
  { 1, 0, 0, 1, 0, 1, 0 },    // Front Left Collision - Enemy to the right
  { 0, 1, 0, 0, 1, 0, 0 },    // Right Collision - Enemy to the left
  { 0, 1, 0, 1, 0, 1, 0 },    // Right Collision - Enemy to the right
  { 1, 1, 0, 0, 1, 0, 0 },    // Front Right Collision - Enemy to the left
  { 1, 1, 0, 0, 0, 1, 0 },    // Front Right Collision - Enemy to the right
  { 0, 0, 1, 1, 1, 0, 0 },    // Back Left Collision - Enemy to the left
  { 0, 0, 1, 1, 0, 1, 0 },    // Back Left Collision - Enemy to the right
  { 0, 1, 1, 0, 1, 0, 0 },    // Back Right Collision - Enemy to the left
  { 0, 1, 1, 0, 0, 1, 0 }     // Back Right Collision - Enemy to the right
};

const short int Target[PatternCount][OutputNodes] = {
  { 0, 0, 0 },   // Turn right
  { 1, 1, 0 },   // Turn left
  { 0, 1, 0 },   // Go straight
  { 0, 0, 0 },   // Turn Right
  { 0, 0, 0 },   // Turn Right
  { 0, 1, 0 },   // Go Straight
  { 0, 1, 0 },   // Go Straight
  { 0, 0, 0 },   // Turn Right
  { 0, 0, 0 },   // Turn Right
  { 0, 1, 0 },   // Go Straight
  { 0, 1, 0 },   // Go Straight
  { 0, 0, 0 },   // Turn Left
  { 0, 0, 0 },    // Turn Left
  { 1, 1, 0 },   // Go Straight
  { 1, 1, 0 },   // Go Straight
  { 0, 0, 0 },   // Go Straight
  { 0, 0, 0 }    // Go Straight
};


//1000 - 0
//1500 - 0.5
//2000 - 1
//1 1 left
//0 0 right
//0 1 forward
//1 0 back

/*
const short int Input[PatternCount][InputNodes] = {
  { 1, 0, 0, 0, 0, 0, 0 },  // Front Collision
  { 0, 1, 0, 0, 0, 0, 0 },  // Right Collision
  { 0, 0, 1, 0, 0, 0, 0 },  // Back  Collision
  { 0, 0, 0, 1, 0, 0, 0 },  // Left  Collision
  { 0, 0, 0, 0, 0, 0, 0 },  // Nothing Happens - Sweep
  { 0, 0, 0, 0, 0, 1, 0 },  // Theta Target Right
  { 0, 0, 0, 0, 1, 0, 0 },  // Theta Target Left
  { 0, 0, 0, 0, 0, 0, 1 },  // Enemy Detect
  { 1, 0, 0, 0, 0, 0, 1 },  // Enemy Detect but front blocked
  { 1, 0, 0, 0, 1, 0, 1 },  // Obstacle in front, but target to your left
  { 1, 0, 0, 0, 0, 1, 1 },  // Obstacle in front, but target to your right
  { 1, 1, 1, 1, 0, 1, 0 },  // Collision all sides, robot stuck, target left
  { 1, 1, 1, 1, 1, 0, 0 },   // Collision all sides, robot stuck, target right
  { 1, 1, 0, 0, 0, 1, 0 },  // Collision front right corner, target right
  { 1, 1, 0, 0, 1, 0, 0 },   // Collision front right corner, target left
  { 1, 0, 0, 1, 0, 1, 0 },  // Collision front left corner, target right
  { 1, 0, 0, 1, 1, 0, 0 }   // Collision front left corner, target left
};

const short int Target[PatternCount][OutputNodes] = {
  { 1, 1, 0 },
  { 0, 1, 0 },
  { 1, 0, 0 },
  { 0, 1, 0 },
  { 0.5, 0.5, 0 },
  { 0, 0, 0 },
  { 1, 1, 0 },
  { 0, 1, 0 },
  { 1, 1, 0 },
  { 1, 1, 0 },
  { 1, 1, 0 },
  { 0.5, 0.5, 0 },
  { 0.5, 0.5, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 1, 1, 0 },
  { 1, 1, 0 }
};
*/
/******************************************************************
 * End Network Configuration
 ******************************************************************/


int j, p, q, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long  TrainingCycle;
float Rando;
float Error;
float Accum;


float Hidden[HiddenNodes];
float Output[OutputNodes];
float HiddenWeights[InputNodes + 1][HiddenNodes];
float OutputWeights[HiddenNodes + 1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];



void runNet() {


    ReportEvery1000 = 1;
    for (p = 0; p < PatternCount; p++) {
        RandomizedIndex[p] = p;
    }

    while (1)
    {

        /******************************************************************
        * Initialize HiddenWeights and ChangeHiddenWeights
        ******************************************************************/

        for (int i = 0; i < HiddenNodes; i++) {
            for (j = 0; j <= InputNodes; j++) {
                ChangeHiddenWeights[j][i] = 0.0;
                Rando = float(rand() % 100) / 100;
                HiddenWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
            }
        }
        /******************************************************************
        * Initialize OutputWeights and ChangeOutputWeights
        ******************************************************************/

        for (int i = 0; i < OutputNodes; i++) {
            for (j = 0; j <= HiddenNodes; j++) {
                ChangeOutputWeights[j][i] = 0.0;
                Rando = float(rand() % 100) / 100;
                OutputWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
            }
        }
        //Serial.println("");
        cout << "Initial/Untrained Outputs: " << endl;
        toTerminal();
        /******************************************************************
        * Begin training
        ******************************************************************/

        for (TrainingCycle = 1; TrainingCycle < 2147483647; TrainingCycle++) {

            /******************************************************************
            * Randomize order of training patterns
            ******************************************************************/

            for (p = 0; p < PatternCount; p++) {
                q = rand() % PatternCount;
                r = RandomizedIndex[p];
                RandomizedIndex[p] = RandomizedIndex[q];
                RandomizedIndex[q] = r;
            }
            Error = 0.0;
            /******************************************************************
            * Cycle through each training pattern in the randomized order
            ******************************************************************/
            for (q = 0; q < PatternCount; q++) {
                p = RandomizedIndex[q];

                /******************************************************************
                * Compute hidden layer activations
                ******************************************************************/

                for (int i = 0; i < HiddenNodes; i++) {
                    Accum = HiddenWeights[InputNodes][i];
                    for (j = 0; j < InputNodes; j++) {
                        Accum += Input[p][j] * HiddenWeights[j][i];
                    }
                    Hidden[i] = 1.0 / (1.0 + exp(-Accum));
                }

                /******************************************************************
                * Compute output layer activations and calculate errors
                ******************************************************************/

                for (int i = 0; i < OutputNodes; i++) {
                    Accum = OutputWeights[HiddenNodes][i];
                    for (j = 0; j < HiddenNodes; j++) {
                        Accum += Hidden[j] * OutputWeights[j][i];
                    }
                    Output[i] = 1.0 / (1.0 + exp(-Accum));
                    OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]);
                    Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]);
                }

                /******************************************************************
                * Backpropagate errors to hidden layer
                ******************************************************************/

                for (int i = 0; i < HiddenNodes; i++) {
                    Accum = 0.0;
                    for (j = 0; j < OutputNodes; j++) {
                        Accum += OutputWeights[i][j] * OutputDelta[j];
                    }
                    HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]);
                }


                /******************************************************************
                * Update Inner-->Hidden Weights
                ******************************************************************/


                for (int i = 0; i < HiddenNodes; i++) {
                    ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i];
                    HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i];
                    for (j = 0; j < InputNodes; j++) {
                        ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
                        HiddenWeights[j][i] += ChangeHiddenWeights[j][i];
                    }
                }

                /******************************************************************
                * Update Hidden-->Output Weights
                ******************************************************************/

                for (int i = 0; i < OutputNodes; i++) {
                    ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i];
                    OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i];
                    for (j = 0; j < HiddenNodes; j++) {
                        ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i];
                        OutputWeights[j][i] += ChangeOutputWeights[j][i];
                    }
                }
            }

            /******************************************************************
            * Every 1000 cycles send data to terminal for display
            ******************************************************************/
            ReportEvery1000 = ReportEvery1000 - 1;
            if (ReportEvery1000 == 0)
            {
                cout << endl;
                cout << endl;
                cout << "TrainingCycle: ";
                cout << TrainingCycle;
                cout << "  Error = ";
                cout << Error << endl;

                toTerminal();

                if (TrainingCycle == 1)
                {
                    ReportEvery1000 = 999;
                }
                else
                {
                    ReportEvery1000 = 1000;
                }
            }


            /******************************************************************
            * If error rate is less than pre-determined threshold then end
            ******************************************************************/

            if (Error < Success) break;
        }
        cout << endl;
        cout << endl;
        cout << "TrainingCycle: ";
        cout << TrainingCycle;
        cout << "  Error = ";
        cout << Error << endl;

        toTerminal();

        cout << endl;
        cout << endl;
        cout << "Training Set Solved! " << endl;
        cout << "--------" << endl;
        cout << endl;
        cout << endl;
        ReportEvery1000 = 1;
        break;
    }


}


void toTerminal()
{

    for (p = 0; p < PatternCount; p++) {
        cout << endl;
        cout << "  Training Pattern: ";
        cout << p << endl;
        cout << "  Input ";
        for (int i = 0; i < InputNodes; i++) {
            cout << Input[p][i];
            cout << "  ";
        }
        cout << "  Target ";
        for (int i = 0; i < OutputNodes; i++) {
            cout << Target[p][i];
            cout << " ";
        }
        /******************************************************************
        * Compute hidden layer activations
        ******************************************************************/

        for (int i = 0; i < HiddenNodes; i++) {
            Accum = HiddenWeights[InputNodes][i];
            for (j = 0; j < InputNodes; j++) {
                Accum += Input[p][j] * HiddenWeights[j][i];
            }
            Hidden[i] = 1.0 / (1.0 + exp(-Accum));
        }

        /******************************************************************
        * Compute output layer activations and calculate errors
        ******************************************************************/

        for (int i = 0; i < OutputNodes; i++) {
            Accum = OutputWeights[HiddenNodes][i];
            for (j = 0; j < HiddenNodes; j++) {
                Accum += Hidden[j] * OutputWeights[j][i];
            }
            Output[i] = 1.0 / (1.0 + exp(-Accum));
        }
        cout << "  Output ";
        for (int i = 0; i < OutputNodes; i++) {
            cout << Output[i];
            cout << " ";
        }
    }


}

void toTerminal2(bool net_mem[], double net_out[])
{
    
    bool Input[7];
    for (int i = 0; i < 7; i++)
    {
        Input[i] = net_mem[i];
    }

    for (p = 0; p < 1; p++) {
        cout << endl;
        cout << "  Training Pattern: ";
        cout << p << endl;
        cout << "  Input ";
        for (int i = 0; i < InputNodes; i++) {
            cout << Input[i];
            cout << "  ";
        }
        cout << "  Target ";
        for (int i = 0; i < OutputNodes; i++) {
            cout << Target[p][i];
            cout << " ";
        }
       
        //Compute hidden layer activations
        

        for (int i = 0; i < HiddenNodes; i++) {
            Accum = HiddenWeights[InputNodes][i];
            for (j = 0; j < InputNodes; j++) {
                Accum += Input[j] * HiddenWeights[j][i];
            }
            Hidden[i] = 1.0 / (1.0 + exp(-Accum));
        }

        
        // Compute output layer activations and calculate errors
     

        for (int i = 0; i < OutputNodes; i++) {
            Accum = OutputWeights[HiddenNodes][i];
            for (j = 0; j < HiddenNodes; j++) {
                Accum += Hidden[j] * OutputWeights[j][i];
            }
            Output[i] = 1.0 / (1.0 + exp(-Accum));
        }
        cout << "  Output ";
        for (int i = 0; i < OutputNodes; i++) {
            cout << Output[i];
            cout << " ";
        }
    }
    for (int i = 0; i < 3; i++)
    {
        net_out[i] = Output[i];
    }
    
}