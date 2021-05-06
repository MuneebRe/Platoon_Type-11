#include "Output.h"

#include <iostream>

using namespace std;

Output::Output()
{
	value = 0;
}

void Output::sigmoid()
{
	value = 1.0 / (1.0 + exp(-value));
}