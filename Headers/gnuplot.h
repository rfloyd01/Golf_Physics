#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

class Gnuplot
{
public:
    Gnuplot();
    ~Gnuplot();
    void operator ()(const string& command); // send any command to gnuplot

protected:
    FILE* gnuplotpipe;
};

//Tester Functions
void graphBallFlight();
