#pragma once

class Output
{
private:
    double value;
public:
    Output();
    double& get_value() { return value; }
    void sigmoid();
};

