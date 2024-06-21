#include <iostream>
#include "math.h"

using namespace std;

int numFinder(double speed)
{
    return int(round((1218.75/speed) - 2.0));
}

int main()
{
    while(1)
    {
        double speed;
        cout<<"Enter Number: ";
        cin>>speed;
        cout<<"n = "<<numFinder(speed)<<"\n";
    }
}
