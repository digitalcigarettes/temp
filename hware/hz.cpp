#include <iostream>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#define KILO 1000
#define MEGA 1000000
#define GIGA  1000000000

using namespace std;

void delay(int milli_seconds)
{
  
    clock_t start_time = clock();
  
    while (clock() < start_time + milli_seconds);
}


int main(){

    long long int frequency_hertz = 10000000;
    long long int NT = GIGA/frequency_hertz;

    cout<<NT<<endl;
    cout<<"Frequency: "<<frequency_hertz<<endl;

    struct timespec cT, eT;
    long long int ticks = 0, TSec;
 
    clock_gettime(CLOCK_REALTIME, &cT);

    while(ticks < frequency_hertz)
    {

        clock_gettime(CLOCK_REALTIME, &eT);
        TSec = GIGA*(eT.tv_sec-cT.tv_sec) + eT.tv_nsec-cT.tv_nsec;


        if( TSec%NT<300) //TUNING
        {
//            cout<<TSec<<endl;
            ticks++;
        }
    }

 
    
    double T = eT.tv_sec - cT.tv_sec + (eT.tv_nsec - cT.tv_nsec)/GIGA;
    printf("T: %f\n",T);
    cout<<ticks<<endl;

    return 0;

}
