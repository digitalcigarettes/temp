#include <iostream>
#include <time.h>

using namespace std;

struct T{

    int M, D, Hr, Min, Sec;


    void refresh(){
        time_t cT = time(NULL);
        tm *lT = localtime(&cT);

        M=(lT->tm_mon)+1; D=lT->tm_mday; Hr=lT->tm_hour; Min=lT->tm_min; Sec=lT->tm_sec;

    }
};

int main() 
{
    // T Synclock; Synclock.refresh();
    // if(Synclock.Hr == 2){
        
    // }
    
    clock_t T;

    long long x = 0;

    T = clock();
    cout<<CLOCKS_PER_SEC<<endl;
    while( (clock()-T) < 1000){
        // if( (clock()-T) >= T){
        //     T++;
        //     x++;
        //     if(clock()%10000 == 0){
        //         cout<<clock()<<" "<<T<<endl;
        //     }
        // }

        // if( (clock()-T) % 1000 == 0){
        //     cout<<clock()<<" "<<T<<endl;
        // }

        // x++;

    }

//    end = clock();
//    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

    printf("%f", x);



    // long long int x = 0;
    // while(1){
    //     x++;
    //     if(x % 1000000==0){
    //         cout<<x<<endl;
    //     }
    // }

    

}
