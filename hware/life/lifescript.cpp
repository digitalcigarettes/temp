#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <cstring>

using namespace std;


#define EXIT exit(1);
typedef long long int ll;

#ifdef __unix__         
    #define LINUX 1
    #include <unistd.h>
    #include <errno.h>
    #include <fcntl.h>

    #include <sys/stat.h>
    #include <sys/types.h>
    #include "gpio.h"
    
#elif defined(_WIN32) || defined(WIN32) || defined(_WIN64)
    #define LINUX 0
    #include <windows.h>
    #include <tchar.h>

#endif



int main(){
    return 0;
}
