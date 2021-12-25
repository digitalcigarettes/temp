#include <iostream>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

using namespace std;

void ExportGPin(char pin[]){
    
    int fd = open("/sys/class/gpio/export", O_WRONLY);

    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/export");
        exit(1);
    }
    

    if (write(fd, pin, strlen(pin)) != 2) {
        perror("Error writing to /sys/class/gpio/export");
        exit(1);
    }

    close(fd);

}

void UNExportGPin(char pin[]){
    


    int fd = open("/sys/class/gpio/unexport", O_WRONLY);

    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/unexport");
        exit(1);
    }

    if (write(fd, pin, strlen(pin)) != 2) {
        perror("Error writing to /sys/class/gpio/unexport");
        exit(1);
    }

    close(fd);

}

int main()
{

    string pin  ="18";

    ExportGPin(const_cast<char*>(pin.c_str()));

    // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio24/direction

    int fd = open("/sys/class/gpio/gpio24/direction", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/gpio24/direction");
        exit(1);
    }

    if (write(fd, "out", 3) != 3) {
        perror("Error writing to /sys/class/gpio/gpio24/direction");
        exit(1);
    }

    close(fd);

    fd = open("/sys/class/gpio/gpio24/value", O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/gpio24/value");
        exit(1);
    }

    // Toggle LED 50 ms on, 50ms off, 100 times (10 seconds)
    
    for (int i = 0; i < 100; i++) {
        cout<<"ON"<<endl;
        if (write(fd, "1", 1) != 1) {
            perror("Error writing to /sys/class/gpio/gpio24/value");
            exit(1);
        }

        usleep(2000000);

        cout<<"OFF"<<endl;
        if (write(fd, "0", 1) != 1) {
            perror("Error writing to /sys/class/gpio/gpio24/value");
            exit(1);
        }

        usleep(50000);
    }
    

    close(fd);
    cout<<"close"<<endl;


    UNExportGPin(const_cast<char*>(pin.c_str()));

    return 0;
}