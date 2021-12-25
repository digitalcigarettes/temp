#ifndef GPIO_H
#define GPIO_H

#include <iostream> 
#include <map>
#include "errno.h"
#include "fcntl.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <unordered_map>

class GPIO_S{
    private:
        const char *GPIO_PATH = "/sys/class/gpio/export";
        std::map<int,int> pins;

        void add(int k, int v){
            pins[k] = v;
        }

        void rmv(int v){
            std::map<int,int>::iterator iter = pins.find(v);
            pins.erase(iter);
        }

    public:
        void setpin(int pinNum, int state), exportPin(int p), unexportPin(int p);
        void shutAll();


};

void GPIO_S::setpin(int pinNum, int State){
    if ( this->pins.find(pinNum) == this->pins.end() ) {
        this->exportPin(pinNum);
    } else {
        this->pins[pinNum] = State;
    }
}

void GPIO_S::exportPin(int p)
{
    const char *pin = std::to_string(p).c_str();

    int fd = open( this->GPIO_PATH, O_WRONLY);

    if(fd==-1){
        perror("Unable to open /sys/class/gpio/export");
        exit(-1);
    }

    if (write(fd, pin, strlen(pin)) != 2) {
        perror("Error writing pin");
        exit(-1);
    }

    this->add(p,0);


    exit(-1);
}



void GPIO_S::unexportPin(int p){
    
    const char *pin = to_string(p).c_str();

    int fd = open( this->GPIO_PATH, O_WRONLY);


    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/unexport");
        EXIT;
    }

    if (write(fd, pin, strlen(pin)) != 2) {
        perror("Error writing to /sys/class/gpio/unexport");
        exit(1);
    }
    
    this->rmv(p);

    close(fd);

}

void GPIO_S::shutAll(){
    std::map<int, int>::iterator it;

    for (it = this->pins.begin(); it != this->pins.end(); it++)
    {
        this->unexportPin(it->first);
    }
}

#endif

