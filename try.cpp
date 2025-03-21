#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <vector>
#include <cmath>
#include <chrono>
#include <csignal>

#define PWM_PIN1 26
#define IN1 22
#define IN2 21

#define PWM_PIN2 23
#define IN3 25
#define IN4 27

void cleanup(int sig) {
    digitalWrite(PWM_PIN1, LOW);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);

    digitalWrite(PWM_PIN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    
    std::cout << "Cleanup complete. Exiting program.\n";
    exit(0); // Exit the program
}

void setup(){
    // Initialize WiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "WiringPi Setup failed!" << std::endl;
    }

    pinMode (PWM_PIN1, PWM_OUTPUT);
    pinMode (IN1, OUTPUT);
    pinMode (IN2, OUTPUT);

    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, HIGH);
    pwmWrite(PWM_PIN1, 0);

    pinMode (PWM_PIN2, PWM_OUTPUT);
    pinMode (IN3, OUTPUT);
    pinMode (IN4, OUTPUT);

    digitalWrite (IN3, HIGH);
    digitalWrite (IN4, HIGH);
    pwmWrite(PWM_PIN2, 0);
}


int main() {
    printf("INIT\n");
    signal(SIGINT, cleanup);
    setup();

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);


    int i = 700;
    while(1){
        pwmWrite(PWM_PIN1, 800);
        pwmWrite(PWM_PIN2, 800);
        delay(30);
    }
    return 0;
}
