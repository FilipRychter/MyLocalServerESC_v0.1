#ifndef MAIN_HPP
#define MAIN_HPP

#include <iostream>

void say_hello();
int send();



#define MAX_DATA_SIZE 30
#define PREFIX_SIZE 3


// Mapa przycisków pada
enum class GamepadButton : uint8_t {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    BACK = 6,
    START = 7,
    LSTICK = 8,
    RSTICK = 9
};

// Mapa osi analogowych (w tym triggerów)
enum class GamepadAxis : uint8_t {
    LX = 0, // Lewa gałka X
    LY = 1, // Lewa gałka Y
    RX = 2, // Prawa gałka X
    RY = 3, // Prawa gałka Y
    L2 = 4, // Lewy trigger
    R2 = 5  // Prawy trigger
};

// Rodzaje operacji
enum class Operation : uint8_t {
    WRITE = 0x02,
    READ = 0x01
};


#endif  // MAIN_HPP