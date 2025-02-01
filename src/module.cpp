#include "module.hpp"


#include <gpiod.h>  // Biblioteka libgpiod


#include <iostream>
#include <chrono>
#include <thread>


void print_message() {
    std::cout << "are we alive ?" << std::endl;
}



int gpio_check(){
 const char *chipname = "gpiochip0";  // Nazwa chipa GPIO (domyślnie gpiochip0 na Raspberry Pi)
    struct gpiod_chip *chip;
    struct gpiod_line *line ,*line2,*line3;
    int ret;

    // Otwórz chip GPIO
    chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        std::cerr << "Nie można otworzyć chipa GPIO!" << std::endl;
        return 1;
    }
    
    // Pobierz linię GPIO (np. GPIO17)
    line = gpiod_chip_get_line(chip, 2);  // Numer pinu GPIO (17)
    if (!line) {
        std::cerr << "Nie można pobrać linii GPIO!" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }


    line2 = gpiod_chip_get_line(chip, 3);  // Numer pinu GPIO (17)
    if (!line) {
        std::cerr << "Nie można pobrać linii GPIO!" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }



    gpiod_line_set_value(line2, 0);  // Ustaw stan niski (LED wyłącza się)

    line3 = gpiod_chip_get_line(chip, 4);  // Numer pinu GPIO (17)
    if (!line) {
        std::cerr << "Nie można pobrać linii GPIO!" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }
    gpiod_line_set_value(line3, 0);  // Ustaw stan niski (LED wyłącza się)


    // Skonfiguruj linię jako wyjście
    ret = gpiod_line_request_output(line, "example", 0);  // "example" to etykieta, 0 to początkowy stan (niski)
    if (ret < 0) {
        std::cerr << "Nie można skonfigurować linii jako wyjście!" << std::endl;
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }
    ret = gpiod_line_request_output(line2, "example", 0);  // "example" to etykieta, 0 to początkowy stan (niski)
    if (ret < 0) {
        std::cerr << "Nie można skonfigurować linii jako wyjście!" << std::endl;
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }
    ret = gpiod_line_request_output(line3, "example", 0);  // "example" to etykieta, 0 to początkowy stan (niski)
    if (ret < 0) {
        std::cerr << "Nie można skonfigurować linii jako wyjście!" << std::endl;
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        return 1;
    }

    // Miganie diodą m
    for (int i = 0; i < 10; ++i) {
        gpiod_line_set_value(line, 1);  // Ustaw stan wysoki (LED włącza się)
        std::cout << "LED ON" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Czekaj 500 ms

        gpiod_line_set_value(line, 0);  // Ustaw stan niski (LED wyłącza się)
        std::cout << "LED OFF" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Czekaj 500 ms
    }



    // Zwolnij linię i zamknij chip
    gpiod_line_release(line);
    gpiod_chip_close(chip);

    std::cout << "Program zakończony." << std::endl;
    return 0;
}