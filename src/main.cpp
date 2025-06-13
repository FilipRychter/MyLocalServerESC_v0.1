/*

PLANS:
uporzątkować progrtam i pohować w dydykowanie moduły 
zrobić aby pad był ładnie podłączany i odłączany 
dodać spi jako osobny i uniwersalny moduł i tak aby STM32 mógł to zrozumieć
dodać MQTT jako osobny wątek 







//  sudo systemctl restart bluetooth
//  bluetoothctl connect 85:55:A1:81:10:04

*/


#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <condition_variable>

#include "main.hpp"
#include "shared_data.hpp"

#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <SDL2/SDL.h>

#include <csignal>


// ------------------- GPIO Globals -------------------
gpiod_line* gpio_line_3;
gpiod_line* gpio_line_2;

gpiod_chip* gpio_chip;

std::atomic<int> duty_cycle_2(50);        // Wypełnienie 0–100%
std::atomic<int> duty_cycle_3(50);        // Wypełnienie 0–100%

std::atomic<bool> pwm_running(true);    // Kontrola wątku PWM

std::queue<uint8_t> spiQueue;
std::mutex queueMutex;
std::condition_variable queueCond;


SDL_GameController *controller = nullptr;
SDL_Event event;


bool running = true;


// ----------------- Kontroler Init -----------------
SDL_GameController *findController()
{
    for (int i = 0; i < SDL_NumJoysticks(); i++)
    {
        if (SDL_IsGameController(i))
        {
            return SDL_GameControllerOpen(i);
        }
    }
    return nullptr;
}

void processEvents()
{
    while (SDL_PollEvent(&event))
    {
        // Odczyt wartości triggerów (przykładowe osie 4 i 5, sprawdź w evtest)
        switch (event.type)
        {
        // SDL zwraca wartości w zakresie 0 - 32767
        case SDL_QUIT:
            running = false;
            break;

        case SDL_CONTROLLERDEVICEADDED:
            if (!controller)
            {
                controller = SDL_GameControllerOpen(event.cdevice.which);
            }
            break;

        case SDL_CONTROLLERDEVICEREMOVED:
            if (controller && event.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controller)))
            {
                SDL_GameControllerClose(controller);
                controller = findController();
            }
            break;

        case SDL_CONTROLLERBUTTONDOWN:
        {
            std::cout << "Button pressed: " << (int)event.cbutton.button << std::endl;

            switch (event.cbutton.button)
            {
            case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_X:
                std::cerr << "X pressed!" << std::endl;
                break;
            }

            std::lock_guard<std::mutex> lock(queueMutex);
            spiQueue.push(event.cbutton.button);
            queueCond.notify_one();
            break;
        }
        case SDL_CONTROLLERAXISMOTION:
        {
            std::cout << "Axis " << (int)event.caxis.axis << " moved to " << event.caxis.value << std::endl;

            switch (event.caxis.axis)
            {
            case SDL_CONTROLLER_AXIS_TRIGGERLEFT: // L2
                printf("L2: %d\n", event.caxis.value);
                break;

            case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: // R2
                printf("R2: %d\n", event.caxis.value);
                break;
            }
            // std::lock_guard<std::mutex> lock(queueMutex);
            // spiQueue.push(event.caxis.value & 0xFF); // Dolny bajt wartości 00000000 000000000 &
            // spiQueue.push(event.caxis.value >> 8);   // 00000000 111111111
            // queueCond.notify_one();
            break;
        }
        }
    }
}

void readTriggers()
{
    static int16_t l2_value_old = -32768, r2_value_old = -32768;

    if (!controller)
        return;

    SDL_Joystick *joystick = SDL_GameControllerGetJoystick(controller);
    int16_t l2_value = SDL_JoystickGetAxis(joystick, 4); // Evtest może pokazać inny numer osi
    int16_t r2_value = SDL_JoystickGetAxis(joystick, 5);

    if (std::abs(l2_value - l2_value_old) > 500)// wez tu wstaw podstawową wartosc aby odrazu nieprubowal wysłac 2 na raz 
    {
        l2_value_old = l2_value;
        duty_cycle_3 = (l2_value + 32768) * 100 / 65535;
        std::cout << "L2" << duty_cycle_3;


    }

    if (std::abs(r2_value - r2_value_old) > 500)
    {
        r2_value_old = r2_value;
        duty_cycle_2 = (r2_value + 32768) * 100 / 65535;
        std::cout << "R2" << duty_cycle_2;

    }
}


//--------------

int Init_SDL2(){
    
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0)
    {
        std::cerr << "SDL Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    controller = findController();
    if (!controller)
    {
        std::cerr << "Nie udało się otworzyć kontrolera." << std::endl;
        SDL_Quit();
        return 1;
    }
    return 0;
}

// Wątek odczytujący gamepada
int readGamepad()
{
    // Init_SDL2();

    while (running)
    {
        processEvents();
        readTriggers();
    }

    if (controller)
    {
        SDL_GameControllerClose(controller);
    }
    return 0;
}



// ------------------- GPIO Init ----------------------
int GPIO_Init_3()
{
      const char *gpio_chipname = "/dev/gpiochip0";
    int gpio_line_num = 3;

    // Połączenie z chipem GPIO
    gpio_chip = gpiod_chip_open(gpio_chipname);
    if (!gpio_chip)
    {
        std::cerr << "Nie udało się otworzyć chipu GPIO." << std::endl;
        return -1;
    }

    // Uzyskanie dostępu do linii GPIO
    gpio_line_3 = gpiod_chip_get_line(gpio_chip, gpio_line_num);
    if (!gpio_line_3)
    {
        std::cerr << "Nie udało się uzyskać dostępu do linii GPIO." << std::endl;
        gpiod_chip_close(gpio_chip);
        return -1;
    }

    // Ustawienie GPIO na wyjście
    if (gpiod_line_request_output(gpio_line_3, "gpio_example", 0) < 0)
    {
        std::cerr << "Nie udało się ustawić linii GPIO jako wyjście." << std::endl;
        gpiod_chip_close(gpio_chip);
        return -1;
    }
    
    // Wysyłanie sygnału na pin (wysoki stan)
    gpiod_line_set_value(gpio_line_3, 1);
    std::cout << "Pin GPIO ustawiony na wysoki stan!" << std::endl;

    return 0;
}


int GPIO_Init_2()
{
      const char *gpio_chipname = "/dev/gpiochip0";
    int gpio_line_num = 2;

    // Połączenie z chipem GPIO
    gpio_chip = gpiod_chip_open(gpio_chipname);
    if (!gpio_chip)
    {
        std::cerr << "Nie udało się otworzyć chipu GPIO." << std::endl;
        return -1;
    }

    // Uzyskanie dostępu do linii GPIO
    gpio_line_2 = gpiod_chip_get_line(gpio_chip, gpio_line_num);
    if (!gpio_line_2)
    {
        std::cerr << "Nie udało się uzyskać dostępu do linii GPIO." << std::endl;
        gpiod_chip_close(gpio_chip);
        return -1;
    }

    // Ustawienie GPIO na wyjście
    if (gpiod_line_request_output(gpio_line_2, "gpio_example", 0) < 0)
    {
        std::cerr << "Nie udało się ustawić linii GPIO jako wyjście." << std::endl;
        gpiod_chip_close(gpio_chip);
        return -1;
    }
    
    // Wysyłanie sygnału na pin (wysoki stan)
    gpiod_line_set_value(gpio_line_2, 1);
    std::cout << "Pin GPIO ustawiony na wysoki stan!" << std::endl;

    return 0;
}


// ------------------- PWM Loop -----------------------
void pwmLoop_2(gpiod_line* line, int frequency_hz = 1000)
{
    using namespace std::chrono;
    int period_us = 1'000'000 / frequency_hz;

    while (pwm_running)
    {
        int high_time = period_us * duty_cycle_2 / 100;
        int low_time = period_us - high_time;

        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(microseconds(high_time));

        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(microseconds(low_time));
    }

    gpiod_line_set_value(line, 0); // Wyłącz na koniec
}


void pwmLoop_3(gpiod_line* line, int frequency_hz = 1000)
{
    using namespace std::chrono;
    int period_us = 1'000'000 / frequency_hz;

    while (pwm_running)
    {
        int high_time = period_us * duty_cycle_3 / 100;
        int low_time = period_us - high_time;

        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(microseconds(high_time));

        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(microseconds(low_time));
    }

    gpiod_line_set_value(line, 0); // Wyłącz na koniec
}

// -------------------- SIGNAL EVENT ---------------------

void signalHandler(int signum)
{
    std::cout << "\nZatrzymywanie programu..." << std::endl;
    running = false;
    pwm_running = false;

    if (controller){
        controller = nullptr;
        SDL_GameControllerClose(controller);
    }

    SDL_Quit();

    if (gpio_chip){
        gpio_chip = nullptr;
        gpiod_chip_close(gpio_chip);
    }

    std::cout << "Zasoby SDL i GPIO zostały zwolnione. Kończę." << std::endl;
}


// ------------------- Main ---------------------------
int main()
{
    signal(SIGINT, signalHandler);


    if (GPIO_Init_2() != 0 || GPIO_Init_3() != 0)
        return -1;

    if (Init_SDL2() != 0)
        return -1;

    // Przykład mapowania wartości (symulacja triggera)
    int raw_value = 125;
    duty_cycle_2 = (raw_value + 32768) * 100 / 65535;
    duty_cycle_3 = (raw_value + 32768) * 100 / 65535;


    pwm_running = true;
    std::thread pwm_thread_2(pwmLoop_2, gpio_line_2, 1000); // 1 kHz PWM
    std::thread pwm_thread_3(pwmLoop_3, gpio_line_3, 1000); // 1 kHz PWM
    std::thread gamepad_thread(readGamepad);
    
    
    gamepad_thread.join();
    pwm_thread_2.join();
    pwm_thread_3.join();


   std::cout << "Zamykanie programu" << std::endl;


    return 0;
}
