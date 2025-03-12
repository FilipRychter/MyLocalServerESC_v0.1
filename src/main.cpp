#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <condition_variable>

#include "main.hpp"

#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <SDL2/SDL.h>


// Mongoose to jako server ?


//to conect device try
// sudo systemctl restart bluetooth
// bluetoothctl connect 85:55:A1:81:10:04





// Globalne zmienne // [DEL] Device 85:55:A1:81:10:04 Gamesir-G4
SDL_GameController *controller = nullptr;
std::queue<uint8_t> spiQueue;
std::mutex queueMutex;
std::condition_variable queueCond;
bool running = true;
Sint16 l2_value;
Sint16 r2_value;

Sint16 l2_value_old = -0;
Sint16 r2_value_old = -0;


// <<--------------------------------------------------- // funkcje

// Znajduje pierwszy dostępny kontroler
SDL_GameController *findController() {
    for (int i = 0; i < SDL_NumJoysticks(); i++) {
        if (SDL_IsGameController(i)) {
            return SDL_GameControllerOpen(i);
        }
    }
    return nullptr;
}

uint8_t map_button(uint16_t button) {
    switch (button) {
        case 1: return 'A'; // 'A' is 65 in ASCII
        default: return 0xFF;
    }
}



// <<--------------------------------------------------- // wontki


// Wątek odczytujący gamepada
int readGamepad() {
    SDL_Event event;
    SDL_Joystick* joystick;
    while (running) {

        while (SDL_PollEvent(&event)) {
            // Odczyt wartości triggerów (przykładowe osie 4 i 5, sprawdź w evtest)
            switch (event.type) {
                // SDL zwraca wartości w zakresie 0 - 32767
                case SDL_QUIT:
                    running = false;
                    break;

                case SDL_CONTROLLERDEVICEADDED:
                    if (!controller) {
                        controller = SDL_GameControllerOpen(event.cdevice.which);

                    }
                    break;

                case SDL_CONTROLLERDEVICEREMOVED:
                    if (controller && event.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controller))) {
                        SDL_GameControllerClose(controller);
                        controller = findController();

                    }
                    break;
                
                case SDL_CONTROLLERBUTTONDOWN: {
                    std::cout << "Button pressed: " << (int)event.cbutton.button << std::endl;

                    switch (event.cbutton.button) {
                        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_X:
                            std::cerr << "X pressed!" << std::endl;
                            break;
                    }

                    std::lock_guard<std::mutex> lock(queueMutex);
                    spiQueue.push(event.cbutton.button);
                    queueCond.notify_one();
                    break;
                }
                case SDL_CONTROLLERAXISMOTION: {
                    // std::cout << "Axis " << (int)event.caxis.axis << " moved to " << event.caxis.value << std::endl;

                    switch (event.caxis.axis) {
                        case SDL_CONTROLLER_AXIS_TRIGGERLEFT: // L2
                            // printf("L2: %d\n", event.caxis.value);
                            break;

                        case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: // R2
                            // printf("R2: %d\n", event.caxis.value);
                            break;
                    }
                    std::lock_guard<std::mutex> lock(queueMutex);
                    spiQueue.push(event.caxis.value & 0xFF); // Dolny bajt wartości
                    queueCond.notify_one();
                    break;
                }
                default:{
                    std::cout << event.type;
                    break;
                }
            }
        }
        joystick = SDL_GameControllerGetJoystick(controller);
        l2_value = SDL_JoystickGetAxis(joystick, 4);  // Evtest może pokazać inny numer osi
        r2_value = SDL_JoystickGetAxis(joystick, 5);
        if(l2_value != l2_value_old || r2_value != r2_value_old){
            l2_value_old = l2_value;
            r2_value_old = r2_value;
            // std::cout << "L2: " << l2_value << " | R2: " << r2_value << std::endl;
        }
    }

    if (controller) {
        SDL_GameControllerClose(controller);
    }
    return 0;
}

// Wątek wysyłający dane przez SPI
void sendSPIData() {
    int spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
        std::cerr << "Nie udało się otworzyć SPI." << std::endl;
        return;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 500000; // 500 kHz

    
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cerr << "Błąd ustawienia trybu SPI." << std::endl;
        close(spi_fd);
        return;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Błąd ustawienia liczby bitów na słowo SPI." << std::endl;
        close(spi_fd);
        return;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "Błąd ustawienia prędkości SPI." << std::endl;
        close(spi_fd);
        return;
    }


    while (running) {
        std::unique_lock<std::mutex> lock(queueMutex);
        queueCond.wait(lock, [] { return !spiQueue.empty() || !running; });

        while (!spiQueue.empty()) {
            uint8_t data = spiQueue.front();
            spiQueue.pop();
            lock.unlock();
             // Przykład wysyłania danych przez SPI
            uint8_t tx_data[] = {0x01, 0x02, 0x03, 0x04};  
            uint8_t rx_data[sizeof(tx_data)] = {0};

            struct spi_ioc_transfer transfer = {};
            transfer.tx_buf = reinterpret_cast<unsigned long>(tx_data);
            transfer.rx_buf = reinterpret_cast<unsigned long>(rx_data);
            transfer.len = sizeof(tx_data);
            transfer.speed_hz = speed;
            transfer.bits_per_word = bits;
            transfer.delay_usecs = 0;

            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
                std::cerr << "Błąd transmisji SPI." << std::endl;
                close(spi_fd);
    }

    // std::cout << "Odebrane dane przez SPI: ";
    // for (int i = 0; i < sizeof(rx_data); ++i) {
    //     std::cout << "0x" << std::hex << (int)rx_data[i] << " ";
    // }
        lock.lock();
        }
    }
    close(spi_fd);
}


int SPI_test(){
    // GPIO: Ustawienie numeru GPIO (np. 17)
    const char* gpio_chipname = "/dev/gpiochip0";
    int gpio_line_num = 3;

    // Połączenie z chipem GPIO
    gpiod_chip* gpio_chip = gpiod_chip_open(gpio_chipname);
    if (!gpio_chip) {
        std::cerr << "Nie udało się otworzyć chipu GPIO." << std::endl;
        return -1;
    }

    // Uzyskanie dostępu do linii GPIO
    gpiod_line* gpio_line = gpiod_chip_get_line(gpio_chip, gpio_line_num);
    if (!gpio_line) {
        std::cerr << "Nie udało się uzyskać dostępu do linii GPIO." << std::endl;
        gpiod_chip_close(gpio_chip);
        return -1;
    }

    // Ustawienie GPIO na wyjście
    if (gpiod_line_request_output(gpio_line, "gpio_example", 0) < 0) {
        std::cerr << "Nie udało się ustawić linii GPIO jako wyjście." << std::endl;
        gpiod_chip_close(gpio_chip);
        return -1;
    }

    // Wysyłanie sygnału na pin (wysoki stan)
    gpiod_line_set_value(gpio_line, 1);
    std::cout << "Pin GPIO ustawiony na wysoki stan!" << std::endl;

    // Ustawienie niskiego stanu
    gpiod_line_set_value(gpio_line, 0);
    std::cout << "Pin GPIO ustawiony na niski stan!" << std::endl;

    // Zwolnienie zasobów GPIO
    gpiod_chip_close(gpio_chip);

    // SPI: Ustawienie SPI (np. /dev/spidev0.0)
    int spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
        std::cerr << "Nie udało się otworzyć SPI." << std::endl;
        return -1;
    }

    // Ustawienie parametrów SPI
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 500000; // 500 kHz

    // Ustawienie trybu SPI
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cerr << "Błąd ustawienia trybu SPI." << std::endl;
        close(spi_fd);
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Błąd ustawienia liczby bitów na słowo SPI." << std::endl;
        close(spi_fd);
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "Błąd ustawienia prędkości SPI." << std::endl;
        close(spi_fd);
        return -1;
    }

    // Przykład wysyłania danych przez SPI
    uint8_t tx_data[] = {0x01, 0x02, 0x03, 0x04};  // Przykładowe dane do wysłania
    uint8_t rx_data[4] = {0};

    struct spi_ioc_transfer transfer = {};
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx_data);
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx_data);
    transfer.len = sizeof(tx_data);
    transfer.speed_hz = speed;
    transfer.bits_per_word = bits;
    transfer.delay_usecs = 0;

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        std::cerr << "Błąd transmisji SPI." << std::endl;
        close(spi_fd);
        return -1;
    }

    std::cout << "Odebrane dane przez SPI: ";
    for (int i = 0; i < sizeof(rx_data); ++i) {
        std::cout << "0x" << std::hex << (int)rx_data[i] << " ";
    }
    std::cout << std::dec << std::endl;

    // Zamknięcie pliku SPI
    close(spi_fd);

    return 0;
}

int main() {
    
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
        std::cerr << "SDL Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    controller = findController();
    if (!controller) {
        std::cerr << "Nie udało się otworzyć kontrolera." << std::endl;
        SDL_Quit();
        return 1;
    }


    std::thread t1(readGamepad);
    std::thread t2(sendSPIData);

    t1.join();
    t2.join();




    SDL_Quit();


    return 0;
}