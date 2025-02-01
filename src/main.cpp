#include <iostream>
#include <chrono>
#include <thread>
#include <gpiod.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

int main() {
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
