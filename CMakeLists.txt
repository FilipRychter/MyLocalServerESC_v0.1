cmake_minimum_required(VERSION 3.10)
project(ESC_Server_V0_1)

set(CMAKE_CXX_STANDARD 17)

# Szukamy bibliotek
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# Tworzymy plik wykonywalny
add_executable(ESC_Server_V0_1
    src/main.cpp
)

# Dodajemy katalogi nagłówkowe
include_directories(${GPIOD_INCLUDE_DIRS})

# Linkujemy biblioteki
target_link_libraries(ESC_Server_V0_1
    ${GPIOD_LIB}
    pthread
    rt
)
