cmake_minimum_required(VERSION 3.10)
project(ESC_Server_V0_1)

set(CMAKE_CXX_STANDARD 20) 

# Szukamy bibliotek
find_package(PkgConfig REQUIRED)

pkg_check_modules(GPIOD REQUIRED libgpiod)
pkg_check_modules(SDL2 REQUIRED sdl2)

# Tworzymy plik wykonywalny
add_executable(ESC_Server_V0_1
    src/main.cpp
    src/shared_data.cpp
)

# Dodajemy katalogi nagłówkowe
include_directories(inc ${GPIOD_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS})

# Linkujemy biblioteki
target_link_libraries(ESC_Server_V0_1
    ${GPIOD_LIBRARIES}
    ${SDL2_LIBRARIES}
    pthread
    rt
)
