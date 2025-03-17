#include "shared_data.hpp"

void SharedData::setValue(int val) {
    std::lock_guard<std::mutex> lock(mtx);
    value = val;
}

int SharedData::getValue() {
    std::lock_guard<std::mutex> lock(mtx);
    return value;
}