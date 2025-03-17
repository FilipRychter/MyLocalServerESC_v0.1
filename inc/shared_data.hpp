#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <mutex>

class SharedData {
public:
    void setValue(int val);
    int getValue();

private:
    int value;
    std::mutex mtx;
};

#endif // SHARED_DATA_H