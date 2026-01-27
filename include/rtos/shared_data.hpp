// shared_data.hpp
#pragma once
#include <FreeRTOS.h>
#include <semphr.h>

// Thread-safe data container
template<typename T>
class SharedData {
public:
    SharedData() {
        mutex_ = xSemaphoreCreateMutex();
    }
    
    void set(const T& value) {
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            data_ = value;
            xSemaphoreGive(mutex_);
        }
    }
    
    T get() {
        T copy;
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            copy = data_;
            xSemaphoreGive(mutex_);
        }
        return copy;
    }
    
    // Lock-free read (use carefully!)
    const T& peek() const { return data_; }

private:
    T data_;
    SemaphoreHandle_t mutex_;
};
