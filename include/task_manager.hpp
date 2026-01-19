#pragma once

#include <STM32FreeRTOS.h>
#include <array>

// ============================================================================
// TASK CONFIGURATION
// ============================================================================

enum class TaskID : uint8_t {
    BATTERY,
    BUTTONS,
    IMU,
    RANGE,
    RUNTIME_STATS,
    UROS,
    WHEELS_CTRL,
    COUNT
};

struct TaskConfig {
    const char* name;
    uint32_t stack_size;
    UBaseType_t priority;
    TaskFunction_t function;
    void* params;
};

// ============================================================================
// TASK MANAGER
// ============================================================================

class TaskManager {
public:
    static TaskManager& getInstance() {
        static TaskManager instance;
        return instance;
    }
    
    void registerTask(TaskID id, const TaskConfig& config) {
        configs_[static_cast<uint8_t>(id)] = config;
        registered_[static_cast<uint8_t>(id)] = true;
    }
    
    void createAll() {
        for (uint8_t i = 0; i < static_cast<uint8_t>(TaskID::COUNT); i++) {
            if (registered_[i]) {
                createTask(static_cast<TaskID>(i));
            }
        }
    }
    
    bool createTask(TaskID id) {
        const uint8_t idx = static_cast<uint8_t>(id);
        if (!registered_[idx]) return false;
        
        const auto& cfg = configs_[idx];
        
        TaskHandle_t handle;
        BaseType_t result = xTaskCreate(
            cfg.function,
            cfg.name,
            cfg.stack_size,
            cfg.params,
            cfg.priority,
            &handle
        );
        
        if (result == pdPASS) {
            handles_[idx] = handle;
            return true;
        }
        return false;
    }
    
    void deleteTask(TaskID id) {
        const uint8_t idx = static_cast<uint8_t>(id);
        if (handles_[idx] != nullptr) {
            vTaskDelete(handles_[idx]);
            handles_[idx] = nullptr;
        }
    }
    
    void suspendTask(TaskID id) {
        if (handles_[static_cast<uint8_t>(id)]) {
            vTaskSuspend(handles_[static_cast<uint8_t>(id)]);
        }
    }
    
    void resumeTask(TaskID id) {
        if (handles_[static_cast<uint8_t>(id)]) {
            vTaskResume(handles_[static_cast<uint8_t>(id)]);
        }
    }
    
    TaskHandle_t getHandle(TaskID id) const { 
        return handles_[static_cast<uint8_t>(id)]; 
    }
    
    bool isRunning(TaskID id) const {
        TaskHandle_t h = handles_[static_cast<uint8_t>(id)];
        return h != nullptr && eTaskGetState(h) != eDeleted;
    }

private:
    TaskManager() {
        handles_.fill(nullptr);
        registered_.fill(false);
    }
    
    std::array<TaskConfig, static_cast<uint8_t>(TaskID::COUNT)> configs_{};
    std::array<TaskHandle_t, static_cast<uint8_t>(TaskID::COUNT)> handles_{};
    std::array<bool, static_cast<uint8_t>(TaskID::COUNT)> registered_{};
};

// Global accessor
inline TaskManager& Tasks = TaskManager::getInstance();