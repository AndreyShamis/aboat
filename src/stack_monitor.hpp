#pragma once
// stack_monitor.hpp - Мониторинг использования стека задач FreeRTOS
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class StackMonitor {
private:
    static const int MAX_TASKS = 10;
    
    struct TaskInfo {
        TaskHandle_t handle;
        String name;
        UBaseType_t stackSize;
        UBaseType_t minFreeStack;
        bool isValid;
    };
    
    TaskInfo tasks[MAX_TASKS];
    int taskCount = 0;
    unsigned long lastCheck = 0;
    const unsigned long CHECK_INTERVAL = 5000; // 5 секунд
    
public:
    StackMonitor() {
        memset(tasks, 0, sizeof(tasks));
    }
    
    // Получить количество отслеживаемых задач
    int getTaskCount() const { return taskCount; }
    
    // Регистрация задачи для мониторинга
    void registerTask(TaskHandle_t handle, const String& name, UBaseType_t stackSize) {
        if (taskCount < MAX_TASKS) {
            tasks[taskCount].handle = handle;
            tasks[taskCount].name = name;
            tasks[taskCount].stackSize = stackSize;
            tasks[taskCount].minFreeStack = stackSize;
            tasks[taskCount].isValid = true;
            taskCount++;
        }
    }
    
    // Автоматическая регистрация основных задач (упрощённый метод)
    void autoRegisterTasks() {
        taskCount = 0;
        
        // // Регистрируем основные задачи вручную с известными параметрами
        // // Основная задача Arduino loop (IDLE task не всегда доступен)
        // if (!registerKnownTask("arduino_task", 8192)) {
        //     Serial.println("[StackMonitor] Задача arduino_task не найдена");
        // }
        // LoRa задачи (если они созданы)
        if (!registerKnownTask("LoRaRecv", 6144)) {
            Serial.println("[StackMonitor] Задача LoRaRecv не найдена");
        }
        if (!registerKnownTask("LoRaSend", 6144)) {
            Serial.println("[StackMonitor] Задача LoRaSend не найдена");
        }
        if (!registerKnownTask("LoRaRetry", 4096)) {
            Serial.println("[StackMonitor] Задача LoRaRetry не найдена");
        }
        // Системные задачи ESP32 (используем большие значения для безопасности)
        if (!registerKnownTask("IDLE0", 1024)) {
            Serial.println("[StackMonitor] Задача IDLE0 не найдена");
        }
        if (!registerKnownTask("IDLE1", 1024)) {
            Serial.println("[StackMonitor] Задача IDLE1 не найдена");
        }
        if (!registerKnownTask("wifi", 16384)) {
            Serial.println("[StackMonitor] Задача wifi не найдена");
        }
        if (!registerKnownTask("ipc0", 1024)) {
            Serial.println("[StackMonitor] Задача ipc0 не найдена");
        }
        if (!registerKnownTask("ipc1", 1024)) {
            Serial.println("[StackMonitor] Задача ipc1 не найдена");
        }
    }
    
public:
    // Обновление статистики стека
    void update() {
        unsigned long now = millis();
        if (now - lastCheck < CHECK_INTERVAL) return;
        
        for (int i = 0; i < taskCount; i++) {
            if (!tasks[i].isValid) continue;
            
            UBaseType_t freeStack = uxTaskGetStackHighWaterMark(tasks[i].handle);
            if (freeStack < tasks[i].minFreeStack) {
                tasks[i].minFreeStack = freeStack;
            }
        }
        
        lastCheck = now;
    }
    
    // Получить отчёт о состоянии стека
    String getReport() {
        String report = "Stack Status:\n";
        
        for (int i = 0; i < taskCount; i++) {
            if (!tasks[i].isValid) continue;
            
            UBaseType_t currentFree = uxTaskGetStackHighWaterMark(tasks[i].handle);
            
            // Проверяем на валидность данных
            if (currentFree == 0) {
                report += tasks[i].name + ": NO_DATA\n";
                continue;
            }
            
            // Если свободного места больше, чем размер стека - корректируем размер
            if (currentFree > tasks[i].stackSize) {
                tasks[i].stackSize = currentFree + 1024; // Добавляем буфер
                report += tasks[i].name + ": SIZE_CORRECTED (new_size=" + String(tasks[i].stackSize) + ")\n";
            }
            
            UBaseType_t used = tasks[i].stackSize - currentFree;
            float usagePercent = (float)used / tasks[i].stackSize * 100.0f;
            
            // Дополнительная проверка на разумность процента
            if (usagePercent < 0.0f || usagePercent > 100.0f) {
                report += tasks[i].name + ": ERROR (calc=" + String(usagePercent, 1) + 
                         "%, used=" + String(used) + "/" + String(tasks[i].stackSize) + ")\n";
                continue;
            }
            
            report += tasks[i].name + ": ";
            report += String(used) + "/" + String(tasks[i].stackSize);
            report += " (" + String(usagePercent, 1) + "%)";
            
            // Предупреждения
            if (usagePercent > 80.0f) {
                report += " ⚠️HIGH";
            } else if (usagePercent > 60.0f) {
                report += " ⚠️MED";
            }
            
            if (currentFree < 512) {
                report += " 🚨CRITICAL";
            }
            
            report += "\n";
        }
        
        return report;
    }
    
    // Проверить, есть ли критические проблемы со стеком
    bool hasCriticalStackUsage() {
        for (int i = 0; i < taskCount; i++) {
            if (!tasks[i].isValid) continue;

            UBaseType_t currentFree = uxTaskGetStackHighWaterMark(tasks[i].handle);
            UBaseType_t stackSize = tasks[i].stackSize;
            if (stackSize == 0) continue; // защита от деления на ноль

            // Корректируем, если currentFree > stackSize
            if (currentFree > stackSize) stackSize = currentFree;

            UBaseType_t used = stackSize - currentFree;
            float usagePercent = (float)used / stackSize * 100.0f;

            // Критично: >90% использования или <256 байт свободно
            if (usagePercent > 90.0f || currentFree < 256) {
                return true;
            }
        }
        return false;
    }
    
    // Получить строку с задачами с высоким использованием стека
    String getCriticalTasks() {
        String critical = "";
        
        for (int i = 0; i < taskCount; i++) {
            if (!tasks[i].isValid) continue;
            
            UBaseType_t currentFree = uxTaskGetStackHighWaterMark(tasks[i].handle);
            UBaseType_t used = tasks[i].stackSize - currentFree;
            float usagePercent = (float)used / tasks[i].stackSize * 100.0f;
            
            if (usagePercent > 80.0f || currentFree < 512) {
                if (critical.length() > 0) critical += ", ";
                critical += tasks[i].name + "(" + String(usagePercent, 0) + "%)";
            }
        }
        
        return critical;
    }
    
    // Получить общее использование памяти
    void getMemoryInfo(size_t& totalHeap, size_t& freeHeap, size_t& minFreeHeap) {
        totalHeap = ESP.getHeapSize();
        freeHeap = ESP.getFreeHeap();
        minFreeHeap = ESP.getMinFreeHeap();
    }
    
    // Получить компактную статистику для логов
    String getCompactStatus() {
        String status = "";
        int criticalCount = 0;
        
        for (int i = 0; i < taskCount; i++) {
            if (!tasks[i].isValid) continue;
            
            UBaseType_t currentFree = uxTaskGetStackHighWaterMark(tasks[i].handle);
            UBaseType_t used = tasks[i].stackSize - currentFree;
            float usagePercent = (float)used / tasks[i].stackSize * 100.0f;
            
            if (usagePercent > 80.0f) {
                criticalCount++;
            }
        }
        
        size_t totalHeap, freeHeap, minFreeHeap;
        getMemoryInfo(totalHeap, freeHeap, minFreeHeap);
        
        status = "Stack: " + String(criticalCount) + " high, ";
        status += "Heap: " + String(freeHeap/1024) + "KB/" + String(totalHeap/1024) + "KB";
        
        return status;
    }
    
private:
    // Вспомогательная функция для регистрации известных задач
    // Теперь возвращает bool: true если задача найдена и зарегистрирована, иначе false
    bool registerKnownTask(const String& name, UBaseType_t estimatedStackSize) {
        if (taskCount >= MAX_TASKS) return false;
        TaskHandle_t handle = xTaskGetHandle(name.c_str());
        if (handle != nullptr) {
            UBaseType_t currentFree = uxTaskGetStackHighWaterMark(handle);
            UBaseType_t actualStackSize = estimatedStackSize;
            if (currentFree > estimatedStackSize) {
                actualStackSize = currentFree + 512;
            }
            tasks[taskCount].handle = handle;
            tasks[taskCount].name = name;
            tasks[taskCount].stackSize = actualStackSize;
            tasks[taskCount].minFreeStack = actualStackSize;
            tasks[taskCount].isValid = true;
            taskCount++;
            return true;
        }
        return false;
    }
};
