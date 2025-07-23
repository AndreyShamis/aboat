#ifndef IMUMODULE_HPP
#define IMUMODULE_HPP

#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Fusion.h>
#include <ArduinoJson.h>
#include "../src/LogInterface.hpp"

/**
 * @brief Модуль для работы с 9-осевым сенсором MPU9250
 * 
 * Этот модуль инкапсулирует работу с:
 * - Акселерометром (измерение ускорения)
 * - Гироскопом (измерение угловой скорости)
 * - Магнетометром (измерение магнитного поля)
 * 
 * Использует библиотеку Fusion для обработки данных и получения:
 * - Курса (Yaw) - поворот вокруг вертикальной оси
 * - Крена (Roll) - поворот вокруг продольной оси
 * - Тангажа (Pitch) - поворот вокруг поперечной оси
 */
class IMUModule {
public:
    /**
     * @brief Структура для хранения данных ориентации
     */
    struct Orientation {
        float roll;     // Крен (градусы)
        float pitch;    // Тангаж (градусы) 
        float yaw;      // Курс (градусы)
        
        Orientation() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
        Orientation(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
    };

    /**
     * @brief Структура для хранения сырых данных с сенсоров
     */
    struct RawData {
        float accelX, accelY, accelZ;    // Ускорения (g)
        float gyroX, gyroY, gyroZ;       // Угловые скорости (°/с)
        float magX, magY, magZ;          // Магнитное поле (µT)
        
        RawData() : accelX(0), accelY(0), accelZ(0),
                   gyroX(0), gyroY(0), gyroZ(0),
                   magX(0), magY(0), magZ(0) {}
    };

private:
    MPU9250_asukiaaa mpu_;               // Объект для работы с MPU9250
    FusionAhrs ahrs_;                    // Фильтр Fusion AHRS
    LogInterface* logger_;               // Интерфейс для логирования
    
    bool initialized_;                   // Флаг инициализации
    unsigned long lastUpdateTime_;       // Время последнего обновления
    unsigned long updateInterval_;       // Интервал обновления (мс)
    
    Orientation currentOrientation_;     // Текущая ориентация
    RawData currentRawData_;            // Текущие сырые данные
    
    // Настройки фильтра
    float sampleRate_;                   // Частота обновления (Гц)
    float gyroscopeError_;              // Ошибка гироскопа (°/с)
    float accelerometerError_;          // Ошибка акселерометра
    float magnetometerError_;           // Ошибка магнетометра
    
    // Калибровочные данные
    bool calibrationActive_;
    int calibrationSamples_;
    float gyroOffsetX_, gyroOffsetY_, gyroOffsetZ_;

public:
    /**
     * @brief Конструктор модуля IMU
     * @param logger Указатель на интерфейс логирования
     * @param sampleRate Частота обновления в Гц (по умолчанию 100 Гц)
     */
    explicit IMUModule(LogInterface* logger = nullptr, float sampleRate = 100.0f);
    
    /**
     * @brief Деструктор
     */
    ~IMUModule() = default;

    /**
     * @brief Инициализация модуля IMU
     * @param wire Указатель на объект Wire для I2C
     * @param sdaPin Пин SDA для I2C (опционально)
     * @param sclPin Пин SCL для I2C (опционально)
     * @return true если инициализация успешна
     */
    bool begin(TwoWire* wire = &Wire, int sdaPin = -1, int sclPin = -1);
    
    /**
     * @brief Обновление данных с сенсоров и фильтра
     * Должна вызываться регулярно в основном цикле
     * @return true если данные обновлены
     */
    bool update();
    
    /**
     * @brief Принудительное обновление данных
     * @return true если данные обновлены
     */
    bool forceUpdate();
    
    /**
     * @brief Получить текущую ориентацию
     * @return Структура с углами ориентации
     */
    const Orientation& getOrientation() const { return currentOrientation_; }
    
    /**
     * @brief Получить текущие сырые данные
     * @return Структура с сырыми данными сенсоров
     */
    const RawData& getRawData() const { return currentRawData_; }
    
    /**
     * @brief Получить курс (Yaw) в градусах
     * @return Курс в диапазоне [-180, 180]
     */
    float getYaw() const { return currentOrientation_.yaw; }
    
    /**
     * @brief Получить крен (Roll) в градусах
     * @return Крен в диапазоне [-180, 180]
     */
    float getRoll() const { return currentOrientation_.roll; }
    
    /**
     * @brief Получить тангаж (Pitch) в градусах
     * @return Тангаж в диапазоне [-90, 90]
     */
    float getPitch() const { return currentOrientation_.pitch; }
    
    /**
     * @brief Получить кватернион ориентации
     * @return Кватернион из фильтра Fusion
     */
    FusionQuaternion getQuaternion() const;
    
    /**
     * @brief Проверить статус инициализации
     * @return true если модуль инициализирован
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Запустить калибровку гироскопа
     * @param samples Количество образцов для калибровки (по умолчанию 1000)
     */
    void startGyroCalibration(int samples = 1000);
    
    /**
     * @brief Проверить статус калибровки
     * @return true если калибровка активна
     */
    bool isCalibrating() const { return calibrationActive_; }
    
    /**
     * @brief Сбросить ориентацию (обнулить фильтр)
     */
    void reset();
    
    /**
     * @brief Установить интервал обновления
     * @param intervalMs Интервал в миллисекундах
     */
    void setUpdateInterval(unsigned long intervalMs) { updateInterval_ = intervalMs; }
    
    /**
     * @brief Получить интервал обновления
     * @return Интервал в миллисекундах
     */
    unsigned long getUpdateInterval() const { return updateInterval_; }
    
    /**
     * @brief Экспорт данных в JSON
     * @param jsonObj Объект JSON для записи данных
     */
    void toJSON(JsonObject& jsonObj) const;
    
    /**
     * @brief Получить строковое представление ориентации
     * @return Строка с углами ориентации
     */
    String getOrientationString() const;
    
    /**
     * @brief Получить диагностическую информацию
     * @return Строка с диагностической информацией
     */
    String getDiagnosticInfo() const;

    /**
     * @brief Установить интерфейс логирования
     * @param logger Указатель на интерфейс логирования
     */
    void setLogger(LogInterface* logger) { logger_ = logger; }

private:
    /**
     * @brief Чтение данных с сенсоров
     * @return true если чтение успешно
     */
    bool readSensors();
    
    /**
     * @brief Обновление фильтра Fusion
     */
    void updateFusion();
    
    /**
     * @brief Применение калибровочных данных
     */
    void applyCalibration();
    
    /**
     * @brief Логирование сообщения
     * @param message Сообщение для логирования
     */
    void log(const String& message);
};

#endif // IMUMODULE_HPP
