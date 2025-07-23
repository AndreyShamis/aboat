#include "IMUModule.hpp"

IMUModule::IMUModule(LogInterface* logger, float sampleRate) 
    : logger_(logger),
      initialized_(false),
      lastUpdateTime_(0),
      updateInterval_(10), // 10ms = 100Hz по умолчанию
      sampleRate_(sampleRate),
      gyroscopeError_(2.0f),      // 2°/с ошибка гироскопа
      accelerometerError_(0.5f),   // Ошибка акселерометра
      magnetometerError_(0.5f),    // Ошибка магнетометра
      calibrationActive_(false),
      calibrationSamples_(0),
      gyroOffsetX_(0.0f),
      gyroOffsetY_(0.0f),
      gyroOffsetZ_(0.0f)
{
    // Конструктор намеренно пустой - вся инициализация перенесена в begin()
    // чтобы избежать проблем с Serial во время глобальной инициализации
}

bool IMUModule::begin(TwoWire* wire, int sdaPin, int sclPin) {
    log("[IMU] Initializing MPU9250...");
    
    // Инициализация фильтра Fusion
    FusionAhrsInitialise(&ahrs_);
    
    // Настройка алгоритма Fusion
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNed,
        .gain = 0.5f,                    // Коэффициент усиления
        .gyroscopeRange = 2000.0f,       // Диапазон гироскопа (°/с)
        .accelerationRejection = 10.0f,  // Отклонение ускорения (g)
        .magneticRejection = 20.0f,      // Отклонение магнитного поля (µT)
        .recoveryTriggerPeriod = (unsigned int)(5 * sampleRate_), // 5 секунд
    };
    FusionAhrsSetSettings(&ahrs_, &settings);
    log("[IMU] Fusion AHRS filter initialized with sample rate: " + String(sampleRate_) + " Hz");
    
    // Настройка I2C если указаны пины
    if (sdaPin >= 0 && sclPin >= 0) {
        wire->begin(sdaPin, sclPin);
        log("[IMU] I2C initialized with SDA=" + String(sdaPin) + ", SCL=" + String(sclPin));
    }
    
    // Установка интерфейса I2C для MPU
    mpu_.setWire(wire);
    
    // Инициализация компонентов MPU9250
    mpu_.beginAccel();
    log("[IMU] Accelerometer initialized");
    
    mpu_.beginGyro();
    log("[IMU] Gyroscope initialized");
    
    mpu_.beginMag();
    log("[IMU] Magnetometer initialized");
    
    // Небольшая задержка для стабилизации
    delay(100);
    
    // Проверочное чтение
    if (!readSensors()) {
        log("[IMU] ERROR: Failed to read sensors during initialization");
        return false;
    }
    
    initialized_ = true;
    lastUpdateTime_ = millis();
    
    log("[IMU] ✅ MPU9250 successfully initialized");
    log("[IMU] Accel: ✅ Gyro: ✅ Mag: ✅");
    log("[IMU] Sample rate: " + String(sampleRate_) + " Hz");
    log("[IMU] Update interval: " + String(updateInterval_) + " ms");
    
    return true;
}

bool IMUModule::update() {
    if (!initialized_) {
        return false;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime_ < updateInterval_) {
        return false; // Еще рано обновляться
    }
    
    return forceUpdate();
}

bool IMUModule::forceUpdate() {
    if (!initialized_) {
        return false;
    }
    
    // Чтение данных с сенсоров
    if (!readSensors()) {
        return false;
    }
    
    // Применение калибровки
    applyCalibration();
    
    // Обновление фильтра Fusion
    updateFusion();
    
    lastUpdateTime_ = millis();
    return true;
}

bool IMUModule::readSensors() {
    // Чтение акселерометра
    mpu_.accelUpdate();
    currentRawData_.accelX = mpu_.accelX();
    currentRawData_.accelY = mpu_.accelY();
    currentRawData_.accelZ = mpu_.accelZ();
    
    // Чтение гироскопа
    mpu_.gyroUpdate();
    currentRawData_.gyroX = mpu_.gyroX();
    currentRawData_.gyroY = mpu_.gyroY();
    currentRawData_.gyroZ = mpu_.gyroZ();
    
    // Чтение магнетометра
    mpu_.magUpdate();
    currentRawData_.magX = mpu_.magX();
    currentRawData_.magY = mpu_.magY();
    currentRawData_.magZ = mpu_.magZ();
    
    return true;
}

void IMUModule::updateFusion() {
    // Подготовка данных для фильтра Fusion
    const FusionVector gyroscope = {
        currentRawData_.gyroX - gyroOffsetX_,
        currentRawData_.gyroY - gyroOffsetY_,
        currentRawData_.gyroZ - gyroOffsetZ_
    };
    
    const FusionVector accelerometer = {
        currentRawData_.accelX,
        currentRawData_.accelY,
        currentRawData_.accelZ
    };
    
    const FusionVector magnetometer = {
        currentRawData_.magX,
        currentRawData_.magY,
        currentRawData_.magZ
    };
    
    // Обновление фильтра с временным интервалом
    float deltaTime = updateInterval_ / 1000.0f; // Преобразование в секунды
    FusionAhrsUpdateNoMagnetometer(&ahrs_, gyroscope, accelerometer, deltaTime);
    
    // Обновление с магнетометром (если данные валидные)
    if (abs(currentRawData_.magX) > 1.0f || 
        abs(currentRawData_.magY) > 1.0f || 
        abs(currentRawData_.magZ) > 1.0f) {
        FusionAhrsUpdate(&ahrs_, gyroscope, accelerometer, magnetometer, deltaTime);
    }
    
    // Получение углов Эйлера
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_));
    
    // Обновление текущей ориентации
    currentOrientation_.roll = euler.angle.roll;
    currentOrientation_.pitch = euler.angle.pitch;
    currentOrientation_.yaw = euler.angle.yaw;
}

void IMUModule::applyCalibration() {
    // Процесс калибровки гироскопа
    if (calibrationActive_) {
        static float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
        static int sampleCount = 0;
        
        gyroSumX += currentRawData_.gyroX;
        gyroSumY += currentRawData_.gyroY;
        gyroSumZ += currentRawData_.gyroZ;
        sampleCount++;
        
        if (sampleCount >= calibrationSamples_) {
            // Вычисление средних смещений
            gyroOffsetX_ = gyroSumX / sampleCount;
            gyroOffsetY_ = gyroSumY / sampleCount;
            gyroOffsetZ_ = gyroSumZ / sampleCount;
            
            log("[IMU] ✅ Gyro calibration completed");
            log("[IMU] Offsets - X: " + String(gyroOffsetX_, 3) + 
                " Y: " + String(gyroOffsetY_, 3) + 
                " Z: " + String(gyroOffsetZ_, 3));
            
            // Сброс калибровки
            calibrationActive_ = false;
            gyroSumX = gyroSumY = gyroSumZ = 0;
            sampleCount = 0;
        } else {
            // Показываем прогресс каждые 100 образцов
            if (sampleCount % 100 == 0) {
                log("[IMU] Calibration progress: " + String(sampleCount) + "/" + String(calibrationSamples_));
            }
        }
    }
}

void IMUModule::startGyroCalibration(int samples) {
    if (!initialized_) {
        log("[IMU] ERROR: Cannot calibrate - module not initialized");
        return;
    }
    
    calibrationSamples_ = samples;
    calibrationActive_ = true;
    
    log("[IMU] 🔧 Starting gyro calibration with " + String(samples) + " samples");
    log("[IMU] ⚠️ Keep IMU stationary during calibration!");
}

void IMUModule::reset() {
    if (initialized_) {
        FusionAhrsInitialise(&ahrs_);
        log("[IMU] 🔄 AHRS filter reset");
    }
}

FusionQuaternion IMUModule::getQuaternion() const {
    if (initialized_) {
        return FusionAhrsGetQuaternion(&ahrs_);
    }
    
    // Возвращаем единичный кватернион если не инициализирован
    const FusionQuaternion identity = {1.0f, 0.0f, 0.0f, 0.0f};
    return identity;
}

void IMUModule::toJSON(JsonObject& jsonObj) const {
    // Ориентация
    JsonObject orientation = jsonObj["orientation"].to<JsonObject>();
    orientation["roll"] = currentOrientation_.roll;
    orientation["pitch"] = currentOrientation_.pitch;
    orientation["yaw"] = currentOrientation_.yaw;
    
    // Сырые данные акселерометра
    JsonObject accel = jsonObj["accelerometer"].to<JsonObject>();
    accel["x"] = currentRawData_.accelX;
    accel["y"] = currentRawData_.accelY;
    accel["z"] = currentRawData_.accelZ;
    
    // Сырые данные гироскопа
    JsonObject gyro = jsonObj["gyroscope"].to<JsonObject>();
    gyro["x"] = currentRawData_.gyroX;
    gyro["y"] = currentRawData_.gyroY;
    gyro["z"] = currentRawData_.gyroZ;
    
    // Сырые данные магнетометра
    JsonObject mag = jsonObj["magnetometer"].to<JsonObject>();
    mag["x"] = currentRawData_.magX;
    mag["y"] = currentRawData_.magY;
    mag["z"] = currentRawData_.magZ;
    
    // Статус
    JsonObject status = jsonObj["status"].to<JsonObject>();
    status["initialized"] = initialized_;
    status["calibrating"] = calibrationActive_;
    status["sample_rate"] = sampleRate_;
    status["update_interval"] = updateInterval_;
    
    // Калибровочные данные
    JsonObject calibration = jsonObj["calibration"].to<JsonObject>();
    calibration["gyro_offset_x"] = gyroOffsetX_;
    calibration["gyro_offset_y"] = gyroOffsetY_;
    calibration["gyro_offset_z"] = gyroOffsetZ_;
}

String IMUModule::getOrientationString() const {
    return "Roll: " + String(currentOrientation_.roll, 1) + "° " +
           "Pitch: " + String(currentOrientation_.pitch, 1) + "° " +
           "Yaw: " + String(currentOrientation_.yaw, 1) + "°";
}

String IMUModule::getDiagnosticInfo() const {
    String info = "[IMU] Status: ";
    info += initialized_ ? "✅ OK" : "❌ NOT_INIT";
    
    if (calibrationActive_) {
        info += " 🔧 CALIBRATING";
    }
    
    info += " | " + getOrientationString();
    info += " | Freq: " + String(1000.0f / updateInterval_, 1) + "Hz";
    
    return info;
}

void IMUModule::log(const String& message) {
    if (logger_) {
        logger_->addLog(message);
    }
}
