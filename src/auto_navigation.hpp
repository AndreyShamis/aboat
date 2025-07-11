#pragma once
#include <Arduino.h>
#include <vector>
#include "boat_utils.hpp"

// Автономная навигационная система
class AutoNavigation {
public:
    struct Waypoint {
        float latitude;
        float longitude;
        float targetHeading;  // Желаемый курс в этой точке
        float tolerance;      // Радиус достижения в метрах
        bool reached;
        
        Waypoint(float lat, float lon, float heading = 0, float tol = 10.0f) 
            : latitude(lat), longitude(lon), targetHeading(heading), tolerance(tol), reached(false) {}
    };
    
    enum NavigationMode {
        MANUAL,
        WAYPOINT_FOLLOWING,
        RETURN_TO_HOME,
        STATION_KEEPING
    };
    
private:
    std::vector<Waypoint> waypoints;
    size_t currentWaypointIndex = 0;
    NavigationMode mode = MANUAL;
    
    // Домашняя позиция
    float homeLat = 0.0f;
    float homeLon = 0.0f;
    bool homeSet = false;
    
    // PID контроллеры для управления
    struct PIDController {
        float kP, kI, kD;
        float lastError = 0.0f;
        float integral = 0.0f;
        unsigned long lastTime = 0;
        
        PIDController(float p = 1.0f, float i = 0.0f, float d = 0.0f) : kP(p), kI(i), kD(d) {}
        
        float update(float error) {
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            if (dt <= 0) return 0;
            
            integral += error * dt;
            float derivative = (error - lastError) / dt;
            
            float output = kP * error + kI * integral + kD * derivative;
            
            lastError = error;
            lastTime = now;
            
            return output;
        }
        
        void reset() {
            lastError = 0;
            integral = 0;
            lastTime = millis();
        }
    };
    
    PIDController headingPID;  // Для управления курсом
    PIDController speedPID;    // Для управления скоростью
    
    // Параметры навигации
    float maxSpeed = 1.0f;     // м/с
    float cruiseSpeed = 0.5f;  // м/с
    float approachDistance = 50.0f; // Дистанция начала снижения скорости
    
    // Приостановка навигации
    bool paused = false;
    NavigationMode previousMode = MANUAL;
    
public:
    AutoNavigation() : headingPID(2.0f, 0.1f, 0.5f), speedPID(1.0f, 0.05f, 0.1f) {}
    
    void setMode(NavigationMode newMode) {
        if (mode != newMode) {
            mode = newMode;
            headingPID.reset();
            speedPID.reset();
        }
    }
    
    NavigationMode getMode() const { return mode; }
    
    void setHome(float lat, float lon) {
        homeLat = lat;
        homeLon = lon;
        homeSet = true;
    }
    
    void addWaypoint(float lat, float lon, float heading = 0, float tolerance = 10.0f) {
        waypoints.emplace_back(lat, lon, heading, tolerance);
    }
    
    void clearWaypoints() {
        waypoints.clear();
        currentWaypointIndex = 0;
    }
    
    size_t getWaypointCount() const { return waypoints.size(); }
    size_t getCurrentWaypointIndex() const { return currentWaypointIndex; }
    
    // Установить единичную цель для навигации
    void setTarget(float latitude, float longitude) {
        clearWaypoints();
        addWaypoint(latitude, longitude);
        currentWaypointIndex = 0;
    }
    
    // Приостановить навигацию (сохранить состояние)
    void pause() {
        previousMode = mode;
        paused = true;
        mode = MANUAL;
    }
    
    // Возобновить навигацию
    void resume() {
        if (paused) {
            mode = previousMode;
            paused = false;
        }
    }
    
    bool isPaused() const { return paused; }
    
    // Основная функция навигации
    struct NavigationOutput {
        float rudderAngle;    // Угол руля (-90 до 90 градусов)
        float throttle;       // Положение газа (0-100%)
        bool navigationActive;
        String statusMessage;
    };
    
    NavigationOutput update(float currentLat, float currentLon, float currentHeading, float currentSpeed) {
        NavigationOutput output = {0, 0, false, "Manual mode"};
        
        if (mode == MANUAL) {
            return output;
        }
        
        output.navigationActive = true;
        
        float targetLat = 0, targetLon = 0;
        bool hasTarget = false;
        
        switch (mode) {
            case WAYPOINT_FOLLOWING:
                if (currentWaypointIndex < waypoints.size()) {
                    auto& wp = waypoints[currentWaypointIndex];
                    targetLat = wp.latitude;
                    targetLon = wp.longitude;
                    hasTarget = true;
                    
                    float distance = BoatUtils::calculateDistance(currentLat, currentLon, targetLat, targetLon);
                    if (distance <= wp.tolerance) {
                        wp.reached = true;
                        currentWaypointIndex++;
                        output.statusMessage = "Waypoint " + String(currentWaypointIndex) + " reached";
                        
                        if (currentWaypointIndex >= waypoints.size()) {
                            setMode(MANUAL);
                            output.statusMessage = "All waypoints reached";
                            return output;
                        }
                    } else {
                        output.statusMessage = "To WP" + String(currentWaypointIndex + 1) + ": " + String(distance, 1) + "m";
                    }
                }
                break;
                
            case RETURN_TO_HOME:
                if (homeSet) {
                    targetLat = homeLat;
                    targetLon = homeLon;
                    hasTarget = true;
                    
                    float distance = BoatUtils::calculateDistance(currentLat, currentLon, targetLat, targetLon);
                    if (distance <= 10.0f) {
                        setMode(STATION_KEEPING);
                        output.statusMessage = "Home reached";
                    } else {
                        output.statusMessage = "To Home: " + String(distance, 1) + "m";
                    }
                }
                break;
                
            case STATION_KEEPING:
                if (homeSet) {
                    targetLat = homeLat;
                    targetLon = homeLon;
                    hasTarget = true;
                    output.statusMessage = "Station keeping";
                }
                break;
        }
        
        if (hasTarget) {
            // Расчет требуемого курса
            float targetBearing = BoatUtils::calculateBearing(currentLat, currentLon, targetLat, targetLon);
            float headingError = BoatUtils::normalizeAngle(targetBearing - currentHeading);
            
            // Управление рулем
            float rudderCommand = headingPID.update(headingError);
            output.rudderAngle = BoatUtils::constrain_range(rudderCommand, -90.0f, 90.0f);
            
            // Управление скоростью
            float distance = BoatUtils::calculateDistance(currentLat, currentLon, targetLat, targetLon);
            float targetSpeed = cruiseSpeed;
            
            // Снижение скорости при приближении к цели
            if (distance < approachDistance) {
                targetSpeed = cruiseSpeed * (distance / approachDistance);
                targetSpeed = max(targetSpeed, 0.1f); // Минимальная скорость
            }
            
            if (mode == STATION_KEEPING && distance < 5.0f) {
                targetSpeed = 0.0f; // Остановка при достижении позиции
            }
            
            float speedError = targetSpeed - currentSpeed;
            float throttleCommand = speedPID.update(speedError);
            output.throttle = BoatUtils::constrain_range(throttleCommand * 100.0f, 0.0f, 100.0f);
        }
        
        return output;
    }
    
    // Получение информации о текущем состоянии
    String getStatusString() const {
        String status = "Nav: ";
        switch (mode) {
            case MANUAL: status += "MANUAL"; break;
            case WAYPOINT_FOLLOWING: 
                status += "WP " + String(currentWaypointIndex + 1) + "/" + String(waypoints.size());
                break;
            case RETURN_TO_HOME: status += "RTH"; break;
            case STATION_KEEPING: status += "STATION"; break;
        }
        return status;
    }
    
    // Настройка PID параметров
    void setHeadingPID(float p, float i, float d) {
        headingPID.kP = p;
        headingPID.kI = i;
        headingPID.kD = d;
    }
    
    void setSpeedPID(float p, float i, float d) {
        speedPID.kP = p;
        speedPID.kI = i;
        speedPID.kD = d;
    }
    
    void setSpeedLimits(float cruise, float max) {
        cruiseSpeed = cruise;
        maxSpeed = max;
    }
};
