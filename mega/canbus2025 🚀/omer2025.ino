#include <Canbus.h>
#include <mcp2515.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <SerLCD.h>

#define RXPIN 4
#define TXPIN 5
#define GPSBAUD 4800
#define SD_CS 10  // SD card chip select pin
#define CAN_ID_CURRENT 0x03B
#define CAN_ID_TEMP 0x123

SoftwareSerial uart_gps(RXPIN, TXPIN);
TinyGPS gps;
SerLCD lcd;
File logFile;
float latitude, longitude, distance, totalDist, power, MPGe;
unsigned long startTime;
int displayMode = 0;
unsigned long lastLcdUpdate = 0;

void setup() {
    Serial.begin(115200);
    uart_gps.begin(GPSBAUD);
    lcd.begin(Wire);
    lcd.clear();
    Wire.begin();
    
    if (!SD.begin(SD_CS)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    
    startTime = millis();
    String fileName = generateFileName();
    logFile = SD.open(fileName, FILE_WRITE);
    if (logFile) {
        logFile.println("Timestamp, Latitude, Longitude, Speed (mph), Voltage (V), Current (A), State of Charge (%), Min Cell Voltage (V), Max Cell Voltage (V), Max Battery Temp (°C), Distance (miles), Energy (kWh), MPGe");
        logFile.close();
    }
}

void loop() {
    unsigned long timestamp = millis() - startTime;
    if (uart_gps.available()) {
        char c = uart_gps.read();
        if (gps.encode(c)) {
            gps.f_get_position(&latitude, &longitude);
            distance = calculateDistance(latitude, longitude);
            totalDist += distance;
        }
    }
    
    float speed = gps.f_speed_mph();
    float voltage, current;
    getCANData(voltage, current);
    float stateOfCharge = getStateOfCharge();
    float minCellVoltage = getLowestCellVoltage();
    float maxCellVoltage = getHighestCellVoltage();
    float maxBatteryTemp = getHighestBatteryTemp();
    float energy = calculateEnergy();
    float mpge = calculateMPGe();
    
    logData(timestamp, latitude, longitude, speed, voltage, current, stateOfCharge, minCellVoltage, maxCellVoltage, maxBatteryTemp, totalDist, energy, mpge);
    updateLcdDisplay(speed, voltage, current, stateOfCharge, totalDist, energy, minCellVoltage, maxBatteryTemp, mpge);
    delay(1000);
}

void getCANData(float &voltage, float &current) {
    tCAN message;
    if (mcp2515_check_message() && mcp2515_get_message(&message)) {
        if (message.id == CAN_ID_CURRENT) {
            current = (message.data[0] * 256 + message.data[1]) / 10.0;
            voltage = message.data[3];
        }
    } else {
        current = 0.0;
        voltage = 0.0;
    }
}

String generateFileName() {
    char filename[20];
    sprintf(filename, "log_%lu.csv", millis() / 1000);
    return String(filename);
}

void logData(unsigned long timestamp, float lat, float lon, float speed, float voltage, float current, float soc, float minCellV, float maxCellV, float maxTemp, float distance, float energy, float mpge) {
    logFile = SD.open(generateFileName(), FILE_WRITE);
    if (logFile) {
        logFile.print(timestamp);
        logFile.print(",");
        logFile.print(lat, 6);
        logFile.print(",");
        logFile.print(lon, 6);
        logFile.print(",");
        logFile.print(speed);
        logFile.print(",");
        logFile.print(voltage);
        logFile.print(",");
        logFile.print(current);
        logFile.print(",");
        logFile.print(soc);
        logFile.print(",");
        logFile.print(minCellV);
        logFile.print(",");
        logFile.print(maxCellV);
        logFile.print(",");
        logFile.print(maxTemp);
        logFile.print(",");
        logFile.print(distance);
        logFile.print(",");
        logFile.print(energy);
        logFile.print(",");
        logFile.println(mpge);
        logFile.close();
    }
}

void updateLcdDisplay(float speed, float voltage, float current, float soc, float distance, float energy, float minCellV, float maxTemp, float mpge) {
    if (millis() - lastLcdUpdate > 3000) {
        lcd.clear();
        if (displayMode == 0) {
            lcd.setCursor(0, 0);
            lcd.print("Speed:"); lcd.print(speed); lcd.print(" mph");
            lcd.setCursor(0, 1);
            lcd.print("Voltage:"); lcd.print(voltage); lcd.print("V");
            lcd.setCursor(0, 2);
            lcd.print("Current:"); lcd.print(current); lcd.print("A");
            lcd.setCursor(0, 3);
            lcd.print("SoC:"); lcd.print(soc); lcd.print("%");
        } else if (displayMode == 1) {
            lcd.setCursor(0, 0);
            lcd.print("Energy:"); lcd.print(energy, 2); lcd.print(" kWh");
            lcd.setCursor(0, 1);
            lcd.print("Distance:"); lcd.print(distance, 2); lcd.print(" mi");
            lcd.setCursor(0, 2);
            lcd.print("Min Cell V:"); lcd.print(minCellV); lcd.print("V");
            lcd.setCursor(0, 3);
            lcd.print("Max Temp:"); lcd.print(maxTemp); lcd.print("C");
        } else {
            lcd.setCursor(0, 0);
            lcd.print("MPGe:"); lcd.print(mpge);
            lcd.setCursor(0, 1);
            lcd.print("Balancing: Active");
            lcd.setCursor(0, 2);
            lcd.print("Errors: None");
            lcd.setCursor(0, 3);
            lcd.print("Time: "); lcd.print(millis() / 1000);
        }
        displayMode = (displayMode + 1) % 3;
        lastLcdUpdate = millis();
    }
}
