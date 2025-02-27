#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#define VOLTAGE_SENSOR A0
#define CURRENT_SENSOR A1
#define TEMP_SENSOR A2
#define VIBRATION_SENSOR 10
#define RELAY_PIN 7
#define BUZZER 6
#define FAN_PIN 5  // Fan control
#define GSM_TX 8
#define GSM_RX 9

LiquidCrystal_I2C lcd(0x27, 20, 4);
SoftwareSerial gsm(GSM_TX, GSM_RX);

// Battery parameters
float batteryVoltage, batteryCurrent, temperature, SOC, SOH;
float Vmax = 12.6;
float Vmin = 9.6;
float nominalCapacity = 10.0;

// ACS712 Current Sensor calibration
float Vref = 2.5;
float sensitivity = 0.185;

void setup() {
    Serial.begin(9600);
    gsm.begin(9600);
    lcd.init();
    lcd.backlight();
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(VIBRATION_SENSOR, INPUT);
}

void sendSMS(String message) {
    gsm.println("AT+CMGF=1");
    delay(500);
    gsm.println("AT+CMGS=\"+254783138126\""); // Replace with user number
    delay(500);
    gsm.print(message);
    delay(500);
    gsm.write(26);
    delay(1000);
}

void loop() {
    batteryVoltage = analogRead(VOLTAGE_SENSOR) * (5.0 / 1023.0) * (12.6 / 5.0);
    int currentRaw = analogRead(CURRENT_SENSOR);
    float voltageAtSensor = currentRaw * (5.0 / 1023.0);
    batteryCurrent = (voltageAtSensor - Vref) / sensitivity;
    temperature = (analogRead(TEMP_SENSOR) * (5.0 / 1023.0)) * 100.0;
    SOC = ((batteryVoltage - Vmin) / (Vmax - Vmin)) * 100.0;
    SOC = constrain(SOC, 0, 100);
    SOH = (8.0 / nominalCapacity) * 100.0;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MOTOR PROTECTION SYS");
    lcd.setCursor(0, 1);
    lcd.print("CUR:");
    lcd.setCursor(4, 1);
    lcd.print(batteryCurrent, 2);
    lcd.print("A");
    lcd.setCursor(10, 1);
    lcd.print("TEMP:");
    lcd.setCursor(15, 1);
    lcd.print(temperature, 1);
    lcd.print("C");
    lcd.setCursor(0, 2);
    lcd.print("VOLTAGE:");
    lcd.setCursor(9, 2);
    lcd.print(batteryVoltage, 2);
    lcd.print("V");
    lcd.setCursor(0, 3);
    lcd.print("MOTOR:");
    lcd.setCursor(7, 3);
    lcd.print(digitalRead(RELAY_PIN) ? "ON " : "OFF");

    if (temperature > 60) {
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(BUZZER, HIGH);
        sendSMS("ALERT: High Temp! Motor Stopped.");
    } else {
        digitalWrite(RELAY_PIN, HIGH);
        digitalWrite(BUZZER, LOW);
    }

    if (batteryVoltage < 9.0) {
        digitalWrite(RELAY_PIN, LOW);
         digitalWrite(BUZZER, HIGH);
        sendSMS("ALERT: Low Voltage! Motor Stopped.");
    }

    if (digitalRead(VIBRATION_SENSOR) == HIGH) {
        digitalWrite(RELAY_PIN, LOW);
         digitalWrite(BUZZER, HIGH);
        sendSMS("ALERT: Vibration Detected! Motor Stopped.");
    }

    delay(1000);
}
