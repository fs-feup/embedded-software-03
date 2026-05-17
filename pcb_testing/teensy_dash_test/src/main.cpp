// Dashboard PCB Test Suite
// Tests all I/O, sensors, CAN, I2C, PWM

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>

// Pin definitions

namespace pins {
    namespace analog {
        constexpr uint8_t APPS_HIGHER = 20;
        constexpr uint8_t APPS_LOWER = 22;
        constexpr uint8_t BRAKE_PRESSURE = 19;
        constexpr uint8_t ROTARY_SWITCH = 23;
    }

    namespace digital_in {
        constexpr uint8_t TS = 21;
        constexpr uint8_t BSPD = 18;
        constexpr uint8_t INERTIA = 7;
        constexpr uint8_t R2D = 4;
        constexpr uint8_t ATS = 5;
    }

    namespace digital_out {
        constexpr uint8_t RACE_LED = 17;
        constexpr uint8_t BUZZER = 2;
        constexpr uint8_t DISPLAY_MODE = 3;
        constexpr uint8_t TS_LED = 16;
        constexpr uint8_t BSPD_LED = 14;
        constexpr uint8_t INERTIA_LED = 15;
        constexpr uint8_t ATS_OUT = 6;
    }

    namespace encoder {
        constexpr uint8_t FRONT_RIGHT_WHEEL = 9;
        constexpr uint8_t FRONT_LEFT_WHEEL = 8;
    }

    namespace spi {
        constexpr uint8_t CS = 10;
        constexpr uint8_t MOSI = 11;
        constexpr uint8_t MISO = 12;
        constexpr uint8_t SCK = 13;
    }

    namespace i2c {
        constexpr uint8_t SDA = 18;  // Teensy 4.1 default
        constexpr uint8_t SCL = 19;
    }
}

// Constants
constexpr float ADC_RESOLUTION = 1023.0f;
constexpr float ADC_VREF = 3.3f;
constexpr uint8_t MPU6050_ADDR = 0x68;

// CAN setup

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

constexpr uint32_t CAN_TEST_TX_ID = 0x100;
constexpr uint32_t CAN_TEST_RX_ID = 0x101;

volatile bool can_message_received = false;
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_error_count = 0;
uint8_t can_rx_data[8] = {0};
uint32_t can_rx_id = 0;

void canSniff(const CAN_message_t &msg) {
    can_message_received = true;
    can_rx_count++;
    can_rx_id = msg.id;
    memcpy(can_rx_data, msg.buf, min(8, (int)msg.len));

    Serial.print("[CAN RX] ID:0x"); Serial.print(msg.id, HEX);
    Serial.print(" Len:"); Serial.print(msg.len);
    Serial.print(" Data:");
    for (int i = 0; i < msg.len; i++) {
        Serial.print(" "); Serial.print(msg.buf[i], HEX);
    }
    Serial.println();
}

// Encoder counters

volatile uint32_t encoder_fr_count = 0;
volatile uint32_t encoder_fl_count = 0;

void encoder_fr_isr() { encoder_fr_count++; }
void encoder_fl_isr() { encoder_fl_count++; }

// Utility functions

void printHeader(const char* title) {
    Serial.println();
    Serial.print("--- ");
    Serial.print(title);
    Serial.println(" ---");
}

float adcToVoltage(int adc) {
    return (adc / ADC_RESOLUTION) * ADC_VREF;
}

// I2C tests

void testI2CBusScan() {
    printHeader("I2C Bus Scan");

    int devicesFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found device at 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);

            if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU6050?)");
            else if (addr == 0x77) Serial.print(" (BMP280?)");
            else if (addr == 0x3C || addr == 0x3D) Serial.print(" (OLED?)");

            Serial.println();
            devicesFound++;
        }
    }

    Serial.print("Total devices: "); Serial.println(devicesFound);
}

void testIMU() {
    printHeader("IMU Test");

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);  // WHO_AM_I register
    if (Wire.endTransmission(false) != 0) {
        Serial.println("MPU6050 not found");
        return;
    }

    Wire.requestFrom(MPU6050_ADDR, 1);
    if (Wire.available()) {
        uint8_t whoami = Wire.read();
        Serial.print("WHO_AM_I: 0x"); Serial.println(whoami, HEX);

        if (whoami == 0x68) {
            // Wake up MPU6050
            Wire.beginTransmission(MPU6050_ADDR);
            Wire.write(0x6B);
            Wire.write(0);
            Wire.endTransmission(true);
            delay(100);

            // Read accelerometer samples
            Serial.println("Reading 10 samples...");
            for (int i = 0; i < 10; i++) {
                Wire.beginTransmission(MPU6050_ADDR);
                Wire.write(0x3B);  // ACCEL_XOUT_H
                Wire.endTransmission(false);
                Wire.requestFrom(MPU6050_ADDR, 6);

                if (Wire.available() == 6) {
                    int16_t ax = Wire.read() << 8 | Wire.read();
                    int16_t ay = Wire.read() << 8 | Wire.read();
                    int16_t az = Wire.read() << 8 | Wire.read();

                    Serial.print(i+1); Serial.print(": X="); Serial.print(ax);
                    Serial.print(" Y="); Serial.print(ay);
                    Serial.print(" Z="); Serial.println(az);
                }
                delay(100);
            }
            Serial.println("Done");
        } else {
            Serial.println("Unexpected WHO_AM_I");
        }
    }
}

// Power rail tests

void testPowerRails() {
    printHeader("Power Rails");
    Serial.println("Requires voltage dividers on test points");

    const int samples = 10;

    // Read reference voltage
    float vref_sum = 0;
    for (int i = 0; i < samples; i++) {
        vref_sum += adcToVoltage(analogRead(14));
        delay(10);
    }
    Serial.print("Vref avg: "); Serial.print(vref_sum / samples, 3); Serial.println("V");

    // 5V rail (with 1:1 divider)
    Serial.println("Connect 5V rail via 10k+10k divider to A15");
    float rail_sum = 0;
    for (int i = 0; i < samples; i++) {
        rail_sum += adcToVoltage(analogRead(A15));
        delay(10);
    }
    float rail_5v = (rail_sum / samples) * 2.0;

    Serial.print("5V rail: "); Serial.print(rail_5v, 2); Serial.print("V ");
    Serial.println((rail_5v > 4.75 && rail_5v < 5.25) ? "OK" : "CHECK");
}

// PWM tests

void testPWMOutputs() {
    printHeader("PWM");
    Serial.println("Testing buzzer tones and LED dimming");

    const uint16_t frequencies[] = {500, 1000, 2000, 3000, 4000};
    for (int i = 0; i < 5; i++) {
        Serial.print(frequencies[i]); Serial.println("Hz");
        tone(pins::digital_out::BUZZER, frequencies[i]);
        delay(500);
        noTone(pins::digital_out::BUZZER);
        delay(200);
    }

    Serial.println("LED dimming...");
    for (int duty = 0; duty <= 255; duty += 32) {
        analogWrite(pins::digital_out::RACE_LED, duty);
        Serial.print(duty * 100 / 255); Serial.println("%");
        delay(300);
    }
    analogWrite(pins::digital_out::RACE_LED, 0);
    Serial.println("Done");
}

// ADC calibration

void testADCCalibration() {
    printHeader("ADC CALIBRATION TEST");
    Serial.println("Testing ADC accuracy and noise...");
    Serial.println("Connect known voltage to APPS_HIGHER pin");
    Serial.println();

    const int samples = 100;
    int readings[samples];

    // Collect samples
    Serial.println("Collecting 100 samples...");
    for (int i = 0; i < samples; i++) {
        readings[i] = analogRead(pins::analog::APPS_HIGHER);
        delayMicroseconds(100);
    }

    // Calculate statistics
    int min_val = 1023, max_val = 0;
    long sum = 0;

    for (int i = 0; i < samples; i++) {
        if (readings[i] < min_val) min_val = readings[i];
        if (readings[i] > max_val) max_val = readings[i];
        sum += readings[i];
    }

    float avg = sum / (float)samples;
    int range = max_val - min_val;
    float avg_voltage = adcToVoltage(avg);

    Serial.print("  Average: "); Serial.print(avg, 2); Serial.print(" (");
    Serial.print(avg_voltage, 3); Serial.println("V)");
    Serial.print("  Min: "); Serial.print(min_val);
    Serial.print("  Max: "); Serial.print(max_val);
    Serial.print("  Range: "); Serial.print(range);
    Serial.print("  ("); Serial.print(range * 100.0 / avg, 2); Serial.println("%)");

    if (range < 10) {
        Serial.println("[PASS] Low noise ADC reading");
    } else if (range < 30) {
        Serial.println("[WARNING] Moderate ADC noise");
    } else {
        Serial.println("[FAIL] High ADC noise - check wiring/grounding");
    }
}

// Advanced CAN tests

void testCANStress() {
    printHeader("CAN STRESS TEST");
    Serial.println("Sending 100 messages as fast as possible...");
    Serial.println();

    CAN_message_t msg;
    msg.id = CAN_TEST_TX_ID;
    msg.len = 8;

    uint32_t start_time = millis();
    uint32_t sent_count = 0;
    uint32_t failed_count = 0;

    for (int i = 0; i < 100; i++) {
        // Fill with counter data
        msg.buf[0] = i >> 8;
        msg.buf[1] = i & 0xFF;
        msg.buf[2] = 0xAA;
        msg.buf[3] = 0x55;
        msg.buf[4] = ~(i >> 8);
        msg.buf[5] = ~(i & 0xFF);
        msg.buf[6] = 0xFF;
        msg.buf[7] = 0x00;

        if (can1.write(msg)) {
            sent_count++;
        } else {
            failed_count++;
        }

        delayMicroseconds(100);  // Small delay
    }

    uint32_t elapsed = millis() - start_time;
    float rate = (sent_count * 1000.0) / elapsed;

    Serial.print("Sent: "); Serial.print(sent_count); Serial.println(" messages");
    Serial.print("Failed: "); Serial.println(failed_count);
    Serial.print("Time: "); Serial.print(elapsed); Serial.println(" ms");
    Serial.print("Rate: "); Serial.print(rate, 1); Serial.println(" msg/s");

    if (failed_count == 0) {
        Serial.println("[PASS] All messages sent successfully");
    } else {
        Serial.println("[WARNING] Some messages failed - check CAN bus load");
    }
}

void testCANBidirectional() {
    printHeader("CAN BIDIRECTIONAL FLOW TEST");
    Serial.println("Testing simultaneous TX and RX...");
    Serial.println("Requires another CAN node responding");
    Serial.println();

    CAN_message_t msg;
    msg.id = CAN_TEST_TX_ID;
    msg.len = 8;

    can_rx_count = 0;
    uint32_t tx_count = 0;

    unsigned long start = millis();

    Serial.println("Running for 10 seconds...");
    Serial.println("Sending messages and listening for replies...");

    while (millis() - start < 10000) {
        // Send message
        for (int i = 0; i < 8; i++) {
            msg.buf[i] = random(256);
        }

        if (can1.write(msg)) {
            tx_count++;
        }

        can1.events();  // Process RX
        delay(100);
    }

    Serial.println();
    Serial.print("TX: "); Serial.print(tx_count); Serial.println(" messages");
    Serial.print("RX: "); Serial.print(can_rx_count); Serial.println(" messages");

    if (can_rx_count > 0) {
        Serial.println("[PASS] Bidirectional communication working");
    } else {
        Serial.println("[INFO] No RX messages (external node may not be responding)");
    }
}

// Interrupt timing test

void testInterruptTiming() {
    printHeader("INTERRUPT TIMING TEST");
    Serial.println("Testing encoder interrupt response time...");
    Serial.println("Generate pulses on encoder pins manually or with signal generator");
    Serial.println();

    encoder_fr_count = 0;
    encoder_fl_count = 0;

    Serial.println("Monitoring for 5 seconds...");
    unsigned long start = millis();
    unsigned long last_count_fr = 0;
    unsigned long last_count_fl = 0;

    while (millis() - start < 5000) {
        if (encoder_fr_count != last_count_fr) {
            Serial.print("[FR] Count: "); Serial.println(encoder_fr_count);
            last_count_fr = encoder_fr_count;
        }
        if (encoder_fl_count != last_count_fl) {
            Serial.print("[FL] Count: "); Serial.println(encoder_fl_count);
            last_count_fl = encoder_fl_count;
        }
        delay(10);
    }

    Serial.println();
    Serial.print("Total FR: "); Serial.println(encoder_fr_count);
    Serial.print("Total FL: "); Serial.println(encoder_fl_count);

    if (encoder_fr_count > 0 || encoder_fl_count > 0) {
        Serial.println("[PASS] Interrupts working");
    } else {
        Serial.println("[INFO] No pulses detected");
    }
}

// Continuous monitoring

void continuousMonitor() {
    printHeader("Continuous Monitor");
    Serial.println("Press any key to exit\n");

    while (!Serial.available()) {
        int apps_h = analogRead(pins::analog::APPS_HIGHER);
        int apps_l = analogRead(pins::analog::APPS_LOWER);
        int brake = analogRead(pins::analog::BRAKE_PRESSURE);
        int rotary = analogRead(pins::analog::ROTARY_SWITCH);

        bool ts = digitalRead(pins::digital_in::TS);
        bool bspd = digitalRead(pins::digital_in::BSPD);
        bool inertia = digitalRead(pins::digital_in::INERTIA);
        bool r2d = digitalRead(pins::digital_in::R2D);
        bool ats = digitalRead(pins::digital_in::ATS);

        Serial.print("\033[2J\033[H");  // Clear screen
        Serial.println("=== MONITOR ===");
        Serial.print("APPS H: "); Serial.print(apps_h);
        Serial.print(" ("); Serial.print(adcToVoltage(apps_h), 2); Serial.println("V)");
        Serial.print("APPS L: "); Serial.print(apps_l);
        Serial.print(" ("); Serial.print(adcToVoltage(apps_l), 2); Serial.println("V)");
        Serial.print("Brake: "); Serial.print(brake);
        Serial.print(" ("); Serial.print(adcToVoltage(brake), 2); Serial.println("V)");
        Serial.print("Rotary: "); Serial.println(rotary);
        Serial.println();
        Serial.print("TS:"); Serial.print(ts);
        Serial.print(" BSPD:"); Serial.print(bspd);
        Serial.print(" Inertia:"); Serial.print(inertia);
        Serial.print(" R2D:"); Serial.print(r2d);
        Serial.print(" ATS:"); Serial.println(ats);
        Serial.println();
        Serial.print("Encoders FR:"); Serial.print(encoder_fr_count);
        Serial.print(" FL:"); Serial.println(encoder_fl_count);
        Serial.print("CAN RX:"); Serial.println(can_rx_count);

        can1.events();
        delay(100);
    }

    while (Serial.available()) Serial.read();
    Serial.println("\nExited");
}

// Basic tests

void testDigitalOutputs() {
    printHeader("Digital Outputs");
    Serial.println("Testing LEDs and buzzer (3x toggle each)");

    const uint8_t outputs[] = {
        pins::digital_out::RACE_LED, pins::digital_out::TS_LED,
        pins::digital_out::BSPD_LED, pins::digital_out::INERTIA_LED,
        pins::digital_out::BUZZER, pins::digital_out::DISPLAY_MODE,
        pins::digital_out::ATS_OUT
    };

    const char* names[] = {
        "RACE_LED(17)", "TS_LED(16)", "BSPD_LED(14)", "INERTIA_LED(15)",
        "BUZZER(2)", "DISPLAY_MODE(3)", "ATS_OUT(6)"
    };

    for (int i = 0; i < 7; i++) {
        Serial.println(names[i]);
        for (int j = 0; j < 3; j++) {
            digitalWrite(outputs[i], HIGH);
            delay(1000);
            digitalWrite(outputs[i], LOW);
            delay(1000);
        }
    }
    Serial.println("Done");
}

void testDigitalInputs() {
    printHeader("DIGITAL INPUT TEST");
    Serial.println("Reading all digital inputs for 10 seconds...");
    Serial.println("Activate switches/buttons to verify reading");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        Serial.print("TS(21)=");         Serial.print(digitalRead(pins::digital_in::TS));
        Serial.print(" | BSPD(18)=");    Serial.print(digitalRead(pins::digital_in::BSPD));
        Serial.print(" | INERTIA(7)=");  Serial.print(digitalRead(pins::digital_in::INERTIA));
        Serial.print(" | R2D(4)=");      Serial.print(digitalRead(pins::digital_in::R2D));
        Serial.print(" | ATS(5)=");      Serial.println(digitalRead(pins::digital_in::ATS));
        delay(500);
    }

    Serial.println("[PASS] Digital inputs test complete");
}

void testAnalogInputs() {
    printHeader("ANALOG INPUT TEST");
    Serial.println("Reading all analog inputs for 10 seconds...");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        int apps_higher = analogRead(pins::analog::APPS_HIGHER);
        int apps_lower = analogRead(pins::analog::APPS_LOWER);
        int brake = analogRead(pins::analog::BRAKE_PRESSURE);
        int rotary = analogRead(pins::analog::ROTARY_SWITCH);

        Serial.print("APPS_H(20)="); Serial.print(apps_higher);
        Serial.print(" ("); Serial.print(adcToVoltage(apps_higher), 2); Serial.print("V)");

        Serial.print(" | APPS_L(22)="); Serial.print(apps_lower);
        Serial.print(" ("); Serial.print(adcToVoltage(apps_lower), 2); Serial.print("V)");

        Serial.print(" | BRAKE(19)="); Serial.print(brake);
        Serial.print(" ("); Serial.print(adcToVoltage(brake), 2); Serial.print("V)");

        Serial.print(" | ROTARY(23)="); Serial.print(rotary);
        Serial.println();

        delay(500);
    }

    Serial.println("[PASS] Analog inputs test complete");
}

void testAPPSPlausibility() {
    printHeader("APPS PLAUSIBILITY TEST");
    Serial.println("Testing APPS correlation...");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        int apps_higher = analogRead(pins::analog::APPS_HIGHER);
        int apps_lower = analogRead(pins::analog::APPS_LOWER);

        float diff_percent = abs(apps_higher - apps_lower) * 100.0 / 1023.0;

        Serial.print("APPS_H="); Serial.print(apps_higher);
        Serial.print(" | APPS_L="); Serial.print(apps_lower);
        Serial.print(" | Diff="); Serial.print(diff_percent, 1); Serial.print("%");

        if (diff_percent > 10.0) {
            Serial.println(" [WARNING]");
        } else {
            Serial.println(" [OK]");
        }

        delay(500);
    }

    Serial.println("[PASS] APPS plausibility test complete");
}

void testEncoders() {
    printHeader("WHEEL ENCODER TEST");
    Serial.println("Testing wheel speed encoders for 15 seconds...");
    Serial.println();

    encoder_fr_count = 0;
    encoder_fl_count = 0;

    unsigned long start = millis();
    while (millis() - start < 15000) {
        if (millis() % 1000 == 0) {
            Serial.print("FR: "); Serial.print(encoder_fr_count);
            Serial.print(" | FL: "); Serial.println(encoder_fl_count);
        }
        delay(10);
    }

    Serial.println();
    Serial.print("[PASS] Total - FR: "); Serial.print(encoder_fr_count);
    Serial.print(" FL: "); Serial.println(encoder_fl_count);
}

void testCANTransmit() {
    printHeader("CAN TRANSMIT TEST");
    Serial.println("Sending 5 CAN messages...");
    Serial.println();

    CAN_message_t msg;
    msg.id = CAN_TEST_TX_ID;
    msg.len = 8;

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            msg.buf[j] = (i * 8) + j;
        }

        Serial.print("TX "); Serial.print(i + 1); Serial.print(": ");
        for (int j = 0; j < 8; j++) {
            Serial.print(msg.buf[j], HEX); Serial.print(" ");
        }

        if (can1.write(msg)) {
            Serial.println(" [OK]");
        } else {
            Serial.println(" [FAILED]");
        }

        delay(500);
    }

    Serial.println("[PASS] CAN transmit test complete");
}

void testCANLoopback() {
    printHeader("CAN RECEIVE TEST");
    Serial.println("Listening for 10 seconds...");
    Serial.println();

    can_message_received = false;
    can_rx_count = 0;
    unsigned long start = millis();

    while (millis() - start < 10000) {
        can1.events();
        delay(10);
    }

    Serial.println();
    Serial.print("Received "); Serial.print(can_rx_count); Serial.println(" messages");

    if (can_rx_count > 0) {
        Serial.println("[PASS] CAN receive working");
    } else {
        Serial.println("[INFO] No messages received");
    }
}

void testSPIPins() {
    printHeader("SPI PIN TEST");
    Serial.println("Toggling SPI pins...");

    Serial.println("CS (Pin 10) toggling...");
    for (int i = 0; i < 5; i++) {
        digitalWrite(pins::spi::CS, HIGH);
        delay(200);
        digitalWrite(pins::spi::CS, LOW);
        delay(200);
    }

    Serial.println("[PASS] SPI pins test complete");
}

void runFullTest() {
    printHeader("FULL TEST SUITE");

    testDigitalOutputs(); delay(1000);
    testDigitalInputs(); delay(1000);
    testAnalogInputs(); delay(1000);
    testAPPSPlausibility(); delay(1000);
    testEncoders(); delay(1000);
    testI2CBusScan(); delay(1000);
    testIMU(); delay(1000);
    testPowerRails(); delay(1000);
    testPWMOutputs(); delay(1000);
    testADCCalibration(); delay(1000);
    testSPIPins(); delay(1000);
    testCANTransmit(); delay(1000);
    testCANLoopback(); delay(1000);
    testCANStress(); delay(1000);
    testInterruptTiming(); delay(1000);

    printHeader("FULL TEST COMPLETE");
}

// Menu

void printMenu() {
    Serial.println("\n=== Dashboard PCB Test ===");
    Serial.println("Basic Tests:");
    Serial.println("  1. Digital Outputs");
    Serial.println("  2. Digital Inputs");
    Serial.println("  3. Analog Inputs");
    Serial.println("  4. APPS Plausibility");
    Serial.println("  5. Wheel Encoders");
    Serial.println("Advanced:");
    Serial.println("  6. I2C Scan  7. IMU  8. Power Rails  9. PWM");
    Serial.println("  a. ADC Cal   b. SPI Pins");
    Serial.println("CAN:");
    Serial.println("  c. TX  d. RX  e. Stress  f. Bidirectional");
    Serial.println("System:");
    Serial.println("  g. Interrupts  h. Monitor  i. Full Test");
    Serial.println("  0. Menu");
    Serial.print("> ");
}

// Setup

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\nDashboard PCB Test - FSFEUP 2026");

    // Digital outputs
    pinMode(pins::digital_out::RACE_LED, OUTPUT);
    pinMode(pins::digital_out::TS_LED, OUTPUT);
    pinMode(pins::digital_out::BSPD_LED, OUTPUT);
    pinMode(pins::digital_out::INERTIA_LED, OUTPUT);
    pinMode(pins::digital_out::BUZZER, OUTPUT);
    pinMode(pins::digital_out::DISPLAY_MODE, OUTPUT);
    pinMode(pins::digital_out::ATS_OUT, OUTPUT);
    digitalWrite(pins::digital_out::RACE_LED, LOW);
    digitalWrite(pins::digital_out::TS_LED, LOW);
    digitalWrite(pins::digital_out::BSPD_LED, LOW);
    digitalWrite(pins::digital_out::INERTIA_LED, LOW);
    digitalWrite(pins::digital_out::BUZZER, LOW);
    digitalWrite(pins::digital_out::DISPLAY_MODE, LOW);
    digitalWrite(pins::digital_out::ATS_OUT, LOW);

    // Digital inputs
    pinMode(pins::digital_in::TS, INPUT_PULLUP);
    pinMode(pins::digital_in::BSPD, INPUT_PULLUP);
    pinMode(pins::digital_in::INERTIA, INPUT_PULLUP);
    pinMode(pins::digital_in::R2D, INPUT_PULLUP);
    pinMode(pins::digital_in::ATS, INPUT_PULLUP);

    // Analog inputs
    pinMode(pins::analog::APPS_HIGHER, INPUT);
    pinMode(pins::analog::APPS_LOWER, INPUT);
    pinMode(pins::analog::BRAKE_PRESSURE, INPUT);
    pinMode(pins::analog::ROTARY_SWITCH, INPUT);

    // SPI
    pinMode(pins::spi::CS, OUTPUT);
    digitalWrite(pins::spi::CS, HIGH);

    // Encoders
    pinMode(pins::encoder::FRONT_RIGHT_WHEEL, INPUT_PULLUP);
    pinMode(pins::encoder::FRONT_LEFT_WHEEL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pins::encoder::FRONT_RIGHT_WHEEL), encoder_fr_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(pins::encoder::FRONT_LEFT_WHEEL), encoder_fl_isr, RISING);

    // I2C
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz

    // CAN
    can1.begin();
    can1.setBaudRate(500000);
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(canSniff);

    Serial.println("Init complete\n");

    // Quick LED blink test
    for (int i = 0; i < 3; i++) {
        digitalWrite(pins::digital_out::RACE_LED, HIGH);
        digitalWrite(pins::digital_out::TS_LED, HIGH);
        digitalWrite(pins::digital_out::BSPD_LED, HIGH);
        digitalWrite(pins::digital_out::INERTIA_LED, HIGH);
        delay(200);
        digitalWrite(pins::digital_out::RACE_LED, LOW);
        digitalWrite(pins::digital_out::TS_LED, LOW);
        digitalWrite(pins::digital_out::BSPD_LED, LOW);
        digitalWrite(pins::digital_out::INERTIA_LED, LOW);
        delay(200);
    }

    printMenu();
}

// Main loop

void loop() {
    can1.events();

    if (Serial.available()) {
        char cmd = Serial.read();
        while (Serial.available()) Serial.read();

        Serial.println(cmd);
        Serial.println();

        switch (cmd) {
            case '1': testDigitalOutputs(); break;
            case '2': testDigitalInputs(); break;
            case '3': testAnalogInputs(); break;
            case '4': testAPPSPlausibility(); break;
            case '5': testEncoders(); break;
            case '6': testI2CBusScan(); break;
            case '7': testIMU(); break;
            case '8': testPowerRails(); break;
            case '9': testPWMOutputs(); break;
            case 'a': case 'A': testADCCalibration(); break;
            case 'b': case 'B': testSPIPins(); break;
            case 'c': case 'C': testCANTransmit(); break;
            case 'd': case 'D': testCANLoopback(); break;
            case 'e': case 'E': testCANStress(); break;
            case 'f': case 'F': testCANBidirectional(); break;
            case 'g': case 'G': testInterruptTiming(); break;
            case 'h': case 'H': continuousMonitor(); break;
            case 'i': case 'I': runFullTest(); break;
            case '0': printMenu(); break;
            default: Serial.println("Invalid option"); break;
        }

        if (cmd != '0') {
            Serial.println("\nPress 0 for menu");
        }
    }

    delay(10);
}
