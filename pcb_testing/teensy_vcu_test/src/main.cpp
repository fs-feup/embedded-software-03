// VCU/Master PCB Test Suite
// Tests all I/O, sensors, CAN, I2C, safety systems

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>

// Pin definitions

namespace pins {
    namespace analog {
        constexpr uint8_t BRAKE_SENSOR = 38;
        constexpr uint8_t SOC = 24;
    }

    namespace digital_in {
        constexpr uint8_t ASMS_IN = 18;
        constexpr uint8_t AMI = 19;
        constexpr uint8_t EBS_SENSOR1 = 41;
        constexpr uint8_t EBS_SENSOR2 = 39;
        constexpr uint8_t SDC_TSMS_STATE = 22;
        constexpr uint8_t RL_WSS = 5;
        constexpr uint8_t RR_WSS = 4;
        constexpr uint8_t ATS = 16;
        constexpr uint8_t ASATS = 20;
        constexpr uint8_t WD_READY = 37;
        constexpr uint8_t WD_SDC_RELAY = 33;
    }

    namespace digital_out {
        constexpr uint8_t ASSI_BLUE = 25;
        constexpr uint8_t ASSI_YELLOW = 12;
        constexpr uint8_t EBS_VALVE_REAR = 17;
        constexpr uint8_t EBS_VALVE_FRONT = 13;
        constexpr uint8_t CLOSE_SDC = 21;
        constexpr uint8_t SDC_BSPD_OUT = 14;
        constexpr uint8_t BRAKE_LIGHT = 2;
        constexpr uint8_t WD_SDC_CLOSE = 40;
        constexpr uint8_t WD_ALIVE = 15;
    }
}

// Constants

constexpr int ADC_MAX_VALUE = 1023;
constexpr float MAX_V_ANALOG = 3.3;
constexpr float MIN_HYDRAULIC_V = 0.5;
constexpr float HYDRAULIC_PRESSURE_SLOPE = 65.0f;
constexpr int HYDRAULIC_PRESSURE_MAX_BAR = 95;

// CAN setup

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

constexpr uint32_t CAN_VCU_TX_ID = 0x200;
constexpr uint32_t CAN_VCU_RX_ID = 0x201;
constexpr uint32_t CAN_DASH_RX_ID = 0x100;

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

// Wheel speed counters

volatile uint32_t wss_rl_count = 0;
volatile uint32_t wss_rr_count = 0;

void wss_rl_isr() { wss_rl_count++; }
void wss_rr_isr() { wss_rr_count++; }

// Utility functions

void printHeader(const char* title) {
    Serial.println();
    Serial.print("--- ");
    Serial.print(title);
    Serial.println(" ---");
}

float adcToVoltage(int adc_value) {
    return (adc_value / static_cast<float>(ADC_MAX_VALUE)) * MAX_V_ANALOG;
}

float voltageToPressure(float voltage) {
    if (voltage < MIN_HYDRAULIC_V) return 0.0f;
    return (voltage - MIN_HYDRAULIC_V) * HYDRAULIC_PRESSURE_SLOPE;
}

// I2C tests

void testI2CBusScan() {
    printHeader("I2C BUS SCAN");
    Serial.println("Scanning I2C bus (0x00 - 0x7F)...");
    Serial.println();

    int devicesFound = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("  [FOUND] Device at address 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);

            // Identify common devices
            if (addr >= 0x48 && addr <= 0x4F) Serial.print(" (Current sensor?)");
            else if (addr == 0x76 || addr == 0x77) Serial.print(" (Temp sensor?)");
            else if (addr >= 0x18 && addr <= 0x1F) Serial.print(" (Temp sensor?)");

            Serial.println();
            devicesFound++;
        }
    }

    Serial.println();
    if (devicesFound == 0) {
        Serial.println("[INFO] No I2C devices found");
    } else {
        Serial.print("[PASS] Found "); Serial.print(devicesFound); Serial.println(" device(s)");
    }
}

void testCurrentSensor() {
    printHeader("CURRENT SENSOR TEST (I2C)");
    Serial.println("Testing BSPD current sensor...");
    Serial.println("Searching for INA219/INA226 (0x40-0x4F)");
    Serial.println();

    bool found = false;
    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found current sensor at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found = true;

            // Try to read config register
            Wire.beginTransmission(addr);
            Wire.write(0x00);  // Config register
            Wire.endTransmission(false);
            Wire.requestFrom(addr, 2);

            if (Wire.available() == 2) {
                uint16_t config = (Wire.read() << 8) | Wire.read();
                Serial.print("  Config: 0x"); Serial.println(config, HEX);
                Serial.println("[PASS] Current sensor communication OK");
            }
            break;
        }
    }

    if (!found) {
        Serial.println("[INFO] No current sensor found on I2C bus");
        Serial.println("      May be using analog shunt instead");
    }
}

void testTemperatureSensors() {
    printHeader("TEMPERATURE SENSOR TEST");
    Serial.println("Scanning for brake/motor temperature sensors...");
    Serial.println();

    // Common temp sensor addresses
    uint8_t temp_addrs[] = {0x18, 0x48, 0x49, 0x4A, 0x4B, 0x76, 0x77};
    bool found = false;

    for (int i = 0; i < 7; i++) {
        Wire.beginTransmission(temp_addrs[i]);
        if (Wire.endTransmission() == 0) {
            Serial.print("  [FOUND] Temp sensor at 0x");
            if (temp_addrs[i] < 16) Serial.print("0");
            Serial.println(temp_addrs[i], HEX);
            found = true;

            // Try to read temperature (generic approach)
            Wire.requestFrom(temp_addrs[i], 2);
            if (Wire.available() == 2) {
                int16_t raw = (Wire.read() << 8) | Wire.read();
                float temp = raw / 256.0;  // Common format
                Serial.print("    Temperature: "); Serial.print(temp, 2); Serial.println(" °C");
            }
        }
    }

    if (found) {
        Serial.println("[PASS] Temperature sensor(s) detected");
    } else {
        Serial.println("[INFO] No temperature sensors found on I2C");
    }
}

// Power rail tests

void testPowerRails() {
    printHeader("POWER RAIL VOLTAGE TEST");
    Serial.println("Measuring power supply voltages...");
    Serial.println("NOTE: Requires voltage dividers on test points");
    Serial.println();

    const int samples = 10;

    // Test 5V rail (with divider)
    Serial.println("5V Rail Test:");
    Serial.println("  Connect 5V -> [10kΩ] -> A15 -> [10kΩ] -> GND");
    float rail_sum = 0;
    for (int i = 0; i < samples; i++) {
        int adc = analogRead(A15);
        rail_sum += adcToVoltage(adc);
        delay(10);
    }
    float rail_avg = rail_sum / samples;
    float rail_5v = rail_avg * 2.0;

    Serial.print("  Measured: "); Serial.print(rail_avg, 3); Serial.print("V (ADC) -> ");
    Serial.print(rail_5v, 2); Serial.println("V (5V rail estimate)");

    if (rail_5v > 4.75 && rail_5v < 5.25) {
        Serial.println("  [PASS] 5V rail OK");
    } else {
        Serial.println("  [WARNING] 5V rail out of range");
    }

    Serial.println();
    Serial.println("12V/24V Rails:");
    Serial.println("  Requires external voltage dividers");
    Serial.println("  12V: Use 22kΩ + 10kΩ divider (3.75V at ADC)");
    Serial.println("  24V: Use 47kΩ + 10kΩ divider (4.21V at ADC - exceeds 3.3V!)");
    Serial.println("  24V: Use 68kΩ + 10kΩ divider (3.08V at ADC) ✓");
}

// PWM tests

void testPWMOutputs() {
    printHeader("PWM OUTPUT TEST");
    Serial.println("Testing PWM generation...");
    Serial.println();

    // Test fan/pump PWM control
    Serial.println("Testing PWM on various output pins...");
    Serial.println("(Simulating fan/pump speed control)");

    const uint8_t pwm_pins[] = {
        pins::digital_out::WD_ALIVE,  // Can be used for PWM
        pins::digital_out::BRAKE_LIGHT
    };

    const char* names[] = {"WD_ALIVE (Pin 15)", "BRAKE_LIGHT (Pin 2)"};

    for (int i = 0; i < 2; i++) {
        Serial.print("\nTesting "); Serial.println(names[i]);
        for (int duty = 0; duty <= 255; duty += 64) {
            analogWrite(pwm_pins[i], duty);
            Serial.print("  Duty: "); Serial.print(duty * 100 / 255); Serial.println("%");
            delay(300);
        }
        analogWrite(pwm_pins[i], 0);
    }

    Serial.println("\n[PASS] PWM test complete");
}

// ADC calibration

void testADCCalibration() {
    printHeader("ADC CALIBRATION TEST");
    Serial.println("Testing brake pressure sensor ADC accuracy...");
    Serial.println();

    const int samples = 100;
    int readings[samples];

    Serial.println("Collecting 100 samples from brake sensor...");
    for (int i = 0; i < samples; i++) {
        readings[i] = analogRead(pins::analog::BRAKE_SENSOR);
        delayMicroseconds(100);
    }

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
    float avg_pressure = voltageToPressure(avg_voltage);

    Serial.print("  Average ADC: "); Serial.print(avg, 2);
    Serial.print(" ("); Serial.print(avg_voltage, 3); Serial.print("V, ");
    Serial.print(avg_pressure, 1); Serial.println(" bar)");

    Serial.print("  Min: "); Serial.print(min_val);
    Serial.print("  Max: "); Serial.print(max_val);
    Serial.print("  Range: "); Serial.print(range);
    Serial.print("  ("); Serial.print(range * 100.0 / avg, 2); Serial.println("%)");

    if (range < 10) {
        Serial.println("[PASS] Low noise ADC");
    } else if (range < 30) {
        Serial.println("[WARNING] Moderate ADC noise");
    } else {
        Serial.println("[FAIL] High ADC noise - check wiring");
    }
}

// Advanced CAN tests

void testCANStress() {
    printHeader("CAN STRESS TEST");
    Serial.println("Sending 100 messages at max rate...");
    Serial.println();

    CAN_message_t msg;
    msg.id = CAN_VCU_TX_ID;
    msg.len = 8;

    uint32_t start_time = millis();
    uint32_t sent_count = 0;
    uint32_t failed_count = 0;

    for (int i = 0; i < 100; i++) {
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

        delayMicroseconds(100);
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
        Serial.println("[WARNING] Some messages failed");
    }
}

void testCANBidirectional() {
    printHeader("CAN BIDIRECTIONAL FLOW TEST");
    Serial.println("Testing simultaneous TX and RX...");
    Serial.println("Requires Dashboard or another CAN node");
    Serial.println();

    CAN_message_t msg;
    msg.id = CAN_VCU_TX_ID;
    msg.len = 8;

    can_rx_count = 0;
    uint32_t tx_count = 0;

    unsigned long start = millis();

    Serial.println("Running for 10 seconds...");

    while (millis() - start < 10000) {
        for (int i = 0; i < 8; i++) {
            msg.buf[i] = random(256);
        }

        if (can1.write(msg)) {
            tx_count++;
        }

        can1.events();
        delay(100);
    }

    Serial.println();
    Serial.print("TX: "); Serial.print(tx_count); Serial.println(" messages");
    Serial.print("RX: "); Serial.print(can_rx_count); Serial.println(" messages");

    if (can_rx_count > 0) {
        Serial.println("[PASS] Bidirectional communication working");
    } else {
        Serial.println("[INFO] No RX (external node may not be responding)");
    }
}

// Interrupt timing test

void testInterruptTiming() {
    printHeader("INTERRUPT TIMING TEST");
    Serial.println("Testing wheel speed sensor interrupts...");
    Serial.println();

    wss_rl_count = 0;
    wss_rr_count = 0;

    Serial.println("Monitoring for 5 seconds...");
    unsigned long start = millis();
    unsigned long last_rl = 0, last_rr = 0;

    while (millis() - start < 5000) {
        if (wss_rl_count != last_rl) {
            Serial.print("[RL] Count: "); Serial.println(wss_rl_count);
            last_rl = wss_rl_count;
        }
        if (wss_rr_count != last_rr) {
            Serial.print("[RR] Count: "); Serial.println(wss_rr_count);
            last_rr = wss_rr_count;
        }
        delay(10);
    }

    Serial.println();
    Serial.print("Total RL: "); Serial.print(wss_rl_count);
    Serial.print(" | RR: "); Serial.println(wss_rr_count);

    if (wss_rl_count > 0 || wss_rr_count > 0) {
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
        int brake_adc = analogRead(pins::analog::BRAKE_SENSOR);
        int soc_adc = analogRead(pins::analog::SOC);
        float brake_v = adcToVoltage(brake_adc);
        float brake_p = voltageToPressure(brake_v);

        bool asms = digitalRead(pins::digital_in::ASMS_IN);
        bool ami = digitalRead(pins::digital_in::AMI);
        bool ebs1 = digitalRead(pins::digital_in::EBS_SENSOR1);
        bool ebs2 = digitalRead(pins::digital_in::EBS_SENSOR2);
        bool sdc = digitalRead(pins::digital_in::SDC_TSMS_STATE);
        bool wd_ready = digitalRead(pins::digital_in::WD_READY);

        Serial.print("\033[2J\033[H");  // Clear screen
        Serial.println("=== MONITOR ===");
        Serial.print("Brake: "); Serial.print(brake_adc);
        Serial.print(" ("); Serial.print(brake_v, 2); Serial.print("V, ");
        Serial.print(brake_p, 1); Serial.println("bar)");
        Serial.print("SOC: "); Serial.print(soc_adc);
        Serial.print(" ("); Serial.print(adcToVoltage(soc_adc), 2); Serial.println("V)");
        Serial.println();
        Serial.print("ASMS:"); Serial.print(asms);
        Serial.print(" AMI:"); Serial.print(ami);
        Serial.print(" SDC:"); Serial.print(sdc);
        Serial.print(" EBS1:"); Serial.print(ebs1);
        Serial.print(" EBS2:"); Serial.print(ebs2);
        Serial.print(" WD:"); Serial.println(wd_ready);
        Serial.println();
        Serial.print("WSS RL:"); Serial.print(wss_rl_count);
        Serial.print(" RR:"); Serial.println(wss_rr_count);
        Serial.print("CAN RX:"); Serial.println(can_rx_count);

        can1.events();
        delay(100);
    }

    while (Serial.available()) Serial.read();
    Serial.println("\nExited");
}

// Basic tests

void testDigitalOutputs() {
    printHeader("DIGITAL OUTPUT TEST");
    Serial.println("Testing all outputs...");
    Serial.println("Each output toggles 3 times (1s on, 1s off)");
    Serial.println();

    const uint8_t outputs[] = {
        pins::digital_out::ASSI_BLUE, pins::digital_out::ASSI_YELLOW,
        pins::digital_out::EBS_VALVE_REAR, pins::digital_out::EBS_VALVE_FRONT,
        pins::digital_out::CLOSE_SDC, pins::digital_out::SDC_BSPD_OUT,
        pins::digital_out::BRAKE_LIGHT, pins::digital_out::WD_SDC_CLOSE,
        pins::digital_out::WD_ALIVE
    };

    const char* names[] = {
        "ASSI_BLUE (25)", "ASSI_YELLOW (12)", "EBS_REAR (17)", "EBS_FRONT (13)",
        "CLOSE_SDC (21)", "BSPD_OUT (14)", "BRAKE_LIGHT (2)",
        "WD_SDC_CLOSE (40)", "WD_ALIVE (15)"
    };

    for (int i = 0; i < 9; i++) {
        Serial.print("Testing: "); Serial.println(names[i]);
        for (int j = 0; j < 3; j++) {
            digitalWrite(outputs[i], HIGH);
            Serial.println("  HIGH");
            delay(1000);
            digitalWrite(outputs[i], LOW);
            Serial.println("  LOW");
            delay(1000);
        }
    }

    Serial.println("[PASS] Digital outputs test complete");
}

void testDigitalInputs() {
    printHeader("DIGITAL INPUT TEST");
    Serial.println("Reading all inputs for 10 seconds...");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        Serial.print("ASMS(18)="); Serial.print(digitalRead(pins::digital_in::ASMS_IN));
        Serial.print(" AMI(19)="); Serial.print(digitalRead(pins::digital_in::AMI));
        Serial.print(" EBS1(41)="); Serial.print(digitalRead(pins::digital_in::EBS_SENSOR1));
        Serial.print(" EBS2(39)="); Serial.print(digitalRead(pins::digital_in::EBS_SENSOR2));
        Serial.print(" SDC(22)="); Serial.println(digitalRead(pins::digital_in::SDC_TSMS_STATE));

        Serial.print("ATS(16)="); Serial.print(digitalRead(pins::digital_in::ATS));
        Serial.print(" ASATS(20)="); Serial.print(digitalRead(pins::digital_in::ASATS));
        Serial.print(" WD_RDY(37)="); Serial.print(digitalRead(pins::digital_in::WD_READY));
        Serial.print(" WD_SDC(33)="); Serial.println(digitalRead(pins::digital_in::WD_SDC_RELAY));
        Serial.println();

        delay(1000);
    }

    Serial.println("[PASS] Digital inputs test complete");
}

void testAnalogInputs() {
    printHeader("ANALOG INPUT TEST");
    Serial.println("Reading analog inputs for 10 seconds...");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        int brake_adc = analogRead(pins::analog::BRAKE_SENSOR);
        int soc_adc = analogRead(pins::analog::SOC);

        float brake_v = adcToVoltage(brake_adc);
        float brake_p = voltageToPressure(brake_v);
        float soc_v = adcToVoltage(soc_adc);

        Serial.print("BRAKE(38): "); Serial.print(brake_adc);
        Serial.print(" ("); Serial.print(brake_v, 2); Serial.print("V, ");
        Serial.print(brake_p, 1); Serial.print(" bar)");

        Serial.print("  |  SOC(24): "); Serial.print(soc_adc);
        Serial.print(" ("); Serial.print(soc_v, 2); Serial.println("V)");

        delay(500);
    }

    Serial.println("[PASS] Analog inputs test complete");
}

void testHydraulicPressure() {
    printHeader("HYDRAULIC PRESSURE CALIBRATION");
    Serial.println("Testing brake pressure sensor...");
    Serial.println("Expected: 0.5V (0 bar) to 1.96V (95 bar)");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        int adc = analogRead(pins::analog::BRAKE_SENSOR);
        float voltage = adcToVoltage(adc);
        float pressure = voltageToPressure(voltage);

        Serial.print("ADC="); Serial.print(adc);
        Serial.print(" | V="); Serial.print(voltage, 3);
        Serial.print(" | P="); Serial.print(pressure, 1); Serial.print(" bar");

        if (voltage < MIN_HYDRAULIC_V) {
            Serial.println(" [WARNING: Below min]");
        } else if (pressure > HYDRAULIC_PRESSURE_MAX_BAR) {
            Serial.println(" [WARNING: Exceeds max]");
        } else {
            Serial.println(" [OK]");
        }

        delay(500);
    }

    Serial.println("[PASS] Hydraulic pressure test complete");
}

void testWheelSpeedSensors() {
    printHeader("WHEEL SPEED SENSOR TEST");
    Serial.println("Testing rear wheel speed sensors...");
    Serial.println();

    wss_rl_count = 0;
    wss_rr_count = 0;

    unsigned long start = millis();
    uint32_t last_rl = 0, last_rr = 0;

    while (millis() - start < 15000) {
        if (millis() % 1000 == 0) {
            uint32_t rl_delta = wss_rl_count - last_rl;
            uint32_t rr_delta = wss_rr_count - last_rr;

            Serial.print("RL: "); Serial.print(wss_rl_count);
            Serial.print(" (+"); Serial.print(rl_delta); Serial.print("/s)");
            Serial.print("  |  RR: "); Serial.print(wss_rr_count);
            Serial.print(" (+"); Serial.print(rr_delta); Serial.println("/s)");

            last_rl = wss_rl_count;
            last_rr = wss_rr_count;
        }
        delay(10);
    }

    Serial.println();
    Serial.print("[PASS] Total - RL: "); Serial.print(wss_rl_count);
    Serial.print(" | RR: "); Serial.println(wss_rr_count);
}

void testEBSSensors() {
    printHeader("EBS SENSOR TEST");
    Serial.println("Testing EBS pressure threshold sensors...");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        bool ebs1 = digitalRead(pins::digital_in::EBS_SENSOR1);
        bool ebs2 = digitalRead(pins::digital_in::EBS_SENSOR2);

        Serial.print("EBS1 (41): "); Serial.print(ebs1 ? "PRESSURE OK" : "LOW/FAULT");
        Serial.print("  |  EBS2 (39): "); Serial.print(ebs2 ? "PRESSURE OK" : "LOW/FAULT");

        if (ebs1 && ebs2) {
            Serial.println("  [OK]");
        } else {
            Serial.println("  [WARNING]");
        }

        delay(1000);
    }

    Serial.println("[PASS] EBS sensor test complete");
}

void testSDCLogic() {
    printHeader("SDC LOGIC TEST");
    Serial.println("Monitoring shutdown circuit...");
    Serial.println();

    unsigned long start = millis();
    while (millis() - start < 10000) {
        bool sdc = digitalRead(pins::digital_in::SDC_TSMS_STATE);
        bool wd_sdc = digitalRead(pins::digital_in::WD_SDC_RELAY);
        bool wd_ready = digitalRead(pins::digital_in::WD_READY);

        Serial.print("SDC/TSMS (22): "); Serial.println(sdc ? "CLOSED" : "OPEN");
        Serial.print("WD_SDC_RELAY (33): "); Serial.println(wd_sdc ? "HIGH" : "LOW");
        Serial.print("WD_READY (37): "); Serial.println(wd_ready ? "READY" : "NOT READY");
        Serial.println();

        delay(1000);
    }

    Serial.println("[PASS] SDC logic test complete");
}

void testWatchdogPulse() {
    printHeader("WATCHDOG PULSE TEST");
    Serial.println("Generating 10 pulses on WD_ALIVE...");
    Serial.println();

    for (int i = 0; i < 10; i++) {
        digitalWrite(pins::digital_out::WD_ALIVE, HIGH);
        Serial.print("Pulse "); Serial.print(i + 1); Serial.println(": HIGH");
        delay(50);
        digitalWrite(pins::digital_out::WD_ALIVE, LOW);
        Serial.print("Pulse "); Serial.print(i + 1); Serial.println(": LOW");
        delay(50);
    }

    Serial.println("[PASS] Watchdog pulse test complete");
}

void testCANTransmit() {
    printHeader("CAN TRANSMIT TEST");
    Serial.println("Sending 5 messages...");
    Serial.println();

    CAN_message_t msg;
    msg.id = CAN_VCU_TX_ID;
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

void testEBSActuation() {
    printHeader("EBS Valve Actuation");
    Serial.println("WARNING: Ensure system is safe!");
    Serial.print("Press 'y' to continue: ");

    while (!Serial.available());
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    Serial.println(c);

    if (c != 'y' && c != 'Y') {
        Serial.println("Skipped");
        return;
    }

    Serial.println("\nFRONT valve (13)...");
    digitalWrite(pins::digital_out::EBS_VALVE_FRONT, HIGH);
    delay(2000);
    digitalWrite(pins::digital_out::EBS_VALVE_FRONT, LOW);
    Serial.println("Released");
    delay(1000);

    Serial.println("REAR valve (17)...");
    digitalWrite(pins::digital_out::EBS_VALVE_REAR, HIGH);
    delay(2000);
    digitalWrite(pins::digital_out::EBS_VALVE_REAR, LOW);
    Serial.println("Released");
    delay(1000);

    Serial.println("BOTH valves...");
    digitalWrite(pins::digital_out::EBS_VALVE_FRONT, HIGH);
    digitalWrite(pins::digital_out::EBS_VALVE_REAR, HIGH);
    delay(2000);
    digitalWrite(pins::digital_out::EBS_VALVE_FRONT, LOW);
    digitalWrite(pins::digital_out::EBS_VALVE_REAR, LOW);
    Serial.println("Released");
    Serial.println("Done");
}

void runFullTest() {
    printHeader("FULL TEST SUITE");

    testDigitalOutputs(); delay(1000);
    testDigitalInputs(); delay(1000);
    testAnalogInputs(); delay(1000);
    testHydraulicPressure(); delay(1000);
    testWheelSpeedSensors(); delay(1000);
    testEBSSensors(); delay(1000);
    testSDCLogic(); delay(1000);
    testWatchdogPulse(); delay(1000);
    testI2CBusScan(); delay(1000);
    testCurrentSensor(); delay(1000);
    testTemperatureSensors(); delay(1000);
    testPowerRails(); delay(1000);
    testPWMOutputs(); delay(1000);
    testADCCalibration(); delay(1000);
    testCANTransmit(); delay(1000);
    testCANLoopback(); delay(1000);
    testCANStress(); delay(1000);
    testInterruptTiming(); delay(1000);

    Serial.println();
    Serial.println("[INFO] EBS actuation skipped in full test (run manually)");

    printHeader("FULL TEST COMPLETE");
}

// Menu

void printMenu() {
    Serial.println("\n=== VCU PCB Test ===");
    Serial.println("Basic:");
    Serial.println("  1. Dig Out  2. Dig In  3. Analog  4. Pressure  5. WSS");
    Serial.println("  6. EBS      7. SDC     8. Watchdog");
    Serial.println("Advanced:");
    Serial.println("  9. I2C Scan  a. Current  b. Temp  c. Power  d. PWM  e. ADC Cal");
    Serial.println("CAN:");
    Serial.println("  f. TX  g. RX  h. Stress  i. Bidirectional");
    Serial.println("System:");
    Serial.println("  j. Interrupts  k. Monitor  l. EBS Actuation  m. Full Test");
    Serial.println("  0. Menu");
    Serial.print("> ");
}

// Setup

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\nVCU PCB Test - FSFEUP 2026");

    // Digital outputs
    pinMode(pins::digital_out::ASSI_BLUE, OUTPUT);
    pinMode(pins::digital_out::ASSI_YELLOW, OUTPUT);
    pinMode(pins::digital_out::EBS_VALVE_REAR, OUTPUT);
    pinMode(pins::digital_out::EBS_VALVE_FRONT, OUTPUT);
    pinMode(pins::digital_out::CLOSE_SDC, OUTPUT);
    pinMode(pins::digital_out::SDC_BSPD_OUT, OUTPUT);
    pinMode(pins::digital_out::BRAKE_LIGHT, OUTPUT);
    pinMode(pins::digital_out::WD_SDC_CLOSE, OUTPUT);
    pinMode(pins::digital_out::WD_ALIVE, OUTPUT);

    // All LOW (safe state)
    digitalWrite(pins::digital_out::ASSI_BLUE, LOW);
    digitalWrite(pins::digital_out::ASSI_YELLOW, LOW);
    digitalWrite(pins::digital_out::EBS_VALVE_REAR, LOW);
    digitalWrite(pins::digital_out::EBS_VALVE_FRONT, LOW);
    digitalWrite(pins::digital_out::CLOSE_SDC, LOW);
    digitalWrite(pins::digital_out::SDC_BSPD_OUT, LOW);
    digitalWrite(pins::digital_out::BRAKE_LIGHT, LOW);
    digitalWrite(pins::digital_out::WD_SDC_CLOSE, LOW);
    digitalWrite(pins::digital_out::WD_ALIVE, LOW);

    // Digital inputs
    pinMode(pins::digital_in::ASMS_IN, INPUT_PULLUP);
    pinMode(pins::digital_in::AMI, INPUT_PULLUP);
    pinMode(pins::digital_in::EBS_SENSOR1, INPUT_PULLUP);
    pinMode(pins::digital_in::EBS_SENSOR2, INPUT_PULLUP);
    pinMode(pins::digital_in::SDC_TSMS_STATE, INPUT_PULLUP);
    pinMode(pins::digital_in::RL_WSS, INPUT_PULLUP);
    pinMode(pins::digital_in::RR_WSS, INPUT_PULLUP);
    pinMode(pins::digital_in::ATS, INPUT_PULLUP);
    pinMode(pins::digital_in::ASATS, INPUT_PULLUP);
    pinMode(pins::digital_in::WD_READY, INPUT_PULLUP);
    pinMode(pins::digital_in::WD_SDC_RELAY, INPUT_PULLUP);

    // Analog inputs
    pinMode(pins::analog::BRAKE_SENSOR, INPUT);
    pinMode(pins::analog::SOC, INPUT);

    // Wheel speed sensor interrupts
    attachInterrupt(digitalPinToInterrupt(pins::digital_in::RL_WSS), wss_rl_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(pins::digital_in::RR_WSS), wss_rr_isr, RISING);

    // I2C
    Wire.begin();
    Wire.setClock(400000);

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
        digitalWrite(pins::digital_out::ASSI_BLUE, HIGH);
        digitalWrite(pins::digital_out::ASSI_YELLOW, HIGH);
        digitalWrite(pins::digital_out::BRAKE_LIGHT, HIGH);
        delay(200);
        digitalWrite(pins::digital_out::ASSI_BLUE, LOW);
        digitalWrite(pins::digital_out::ASSI_YELLOW, LOW);
        digitalWrite(pins::digital_out::BRAKE_LIGHT, LOW);
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
            case '4': testHydraulicPressure(); break;
            case '5': testWheelSpeedSensors(); break;
            case '6': testEBSSensors(); break;
            case '7': testSDCLogic(); break;
            case '8': testWatchdogPulse(); break;
            case '9': testI2CBusScan(); break;
            case 'a': case 'A': testCurrentSensor(); break;
            case 'b': case 'B': testTemperatureSensors(); break;
            case 'c': case 'C': testPowerRails(); break;
            case 'd': case 'D': testPWMOutputs(); break;
            case 'e': case 'E': testADCCalibration(); break;
            case 'f': case 'F': testCANTransmit(); break;
            case 'g': case 'G': testCANLoopback(); break;
            case 'h': case 'H': testCANStress(); break;
            case 'i': case 'I': testCANBidirectional(); break;
            case 'j': case 'J': testInterruptTiming(); break;
            case 'k': case 'K': continuousMonitor(); break;
            case 'l': case 'L': testEBSActuation(); break;
            case 'm': case 'M': runFullTest(); break;
            case '0': printMenu(); break;
            default: Serial.println("Invalid option"); break;
        }

        if (cmd != '0') {
            Serial.println("\nPress 0 for menu");
        }
    }

    delay(10);
}
