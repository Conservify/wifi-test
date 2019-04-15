#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

#include <Arduino.h>
#include <SerialFlash.h>
#include <SPI.h>

#include <cstdarg>

#include "secrets.h"

void debugf(const char *f, ...);

constexpr size_t DEBUG_LINE_MAX = 256;

void debugf(const char *f, ...) {
    char buffer[DEBUG_LINE_MAX];
    va_list args;

    va_start(args, f);
    vsnprintf(buffer, DEBUG_LINE_MAX, f, args);
    va_end(args);

    if (Serial) {
        Serial.print(buffer);
    }
    Serial5.print(buffer);
}

void setup() {
    Serial5.begin(115200);
    Serial.begin(115200);

    while (!Serial) {
        delay(10);
    }

    #ifdef FK_NATURALIST
    static constexpr uint8_t WIFI_PIN_CS = 7;
    static constexpr uint8_t WIFI_PIN_IRQ = 16;
    static constexpr uint8_t WIFI_PIN_RST = 15;
    static constexpr uint8_t WIFI_PIN_EN = 38;
    Serial.println("FK_NATURALIST");
    #endif

    #ifdef FK_CORE
    static constexpr uint8_t WIFI_PIN_CS = 7;
    static constexpr uint8_t WIFI_PIN_IRQ = 16;
    static constexpr uint8_t WIFI_PIN_RST = 15;
    static constexpr uint8_t WIFI_PIN_EN = 38;
    Serial.println("FK_CORE");
    #endif

    #ifdef ADAFRUIT_FEATHER
    static constexpr uint8_t WIFI_PIN_CS = 8;
    static constexpr uint8_t WIFI_PIN_IRQ = 7;
    static constexpr uint8_t WIFI_PIN_RST = 4;
    static constexpr uint8_t WIFI_PIN_EN = 2;
    Serial.println("ADAFRUIT_FEATHER");
    #endif

    Serial.println("Reset board...");

    SPI.begin();

    static constexpr uint8_t SD_PIN_CS = 12;
    static constexpr uint8_t FLASH_PIN_CS = 26u;
    static constexpr uint8_t PERIPHERALS_ENABLE_PIN = 25u;
    static constexpr uint8_t RFM95_PIN_CS = 5;

    pinMode(PERIPHERALS_ENABLE_PIN, OUTPUT);
    digitalWrite(PERIPHERALS_ENABLE_PIN, LOW);

    pinMode(WIFI_PIN_RST, OUTPUT);
    pinMode(WIFI_PIN_EN, OUTPUT);
    pinMode(FLASH_PIN_CS, OUTPUT);
    pinMode(SD_PIN_CS, OUTPUT);
    pinMode(WIFI_PIN_CS, OUTPUT);
    pinMode(RFM95_PIN_CS, OUTPUT);

    digitalWrite(FLASH_PIN_CS, HIGH);
    digitalWrite(SD_PIN_CS, HIGH);
    digitalWrite(WIFI_PIN_CS, HIGH);
    digitalWrite(RFM95_PIN_CS, HIGH);

    pinMode(WIFI_PIN_EN, LOW);

    digitalWrite(PERIPHERALS_ENABLE_PIN, LOW);
    pinMode(WIFI_PIN_RST, LOW);
    delay(100);
    digitalWrite(PERIPHERALS_ENABLE_PIN, HIGH);
    pinMode(WIFI_PIN_RST, HIGH);
    delay(100);

    SPI.begin();

    if (false)
        while (true) {
            if (!SerialFlash.begin(FLASH_PIN_CS)) {
                Serial.println("Flash FAILED...");
                while (true) {
                    delay(100);
                }
            }
            break;
            delay(1000);
        }

    Serial.println("Configuring...");
    WiFi.setPins(WIFI_PIN_CS, WIFI_PIN_IRQ, WIFI_PIN_RST, WIFI_PIN_EN);

    Serial.println("Check board...");
    while (true) {
        if (WiFi.status() == WL_NO_SHIELD) {
            Serial.println("FAIL");

            delay(1000);

            /*
            pinMode(13, OUTPUT);
            while (true) {
                delay(100);
                digitalWrite(13, HIGH);
                delay(100);
                digitalWrite(13, LOW);
            }
            */
        }
        else {
            break;
        }
    }

    Serial.println("Creating AP...");
    auto status = WiFi.beginAP("JACOB", 5);
    if (status != WL_AP_LISTENING) {
        Serial.println("FAIL");
    }

    // Serial.println("Connecting...");
    // Serial.println(WifiSsid);
    // WiFi.begin(WifiSsid, WifiPassword);
}

const char *getWifiStatus(uint8_t status) {
    switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    case WL_AP_LISTENING: return "WL_AP_LISTENING";
    case WL_AP_CONNECTED: return "WL_AP_CONNECTED";
    case WL_AP_FAILED: return "WL_AP_FAILED";
    case WL_PROVISIONING: return "WL_PROVISIONING";
    case WL_PROVISIONING_FAILED: return "WL_PROVISIONING_FAILED";
    default: return "Unknown";
    }
}

void upload(size_t length, size_t buffer_size) {
    WiFiClient wcl;

    wcl.stop();

    const char *server = "192.168.5.148";

    if (wcl.connect(server, 8080)) {
        debugf("Connecting...\n");
        wcl.println("POST /data.bin HTTP/1.1");
        wcl.print("Host: "); wcl.println(server);
        wcl.println("Content-Type: application/octet");
        wcl.print("Content-Length: "); wcl.println(length);
        wcl.println("User-Agent: ArduinoWiFi/1.1");
        wcl.println("Connection: close");
        wcl.println();

        debugf("Connected...\n");

        auto uploadStarted = millis();
        auto lastStatus = 0;
        auto uploaded = 0;
        auto lastByte = 0;
        auto totalTimeInWrite = 0;
        auto timeInWrite = 0;
        uint8_t data[buffer_size];
        memset(data, 0xdf, sizeof(data));

        while (uploaded < length) {
            auto writeStarted = millis();
            auto wrote = wcl.write(data, sizeof(data));
            auto writeEnded = millis();
            if (wrote < 0) {
                debugf("Failed to write!\n");
                break;
            }
            if (wcl.getWriteError()) {
                debugf("Write Error! (%d)\n", wcl.getWriteError());
                break;
            }

            if (wrote > 0) {
                lastByte = millis();
            }

            totalTimeInWrite += writeEnded - writeStarted;
            timeInWrite += writeEnded - writeStarted;

            if (false && writeEnded - writeStarted > 1000) {
                debugf("Long call to write: %dms\n", writeEnded - writeStarted);
            }

            uploaded += wrote;
            if (millis() - lastStatus > 1000) {
                auto elapsed = (millis() - uploadStarted) / 1000.0f;
                auto complete = (uploaded / (float)length) * 100.0f;
                auto speed = ((uploaded / 1024.f) / (float)elapsed);
                auto rssi = WiFi.RSSI();
                debugf("Upload: %d/%d speed = %fkb/s complete = %f (%d) (tiw = %dms) (ttiw = %dms) (rssi = %d)\n", uploaded, length, speed, complete, buffer_size, timeInWrite, totalTimeInWrite, rssi);
                lastStatus = millis();

                timeInWrite = 0;
            }
        }

        wcl.stop();
    }
    else {
        debugf("Connection failed\n");
    }

    delay(1000);
}

void ping() {
    auto destination = IPAddress(255, 255, 255, 255);

    auto started = millis();

    // Why is this API like this? So weird.
    WiFiUDP udp;
    if (udp.begin(54321)) {
        if (!udp.beginPacket(destination, 54321)) {
            debugf("Failed: beginPacket\n");
        }
        else {
            uint8_t deviceId[8] = { 0xcc };

            udp.write(deviceId, sizeof(deviceId));

            if (!udp.endPacket()) {
                debugf("Failed: endPacket\n");
            }
        }
        udp.stop();
    }

    auto finished = millis();

    debugf("Ping (%dms)\n", finished - millis());
}

void loop() {
    auto statusAt = millis();

    debugf("Ready\n");

    while (true) {
        if (millis() - statusAt > 1000) {
<<<<<<< HEAD
            auto s1 = millis();
            auto status = WiFi.status();
            auto s2 = millis();
            auto ssid = WiFi.SSID();
            auto s3 = millis();
            auto rssi = WiFi.RSSI();
            auto s4 = millis();
            auto version = WiFi.firmwareVersion();
            auto s5 = millis();

            Serial.print(getWifiStatus(status));
            Serial.print(" ");
            Serial.print(rssi);
            Serial.print(" ");
            Serial.print(ssid);
            Serial.print(" ");
            Serial.print(version);
            Serial.print(" ");
            Serial.print(s2 - s1);
            Serial.print(" ");
            Serial.print(s3 - s2);
            Serial.print(" ");
            Serial.print(s4 - s3);
            Serial.print(" ");
            Serial.print(s5 - s4);
            Serial.println("ms");

=======
            debugf(getWifiStatus(WiFi.status()));
            debugf("\n");
>>>>>>> 22b30ff845798799ffb513bd93167b3c512d39a0
            statusAt = millis();

            ping();
        }

        /*
        if (WiFi.status() == WL_CONNECTED) {
            WiFi.noLowPowerMode();

            upload(1024 * 1024, 256);
            upload(1024 * 1024, 512);
            upload(1024 * 1024, 1024);
            upload(1024 * 1024, 1400);
        }
        */
    }
}
