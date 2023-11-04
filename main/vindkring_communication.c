
#include <vindkring_communication.h>
#include "esp_log.h"


void clearRxBuf(uint8_t *serialRxBuf) {
        // Clear everything for the next message
        memset(serialRxBuf, 0, sizeof(serialRxBuf));
        rxBufIdx = 0;
    }

void parseState(particleSensorState_t& state) {
    /**
     *         MSB  DF 3     DF 4  LSB
     * uint16_t = xxxxxxxx xxxxxxxx
     */
    const uint16_t pm25 = (serialRxBuf[5] << 8) | serialRxBuf[6];

    Serial.printf("Received PM 2.5 reading: %d\n", pm25);

    state.measurements[state.measurementIdx] = pm25;

    state.measurementIdx = (state.measurementIdx + 1) % 5;

    if (state.measurementIdx == 0) {
        float avgPM25 = 0.0f;

        for (uint8_t i = 0; i < 5; ++i) {
            avgPM25 += state.measurements[i] / 5.0f;
        }

        state.avgPM25 = avgPM25;
        state.valid = true;

        Serial.printf("New Avg PM25: %d\n", state.avgPM25);
    }

    clearRxBuf();
}

bool isValidHeader() {
    bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

    if (!headerValid) {
        Serial.println("Received message with invalid header.");
    }

    return headerValid;
}

bool isValidChecksum() {
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < 20; i++) {
        checksum += serialRxBuf[i];
    }

    if (checksum != 0) {
        Serial.printf("Received message with invalid checksum. Expected: 0. Actual: %d\n", checksum);
    }

    return checksum == 0;
}