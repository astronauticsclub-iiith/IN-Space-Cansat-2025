#ifndef LORA_H
#define LORA_H

#ifdef TESTING_MODE
    #include "../tests/mocks/arduino_mock.h"
#else
    #include "../tests/mocks/arduino_mock.h"
#endif

// LoRa mock class
class LoRaMock {
public:
    bool begin(long frequency) { return true; }
    void setPins(int ss = -1, int reset = -1, int dio0 = -1) {}
    void setSpreadingFactor(int sf) {}
    void setSignalBandwidth(long sbw) {}
    void setCodingRate4(int denominator) {}
    void setPreambleLength(long length) {}
    void setSyncWord(int sw) {}
    void enableCrc() {}
    void disableCrc() {}
    
    int beginPacket(int implicitHeader = false) { return 1; }
    int endPacket(bool async = false) { return 1; }
    int parsePacket(int size = 0) { return 0; }
    int packetRssi() { return -50; }
    float packetSnr() { return 10.0; }
    long packetFrequencyError() { return 0; }
    
    size_t write(uint8_t byte) { return 1; }
    size_t write(const uint8_t *buffer, size_t size) { return size; }
    int available() { return 0; }
    int read() { return 0; }
    int peek() { return 0; }
    void flush() {}
    
    void onReceive(void(*callback)(int)) {}
    void onTxDone(void(*callback)()) {}
    
    void receive(int size = 0) {}
    void idle() {}
    void sleep() {}
    
    void setTxPower(int level, int outputPin = 1) {}
    void setFrequency(long frequency) {}
    void setOCP(uint8_t mA) {}
    void setGain(uint8_t gain) {}
    
    uint8_t random() { return 42; }
    void enableInvertIQ() {}
    void disableInvertIQ() {}
    
    void onCadDone(void(*callback)(bool)) {}
    int rssi() { return -50; }
    
    void setLdoFlag() {}
    
    size_t print(const String& s) { return s.length(); }
    size_t print(const char* str) { return strlen(str); }
    size_t print(int n) { return 1; }
    size_t print(float f) { return 1; }
    size_t println(const String& s) { return s.length() + 1; }
    size_t println(const char* str) { return strlen(str) + 1; }
    size_t println(int n) { return 2; }
    size_t println(float f) { return 2; }
    size_t println() { return 1; }
};

extern LoRaMock LoRa;

#endif // LORA_H