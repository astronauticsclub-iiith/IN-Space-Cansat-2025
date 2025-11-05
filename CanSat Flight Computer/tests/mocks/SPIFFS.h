#ifndef SPIFFS_H
#define SPIFFS_H

#ifdef TESTING_MODE
    #include "../tests/mocks/arduino_mock.h"
#else
    #include "../tests/mocks/arduino_mock.h"
#endif

// SPIFFS mock class
class SPIFFSMock {
public:
    bool begin(bool formatOnFail = false) { return true; }
    void end() {}
    bool format() { return true; }
    size_t totalBytes() { return 1000000; }
    size_t usedBytes() { return 500000; }
};

extern SPIFFSMock SPIFFS;

#endif // SPIFFS_H