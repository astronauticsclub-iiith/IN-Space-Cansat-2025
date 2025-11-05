#ifndef ARDUINO_H
#define ARDUINO_H

// Arduino.h mock - redirects to our mock system when testing
#ifdef TESTING_MODE
    #include "../tests/mocks/arduino_mock.h"
#else
    // This would normally include the real Arduino.h
    // For testing, we redirect to our mock
    #include "../tests/mocks/arduino_mock.h"
#endif

// Include other Arduino libraries as mocks
#define Preferences_h
#define SPI_h
#define Wire_h
#define SPIFFS_h

#endif // ARDUINO_H