#ifndef DATABUFFER_H
#define DATABUFFER_H

#include <Arduino.h>

#define BUFFER_SIZE 20  // Reduced for memory efficiency

/**
 * Circular buffer for storing telemetry strings
 * Memory-optimized with fixed string length
 */
struct DataBuffer {
private:
    static const size_t MAX_ENTRY_SIZE = 400;  // Max telemetry string length
    char data[BUFFER_SIZE][MAX_ENTRY_SIZE];
    volatile int writeIndex;
    volatile int readIndex;
    volatile int count;
    SemaphoreHandle_t mutex;
    
public:
    DataBuffer() : writeIndex(0), readIndex(0), count(0) {
        mutex = xSemaphoreCreateMutex();
    }
    
    ~DataBuffer() {
        if (mutex) vSemaphoreDelete(mutex);
    }
    
    /**
     * Add entry to buffer (thread-safe)
     * @param entry String to add
     * @return true if successful, false if buffer full
     */
    bool add(const String& entry) {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return false;
        }
        
        if (count >= BUFFER_SIZE) {
            xSemaphoreGive(mutex);
            return false;
        }
        
        // Copy string with bounds checking
        strncpy(data[writeIndex], entry.c_str(), MAX_ENTRY_SIZE - 1);
        data[writeIndex][MAX_ENTRY_SIZE - 1] = '\0';
        
        writeIndex = (writeIndex + 1) % BUFFER_SIZE;
        count++;
        
        xSemaphoreGive(mutex);
        return true;
    }
    
    /**
     * Get entry from buffer (thread-safe)
     * @return String entry, or empty if buffer empty
     */
    String get() {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return "";
        }
        
        if (count == 0) {
            xSemaphoreGive(mutex);
            return "";
        }
        
        String entry(data[readIndex]);
        readIndex = (readIndex + 1) % BUFFER_SIZE;
        count--;
        
        xSemaphoreGive(mutex);
        return entry;
    }
    
    /**
     * Check if buffer is empty (thread-safe)
     */
    bool isEmpty() {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return true;
        }
        bool empty = (count == 0);
        xSemaphoreGive(mutex);
        return empty;
    }
    
    /**
     * Check if buffer is full (thread-safe)
     */
    bool isFull() {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return false;
        }
        bool full = (count >= BUFFER_SIZE);
        xSemaphoreGive(mutex);
        return full;
    }
    
    /**
     * Get current count (thread-safe)
     */
    int getCount() {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return 0;
        }
        int c = count;
        xSemaphoreGive(mutex);
        return c;
    }
    
    /**
     * Clear buffer (thread-safe)
     */
    void clear() {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            writeIndex = 0;
            readIndex = 0;
            count = 0;
            xSemaphoreGive(mutex);
        }
    }
};

#endif
