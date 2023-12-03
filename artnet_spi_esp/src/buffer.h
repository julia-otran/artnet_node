#include <Arduino.h>

class DataBuffer {
    public:
        DataBuffer(uint8_t *buffer, uint16_t size);
        void clear();
        uint8_t isEmpty();
        uint8_t canEnqueue();
        int used();
        int available();
        void enqueue(uint8_t data);
        void enqueueIfValid(uint8_t data);
        uint8_t dequeue();
        uint8_t get();
    private:
        uint16_t dequeuePosition, enqueuePosition, usedCount, totalSize;
        uint8_t *buffer;
};
