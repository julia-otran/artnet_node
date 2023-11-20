#include <Arduino.h>

class DmxBuffer {
    public:
        DmxBuffer(uint8_t *buffer, uint16_t size);
        void clear();
        uint8_t isEmpty();
        uint8_t canEnqueue();
        int used();
        int available();
        void enqueue(uint8_t data);
        uint8_t dequeue();
    private:
        uint16_t dequeuePosition, enqueuePosition, usedCount, totalSize;
        uint8_t *buffer;
};

