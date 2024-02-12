#include <Arduino.h>

class DataBuffer {
    public:
        DataBuffer(uint8_t *buffer, uint16_t size);
        void clear();
        uint8_t isEmpty();
        uint8_t canEnqueue();
        uint16_t used();
        uint16_t available();
        void enqueue(uint8_t data);
        void enqueueIfValid(uint8_t data);
        uint8_t dequeue();
        uint32_t dequeue32();
        uint8_t get();
    private:
        uint16_t dequeuePosition, enqueuePosition, usedCount, totalSize;
        uint8_t *buffer;
};
