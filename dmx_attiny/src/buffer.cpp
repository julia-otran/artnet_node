#include <Arduino.h>
#include <buffer.h>

DmxBuffer::DmxBuffer(uint8_t *buffer, uint16_t totalSize) {
    this->buffer = buffer;
    this->totalSize = totalSize;
}

void DmxBuffer::clear() {
    dequeuePosition = 0;
    enqueuePosition = 0;
    usedCount = 0;
}

uint8_t DmxBuffer::isEmpty() {
    return usedCount == 0;
}

uint8_t DmxBuffer::canEnqueue() {
    return usedCount < totalSize;
}

int DmxBuffer::used() {
    return usedCount;
}

int DmxBuffer::available() {
    return totalSize - usedCount;
}

void DmxBuffer::enqueue(uint8_t data) {
    buffer[enqueuePosition] = data;

    usedCount++;
    enqueuePosition++;

    if (enqueuePosition >= 500) {
        enqueuePosition = 0;
    }
}

uint8_t DmxBuffer::dequeue() {
    uint8_t result = buffer[dequeuePosition];

    usedCount--;
    dequeuePosition++;

    if (dequeuePosition >= 500) {
        dequeuePosition = 0;
    }

    return result;
}
