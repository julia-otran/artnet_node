#include <Arduino.h>
#include <buffer.h>

DataBuffer::DataBuffer(uint8_t *buffer, uint16_t totalSize) {
    this->buffer = buffer;
    this->totalSize = totalSize;
    this->dequeuePosition = 0;
    this->enqueuePosition = 0;
    this->usedCount = 0;
}

void DataBuffer::clear() {
    dequeuePosition = 0;
    enqueuePosition = 0;
    usedCount = 0;
}

uint8_t DataBuffer::isEmpty() {
    return usedCount == 0;
}

uint8_t DataBuffer::canEnqueue() {
    return usedCount < totalSize;
}

uint16_t DataBuffer::used() {
    return usedCount;
}

uint16_t DataBuffer::available() {
    return totalSize - usedCount;
}

void DataBuffer::enqueue(uint8_t data) {
    if (usedCount >= totalSize) {
        return;
    }

    buffer[enqueuePosition] = data;

    usedCount++;
    enqueuePosition++;

    if (enqueuePosition >= totalSize) {
        enqueuePosition = 0;
    }
}

void DataBuffer::enqueueIfValid(uint8_t data) {
    if (data > 0) {
        enqueue(data);
    }
}

uint8_t DataBuffer::dequeue() {
    if (usedCount <= 0) {
        return 0;
    }

    uint8_t result = buffer[dequeuePosition];

    usedCount--;
    dequeuePosition++;

    if (dequeuePosition >= totalSize) {
        dequeuePosition = 0;
    }

    return result;
}

uint32_t DataBuffer::dequeue32() {
    uint32_t result = 0;

    result |= dequeue();
    result |= ((uint32_t)dequeue()) << 8;
    result |= ((uint32_t)dequeue()) << 16;
    result |= ((uint32_t)dequeue()) << 24;

    return result;
}

uint8_t DataBuffer::get() {
    return buffer[dequeuePosition];
}
