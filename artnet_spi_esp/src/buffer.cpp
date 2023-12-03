#include <Arduino.h>
#include <buffer.h>

DataBuffer::DataBuffer(uint8_t *buffer, uint16_t totalSize) {
    this->buffer = buffer;
    this->totalSize = totalSize;
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

int DataBuffer::used() {
    return usedCount;
}

int DataBuffer::available() {
    return totalSize - usedCount;
}

void DataBuffer::enqueue(uint8_t data) {
    buffer[enqueuePosition] = data;

    usedCount++;
    enqueuePosition++;

    if (enqueuePosition >= 500) {
        enqueuePosition = 0;
    }
}

void DataBuffer::enqueueIfValid(uint8_t data) {
    if (data > 0) {
        enqueue(data);
    }
}

uint8_t DataBuffer::dequeue() {
    uint8_t result = buffer[dequeuePosition];

    usedCount--;
    dequeuePosition++;

    if (dequeuePosition >= 500) {
        dequeuePosition = 0;
    }

    return result;
}

uint8_t DataBuffer::get() {
    return buffer[dequeuePosition];
}
