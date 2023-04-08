#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_
#include <array>
#include <stdint.h>
#include "utils/log.h"

// #define SIZE 512

template<typename T, uint32_t SIZE>
class RingBuffer {
public:
  RingBuffer() {}
  ~RingBuffer() {}
  // Return true on success, false on failure.
  bool push(T _data) {
    if (isFull()) {
      LOGERROR("RingBuffer is full, cannot push data!");
      return false;
    }
    buf[head] = _data;
    head = (head + 1) % SIZE;
    if (head == tail) {
      tail = (tail + 1) % SIZE;
    }
    return true;
  }
  bool pop(T &outData) {
    if (isEmpty()) {
      LOGERROR("RingBuffer is empty, cannot pop data!");
      return false;
    }
    outData = buf[tail];
    tail = (tail + 1) % SIZE;
    return true;
  }
  uint32_t size() {
    if (head >= tail) {
      return head - tail;
    } else {
      return SIZE - tail + head;
    }
  }
  uint32_t capacity() { return SIZE - 1; }
  bool isEmpty() {
    if (head == tail) {
      return true;
    } else {
      return false;
    }
  }
  bool isFull() {
    if ((head + 1) % SIZE == tail) {
      return true;
    } else {
      return false;
    }
  }
  void clear() {
    head = 0;
    tail = 1;
  }

private:
  std::array<T, SIZE> buf{};
  uint32_t head = 0;
  uint32_t tail = 1;
  uint32_t _size = SIZE;
};

#endif // _RINGBUFFER_H_
