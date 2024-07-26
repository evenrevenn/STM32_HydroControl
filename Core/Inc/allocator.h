#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include "FreeRTOS.h"
#include <cstdint>

template <class T>
class Heap4Alocator
{
public:
    typedef T value_type;

    T* allocate(std::size_t n){ return (T *)pvPortMalloc(n * sizeof(T)); }
    void deallocate(T *p, std::size_t n){ vPortFree((void *)p); }
};

#endif //ALLOCATOR_H