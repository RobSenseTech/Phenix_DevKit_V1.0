#include <mm.h>

void *kzalloc(size_t size)
{
    void *alloc = pvPortMalloc(size);
    if(alloc)
    {
        memset(alloc, 0, size);
    }

    return alloc;
}


