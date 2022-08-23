#include <algorithm>

#ifndef UTILITY_H
#define UTILITY_H

using namespace std;

template <typename T>
void ResetArraySize(T** arrStart, int oldSize, int newSize)
{
    if (arrStart == NULL || oldSize == 0)
    {
        *arrStart = (T*) calloc(newSize, sizeof(T));
    }
    else if (oldSize <= newSize) // copy all the old values
    {
        auto temp = (T*) calloc(newSize, sizeof(T));
        copy(*arrStart, *arrStart + oldSize, temp);
        delete *arrStart;
        *arrStart = temp;
    }
    else // copy only up to the new size
    {
        auto temp = (T*) calloc(newSize, sizeof(T));
        copy(*arrStart, *arrStart + newSize, temp);
        delete *arrStart;
        *arrStart = temp;
    }
}

#endif