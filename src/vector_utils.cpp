#include "vector_utils.hpp"

int boundPos(int x, int max)
{
    if(x>=max)
        x=max-1;
    if(x<0)
        x=0;
    return x;
}
