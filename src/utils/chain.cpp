#include "chain.hpp"

Chain::Chain(int x_,int y_,int d_):x(x_),y(y_),d(d_)
{
}

bool Chain::operator==(Chain const & C)
{
    return (this->x == C.x &&
            this->y == C.y &&
            this->d == C.d);
}

bool Chain::operator!=(Chain const & C)
{
    return !(*this == C);
}

