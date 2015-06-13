#ifndef CHAIN_HPP
#define CHAIN_HPP

class Chain{
public:
    int x;
    int y;
    int d;
    Chain(int x_,int y_,int d_);
    bool operator==(Chain const & C);
    bool operator!=(Chain const & C);
};


#endif // CHAIN_HPP
