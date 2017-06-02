#ifndef POINTI_HPP
#define POINTI_HPP

class PointI{
public:
    int i;
    int j;
    PointI(int a, int b);
    PointI();
    unsigned long int diff2(PointI other);
    bool operator==(const PointI &other) const;
    bool operator!=(const PointI &other) const;
};
#endif // POINTI_HPP
