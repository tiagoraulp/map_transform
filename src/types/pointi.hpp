#ifndef POINTI_HPP
#define POINTI_HPP

class PointI{
public:
    int i;
    int j;
    PointI(int a, int b);
    PointI();
    bool operator==(const PointI &other) const;
    bool operator!=(const PointI &other) const;
};
#endif // POINTI_HPP
