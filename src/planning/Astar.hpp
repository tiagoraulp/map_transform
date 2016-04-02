#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>

class PointI{
public:
    int i;
    int j;
    PointI(int a, int b);
    PointI();
};

class Apath{
public:
    std::vector<PointI> points;
    double cost;
    Apath();
};

template <typename T>
Apath Astar(PointI p0, PointI p1, std::vector<std::vector<bool> > msg_rcv);

#endif // ASTAR_HPP
