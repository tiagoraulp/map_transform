#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include "pointi.hpp"

class Apath{
public:
    std::vector<PointI> points;
    double cost;
    Apath();
};

template<typename T>
class node{
protected:
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    T level;
    // priority=level+remaining distance estimate
    T priority;  // smaller: higher priority

    inline float costEstimateDist(const float d, const int ss) const{
        return d/14.14213562*14;
    }
    inline float costEstimateDist(const float d, const float ss) const{
        return d;
    }
    virtual float costEstimate(const int x,const int y) const;
    const T & staticCast(const float d) const;
public:
    node(int xp, int yp, T d, T p);
    int getxPos() const ;
    int getyPos() const ;
    T getLevel() const ;
    T getPriority() const ;
    bool abovePriorityThreshold(T th) const ;

    virtual void updatePriority(const int & xDest, const int & yDest);
    // give better priority to going strait instead of diagonally
    virtual void nextLevel(const int & i);// i: direction
    // Estimation function for the remaining distance to the goal.
    virtual const T & estimate(const int & xDest, const int & yDest) const;
};

template <typename T>
Apath Astar(PointI p0, PointI p1, std::vector<std::vector<bool> > msg_rcv, bool disable_thin_diagonals=false, T th=T(-1));

#endif // ASTAR_HPP
