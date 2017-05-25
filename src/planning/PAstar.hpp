#ifndef PASTAR_HPP
#define PASTAR_HPP

#include "Astar.hpp"
#include <map_transform/VisNode.h>
#include <opencv2/core/core.hpp>

template<typename T>
class nodePA: public node<T>{
    int infl, defl;
    T sens;
    float opt;
    map_transform::VisNode crit;
    float K;
    bool bfs;
    bool power;
    int Gx;
    int Gy;
    float (nodePA::*costEst) (const int x, const int y) const;
    float (nodePA::*costSens) (const int x, const int y) const;

    float costSensing(const int x,const int y) const;
    float costSensing2(const int x,const int y) const;
    float costEstimate(const int x,const int y) const;
    float costEstimate2(const int x,const int y) const;
public:
    nodePA(int xp, int yp, T d, T p, int inf, int def, T ss, float dist, bool q, bool bfs_, map_transform::VisNode cr_);
    T getSensing() const;
    void S2P(void);
    void updatePriority(const int & xDest, const int & yDest);
    void updateSensing(const int & xDest, const int & yDest);
    const T & estimate(const int & xDest, const int & yDest) const;
    const T & sensing(const int & xDest, const int & yDest) const;
};

class PAstar{
private:
    std::vector<std::vector<bool> > map_;
    cv::Mat or_map;
    int infl;
    int defl;
    bool isGoal(PointI p0, PointI p1);
public:
    PAstar(int in, int de);
    PAstar();
    void updateNavMap(std::vector<std::vector<bool> > map);
    void updateOrMap(cv::Mat map);
    template <typename T=float>
    Apath run(PointI p0, PointI p1, float opt=-3, bool bfs=false, map_transform::VisNode crit=map_transform::VisNode());
};

#endif // PASTAR_HPP
