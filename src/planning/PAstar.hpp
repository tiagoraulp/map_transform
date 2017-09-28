#ifndef PASTAR_HPP
#define PASTAR_HPP

#include "Astar.hpp"
#include <map_transform/VisNode.h>
#include <opencv2/core/core.hpp>

class PApath: public Apath {
public:
    double costM, costP;
    long int exp_nodes_r, tested_goal, exp_unfiltered;
    PApath();
};

template<typename T>
class nodePA: public node<T>{
    int infl, defl;
    T sens;
    float opt;
    map_transform::VisNode * crit;
    std::vector<float> * dist_c_g, * angle_c_g, * angleD_c_g;
    bool useCritSens, useOptSens;
    float K;
    bool bfs;
    bool power;
    int Gx;
    int Gy;
    float k2;
    float (nodePA::*costEst) (const int x, const int y) const;
    float (nodePA::*costSens) (const int x, const int y) const;
    bool validSensing(const int x, const int y, float dist) const;
    float costSensing(const int x,const int y) const;
    float costSensing2(const int x,const int y) const;
    float costEstimate(const int x,const int y) const;
    float costEstimate2(const int x,const int y) const;
public:
    nodePA(int xp, int yp, T d, T p, int inf, int def, T ss, float cost2, float dist, bool q, bool bfs_, map_transform::VisNode * cr_, std::vector<float> * dist_crit_goal,bool use_opt_sens, bool use_crit_sens, std::vector<float> * angle_crit_goal, std::vector<float> * angleD_crit_goal);
    T getSensing() const;
    double getCost() const;
    double getMotionCost() const;
    double getSensingCost() const;
    void S2P(void);
    void updatePriority(const int & xDest, const int & yDest);
    void updateSensing(const int & xDest, const int & yDest);
    const T & estimate(const int & xDest, const int & yDest) const;
    const T & sensing(const int & xDest, const int & yDest) const;
};

class PAstar{
private:
    std::vector<std::vector<bool> > map_;
    cv::Mat or_map, expansion;
    std::vector<cv::Mat> expansions;
    int infl;
    int defl;
    bool isGoal(PointI p0, PointI p1);
    int save_rate;
public:
    PAstar(int in, int de);
    PAstar();
    void updateNavMap(std::vector<std::vector<bool> > map);
    void updateOrMap(cv::Mat map);
    void resetExpansion(void);
    cv::Mat getExpansion(void);
    std::vector<cv::Mat> getExpansions(void);
    template <typename T=float>
    PApath run(PointI p0, PointI p1, float k2=1, bool quad=false, float opt=-3, bool bfs=false, map_transform::VisNode * crit=NULL, std::vector<float> * dist_crit_goal=NULL, bool use_opt_sens=false, bool use_crit_sens=false, std::vector<float> * angle_crit_goal=NULL, std::vector<float> * angleD_crit_goal=NULL, int rate_save=-1);
};

#endif // PASTAR_HPP
