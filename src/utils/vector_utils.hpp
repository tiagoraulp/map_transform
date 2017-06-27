#ifndef VECTOR_UTILS_HPP
#define VECTOR_UTILS_HPP

#include <vector>

#include <opencv2/core/core.hpp>

std::vector<cv::Point> bf_hlx(int defl);

template <typename T, typename T2=T>
class FindElem
{
protected:
    int n;
    int ind;
    T fv;
    T2 p;
    T fv2;
    virtual bool func(T var, T local)=0;
public:
    //void iter(T var);
    void iter(T var,T2 pt, T var2);
    FindElem();
    void clear(void);
    int getInd(void);
    T getVal(void);
    T2 getP(void);
    bool valid();
};

template <typename T, typename T2=T>
class FindMax : public FindElem<T,T2>
{
protected:
    bool func(T var, T local);
public:
    FindMax(std::vector<T> vars);
    FindMax(std::vector<T> vars, std::vector<T2> pts);
    FindMax(void);
};

template <typename T,typename T2=T>
class FindMin : public FindElem<T,T2>
{
protected:
    bool func(T var, T local);
public:
    FindMin(std::vector<T> vars);
    FindMin(std::vector<T> vars, std::vector<T2> pts);
    FindMin(void);
};

template <typename T, typename T2=T>
class OrderedVector
{
protected:
    std::vector<T> fv;
    std::vector<T2> p;
    virtual bool func(T var, T comp)=0;
public:
    unsigned int iter(T var,T2 pt);
    OrderedVector();
    void clear(void);
    unsigned getSize(void);
    T getVal(unsigned int ind);
    T2 getP(unsigned int ind);
};

template <typename T, typename T2=T>
class OrderedDecresc : public OrderedVector<T,T2>
{
protected:
    bool func(T var, T comp);
public:
    OrderedDecresc(void);
};

template <typename T, typename T2=T>
class OrderedCresc : public OrderedVector<T,T2>
{
protected:
    bool func(T var, T comp);
public:
    OrderedCresc(void);
};


//template <typename T, typename T2>
//void FindElem<T,T2>::iter(T var) {
//    if(n==0)
//    {
//        ind=0;
//        fv=var;
//    }
//    else if(func(var))
//    {
//        ind=n;
//        fv=var;
//    }
//    n++;
//}

template <typename T, typename T2>
void FindElem<T,T2>::iter(T var,T2 pt=T2(), T var2=T()) {
    if(n==0)
    {
        ind=0;
        fv=var;
        p=pt;
        fv2=var2;
    }
    else if(func(var, fv))
    {
        ind=n;
        fv=var;
        p=pt;
        fv2=var2;
    }
    else if(!func(-var, -fv)){
        if(func(var2, fv2)){
            ind=n;
            fv=var;
            p=pt;
            fv2=var2;
        }
    }
    n++;
}

template <typename T, typename T2>
FindElem<T,T2>::FindElem()
{
    clear();
}

template <typename T, typename T2>
void FindElem<T,T2>::clear(void)
{
    n=0;
    ind=0;
    fv=T();
    p=T2();
    fv2=T();
}

template <typename T, typename T2>
int FindElem<T,T2>::getInd(void)
{
    return ind;
}

template <typename T, typename T2>
T FindElem<T,T2>::getVal(void)
{
    return fv;
}

template <typename T, typename T2>
T2 FindElem<T,T2>::getP(void)
{
    return p;
}

template <typename T, typename T2>
bool FindElem<T,T2>::valid(void)
{
    return (n!=0);
}

template <typename T, typename T2>
bool FindMax<T,T2>::func(T var, T local)
{
    if(var>local)
        return true;
    else
        return false;
}

template <typename T, typename T2>
FindMax<T,T2>::FindMax(std::vector<T> vars)
{
    this->clear();
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i]);
    }
}

template <typename T, typename T2>
FindMax<T,T2>::FindMax(std::vector<T> vars, std::vector<T2> pts)
{
    this->clear();
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i],pts[i]);
    }
}

template <typename T, typename T2>
FindMax<T,T2>::FindMax(void)
{
    this->clear();
}

template <typename T, typename T2>
bool FindMin<T,T2>::func(T var, T local)
{
    if(var<local)
        return true;
    else
        return false;
}

template <typename T, typename T2>
FindMin<T,T2>::FindMin(std::vector<T> vars)
{
    this->clear();
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i]);
    }
}

template <typename T, typename T2>
FindMin<T,T2>::FindMin(std::vector<T> vars, std::vector<T2> pts)
{
    this->clear();
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i],pts[i]);
    }
}

template <typename T, typename T2>
FindMin<T,T2>::FindMin(void)
{
    this->clear();
}

//template <typename T, typename T2>
//unsigned int OrderedVector<T,T2>::iter(T var) {
//    typename std::vector<T>::iterator it=fv.begin();
//    for(unsigned int i=0; i<fv.size(); i++)
//    {
//        if(func(var,fv[i]))
//        {
//            fv.insert(it,var);
//            return i;
//        }
//        it++;
//    }
//    fv.insert(fv.end(),var);
//    return fv.size()-1;
//}

template <typename T, typename T2>
unsigned int OrderedVector<T,T2>::iter(T var,T2 pt=T2()) {
    typename std::vector<T>::iterator it=fv.begin();
    typename std::vector<T2>::iterator itp=p.begin();
    for(unsigned int i=0; i<fv.size(); i++)
    {
        if(func(var,fv[i]))
        {
            fv.insert(it,var);
            p.insert(itp,pt);
            return i;
        }
        it++;
        itp++;
    }
    fv.insert(fv.end(),var);
    p.insert(p.end(),pt);
    return fv.size()-1;
}

template <typename T, typename T2>
OrderedVector<T,T2>::OrderedVector()
{
    clear();
}

template <typename T, typename T2>
void OrderedVector<T,T2>::clear(void)
{
    fv.clear();
    p.clear();
}

template <typename T, typename T2>
unsigned OrderedVector<T,T2>::getSize(void)
{
    return fv.size();
}

template <typename T, typename T2>
T OrderedVector<T,T2>::getVal(unsigned int ind)
{
    if(ind<fv.size())
        return fv[ind];
    else
        return T();
}

template <typename T, typename T2>
T2 OrderedVector<T,T2>::getP(unsigned int ind)
{
    if(ind<p.size())
        return p[ind];
    else
        return T2();
}

template <typename T, typename T2>
bool OrderedDecresc<T,T2>::func(T var, T comp)
{
    if(var>comp)
        return true;
    else
        return false;
}

template <typename T, typename T2>
OrderedDecresc<T,T2>::OrderedDecresc(void)
{
    this->clear();
}

template <typename T, typename T2>
bool OrderedCresc<T,T2>::func(T var, T comp)
{
    if(var<comp)
        return true;
    else
        return false;
}

template <typename T, typename T2>
OrderedCresc<T,T2>::OrderedCresc(void)
{
    this->clear();
}

int boundPos(int x, int max);

double boundAngleR(double x);

double boundAngleD(double x);

double boundAngleRN(double x);

double boundAngleDN(double x);

int angleD2I(double rtrd, int angle_res);

int angleR2I(double rtrd, int angle_res);

int incAngle(int p, int num);

int decAngle(int p, int num);

double pi(void);

template <typename T>
T angleR2D(T angle)
{
    return angle*180/pi();
}

template <typename T>
T angleD2R(T angle)
{
    return angle/180*pi();
}

int angleDiff(int a, int b, int res);

cv::Point2f operator*(cv::Mat M, const cv::Point2f p);

#endif // VECTOR_UTILS_HPP
