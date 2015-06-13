#ifndef VECTOR_UTILS_HPP
#define VECTOR_UTILS_HPP

#include <vector>

using namespace std;

template <typename T, typename T2=T>
class FindElem
{
protected:
    int n;
    int ind;
    T fv;
    T2 p;
    virtual bool func(T var)=0;
public:
    void iter(T var);
    void iter(T var,T2 pt);
    FindElem();
    void clear(void);
    int getInd(void);
    T getVal(void);
    T2 getP(void);
};

template <typename T, typename T2=T>
class FindMax : public FindElem<T,T2>
{
protected:
    bool func(T var);
public:
    FindMax(vector<T> vars);
    FindMax(vector<T> vars, vector<T2> pts);
    FindMax(void);
};

template <typename T,typename T2=T>
class FindMin : public FindElem<T,T2>
{
protected:
    bool func(T var);
public:
    FindMin(vector<T> vars);
    FindMin(vector<T> vars, vector<T2> pts);
    FindMin(void);
};

template <typename T, typename T2=T>
class OrderedVector
{
protected:
    vector<T> fv;
    vector<T2> p;
    virtual bool func(T var, T comp)=0;
public:
    unsigned int iter(T var);
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


template <typename T, typename T2>
void FindElem<T,T2>::iter(T var) {
    if(n==0)
    {
        ind=0;
        fv=var;
    }
    else if(func(var))
    {
        ind=n;
        fv=var;
    }
    n++;
}

template <typename T, typename T2>
void FindElem<T,T2>::iter(T var,T2 pt) {
    if(n==0)
    {
        ind=0;
        fv=var;
        p=pt;
    }
    else if(func(var))
    {
        ind=n;
        fv=var;
        p=pt;
    }
    n++;
}

template <typename T, typename T2>
FindElem<T,T2>::FindElem()
{
    n=0;
    ind=0;
    fv=T();
    p=T2();
}

template <typename T, typename T2>
void FindElem<T,T2>::clear(void)
{
    n=0;
    ind=0;
    fv=T();
    p=T2();
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
bool FindMax<T,T2>::func(T var)
{
    if(var>FindElem<T,T2>::fv)
        return true;
    else
        return false;
}

template <typename T, typename T2>
FindMax<T,T2>::FindMax(vector<T> vars)
{
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i]);
    }
}

template <typename T, typename T2>
FindMax<T,T2>::FindMax(vector<T> vars, vector<T2> pts)
{
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i],pts[i]);
    }
}

template <typename T, typename T2>
FindMax<T,T2>::FindMax(void)
{
}

template <typename T, typename T2>
bool FindMin<T,T2>::func(T var)
{
    if(var<this->fv)
        return true;
    else
        return false;
}

template <typename T, typename T2>
FindMin<T,T2>::FindMin(vector<T> vars)
{
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i]);
    }
}

template <typename T, typename T2>
FindMin<T,T2>::FindMin(vector<T> vars, vector<T2> pts)
{
    for(int i=0;i<vars.size();i++)
    {
        this->iter(vars[i],pts[i]);
    }
}

template <typename T, typename T2>
FindMin<T,T2>::FindMin(void)
{
}

template <typename T, typename T2>
unsigned int OrderedVector<T,T2>::iter(T var) {
    typename vector<T>::iterator it=fv.begin();
    for(unsigned int i=0; i<fv.size(); i++)
    {
        if(func(var,fv[i]))
        {
            fv.insert(it,var);
            return i;
        }
        it++;
    }
    fv.insert(fv.end(),var);
    return fv.size()-1;
}

template <typename T, typename T2>
unsigned int OrderedVector<T,T2>::iter(T var,T2 pt) {
    typename vector<T>::iterator it=fv.begin();
    typename vector<T2>::iterator itp=p.begin();
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
    fv.clear();
    p.clear();
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
}

int boundPos(int x, int max);

#endif // VECTOR_UTILS_HPP
