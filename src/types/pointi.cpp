#include "pointi.hpp"

PointI::PointI(int a, int b){
    i=a;
    j=b;
}

PointI::PointI(){
    i=0;
    j=0;
}

bool PointI::operator==(const PointI &other) const {
    return (i==other.i) && (j==other.j);
}

bool PointI::operator!=(const PointI &other) const {
  return !(*this == other);
}
