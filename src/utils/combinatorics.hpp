#ifndef COMBINATORICS_HPP
#define COMBINATORICS_HPP

#include <vector>

class Sequence {
public:
    std::vector<std::vector<int> > seq;
    std::vector<std::vector<int> > rem;
    Sequence();
    void append(Sequence b);
    void print(void);
};

Sequence permutations(std::vector<int> in, int size);

Sequence combinations(std::vector<int> in, unsigned int size);

Sequence combine(std::vector<int> g1, std::vector<int> g2, std::vector<int> gc);

Sequence permute(std::vector<int> g1, std::vector<int> g2);

#endif // COMBINATORICS_HPP
