#include <iostream>
#include <vector>

using namespace std;

class Sequence {
public:
    vector<vector<int> > seq;
    vector<vector<int> > rem;

    Sequence()
    {
        seq.clear();
        rem.clear();
    }

    void append(Sequence b)
    {
        seq.insert(seq.end(),b.seq.begin(),b.seq.end());
        rem.insert(rem.end(),b.rem.begin(),b.rem.end());
    }

//    Sequence & operator= (const Sequence & other)
//    {
//            if (this != &other) // protect against invalid self-assignment
//            {
//                seq=other.seq;
//                rem=other.rem;
//            }
//            // by convention, always return *this
//            return *this;
//    }

    void print(void)
    {
        for (int i=0;i<seq.size();i++)
        {
            cout<<"Permutation: ";
            for (int j=0;j<seq[i].size();j++)
            {
                cout<<seq[i][j]<<" ";
            }
            cout<<"Remainder: ";
            for (int j=0;j<rem[i].size();j++)
            {
                cout<<rem[i][j]<<" ";
            }
            cout<<endl;
        }
    }
};

Sequence permutations(vector<int> in, int size)
{
    Sequence perm, temp;
    vector<int> poss;

    if(size==0)
    {
        perm.seq.push_back(vector<int>(0));
        perm.rem.push_back(in);
        return perm;
    }

    for (int i=0;i<in.size();i++)
    {
        poss=in;
        poss.erase(poss.begin()+i);
        temp=permutations(poss,size-1);
        for(int j=0;j<temp.seq.size();j++)
            temp.seq[j].insert ( temp.seq[j].begin() , in[i]);
        perm.append(temp);
    }

    return perm;
}

Sequence combinations(vector<int> in, int size)
{
    Sequence comb, temp;
    vector<int> poss, poss_r;

    if(size==0)
    {
        comb.seq.push_back(vector<int>(0));
        comb.rem.push_back(in);
        return comb;
    }

    for (int i=0;i<in.size();i++)
    {
        poss=in;
        poss.erase(poss.begin(),poss.begin()+i+1);
        poss_r=vector<int>(in.begin(),in.begin()+i);
        temp=combinations(poss,size-1);
        for(int j=0;j<temp.seq.size();j++)
        {
            temp.seq[j].insert ( temp.seq[j].begin() , in[i]);
            temp.rem[j].insert ( temp.rem[j].begin() , poss_r.begin(), poss_r.end());
        }
        comb.append(temp);
    }

    return comb;
}

Sequence combine(vector<int> g1, vector<int> g2, vector<int> gc)
{
    Sequence ret, comb, temp, temp2;

    for(int i=0;i<=gc.size();i++)
    {
        temp=combinations(gc,i);
        comb.append(temp);
    }

    vector<int> temp_s, temp_s2;

    for(int i=0;i<comb.seq.size();i++)
    {
        temp_s=g1;
        temp_s.insert(temp_s.end(),comb.seq[i].begin(),comb.seq[i].end());

        temp_s2=g2;
        temp_s2.insert(temp_s2.end(),comb.rem[i].begin(),comb.rem[i].end());


        temp=permutations(temp_s,temp_s.size());

        for(int j=0;j<temp.seq.size();j++)
        {

            temp2=permutations(temp_s2,temp_s2.size());

            for(int k=0; k<temp2.seq.size();k++)
            {
                ret.seq.push_back(temp.seq[j]);
                ret.rem.push_back(temp2.seq[k]);
            }
        }
    }

    return ret;
}

main()
{
    vector<int> x,y,z;
    x.clear();
    y.clear();
    z.clear();
    x.push_back(3);
    x.push_back(5);
    //x.push_back(1);
    y.push_back(6);
    y.push_back(15);
    //y.push_back(90);
    z.push_back(0);
    z.push_back(2);
    //z.push_back(31);

    Sequence perm, comb, temp;

    for(int i=0;i<=x.size();i++)
    {
        temp=permutations(x,i);
        perm.append(temp);
    }

    for(int i=0;i<=x.size();i++)
    {
        temp=combinations(x,i);
        comb.append(temp);
    }

    cout<<"---> Permutations"<<endl;

    perm.print();

    cout<<"---> Combinations"<<endl;

    comb.print();

    perm=combine(x,y,z);

    cout<<"---> Permutations of paths for 2 Robots"<<endl;

    perm.print();

    cout<<"Total permutations: "<<perm.seq.size()<<endl;

}
