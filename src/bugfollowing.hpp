#ifndef BUGFOLLOWING_HPP
#define BUGFOLLOWING_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include "chain.hpp"

using namespace std;

class BugFollowing
{
private:
    const static int dir=8; // number of possible directions to go at any position
    //if (dir==4){
    //    static int dx[dir]={1, 0, -1, 0};
    //    static int dy[dir]={0, 1, 0, -1};
    //}
    //if (dir==8) {
    const int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};
    //}
    int prev_d;
    int sign;
    cv::Mat contours, contours_check;
    cv::Point pos;
    vector<Chain> chain;
    void run(void);
    bool iter(void);
    void initialization(void);
    void saveP(int d);
    void save(int d);
    bool check_direction(int sn, int d, int max_dir);
public:
    BugFollowing(cv::Mat con, cv::Mat con_ch, cv::Point ini);
    vector<Chain> getChain(void);
    cv::Mat getContourChecked(void);
};

#endif // BUGFOLLOWING_HPP
