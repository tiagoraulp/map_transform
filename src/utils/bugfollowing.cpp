#include "bugfollowing.hpp"

using namespace std;


BugFollowing::BugFollowing(cv::Mat con, cv::Mat con_ch, cv::Point ini): contours(con), contours_check(con_ch), pos(ini)
{
    chain.clear();
    prev_d=0;
    sign=1;
    run();
}
vector<Chain> BugFollowing::getChain(void)
{
    return chain;
}
cv::Mat BugFollowing::getContourChecked(void)
{
    return contours_check;
}

void BugFollowing::saveP(int d)
{
    contours_check.at<uchar>(pos.x,pos.y)=255;
    pos.x=pos.x+dx[d];
    pos.y=pos.y+dy[d];
    prev_d=(d+dir/2)%dir;
}

void BugFollowing::save(int d)
{
    Chain c(pos.x,pos.y,d);
    chain.push_back(c);
    saveP(d);
}

bool BugFollowing::iter(void)
{
    int act_dir;
    for(int d=1;d<=dir;d++)
    {
        act_dir=(prev_d+sign*d+dir)%dir;
        if( (pos.x+dx[act_dir])>=0 && (pos.x+dx[act_dir])<contours.rows && (pos.y+dy[act_dir])>=0 && (pos.y+dy[act_dir])<contours.cols )
        {
            if( contours.at<uchar>(pos.x+dx[act_dir],pos.y+dy[act_dir])==0)
            {
                Chain c(pos.x,pos.y,act_dir);
                saveP(act_dir);

                if(chain[chain.size()-1]==c)
                {
                    return true;
                }

                chain.push_back(c);

                if(chain[0]==c)
                {
                    return true;
                }
                break;
            }
        }
    }
    return false;
}

bool BugFollowing::check_direction(int sn, int d, int max_dir)
{
    bool valid_rotation=true;
    for(int dd=1;dd<=max_dir;dd++)
    {
        int act_dir=(prev_d+sn*dd+dir)%dir;
        if( (pos.x+dx[d]+dx[act_dir])>=0 && (pos.x+dx[d]+dx[act_dir])<contours.rows && (pos.y+dy[d]+dy[act_dir])>=0 && (pos.y+dy[d]+dy[act_dir])<contours.cols )
        {
            if( contours.at<uchar>(pos.x+dx[d]+dx[act_dir],pos.y+dy[d]+dy[act_dir])==0)
            {
                valid_rotation=false;
                break;
            }
            else
            {
                valid_rotation=true;
            }
        }
    }
    return valid_rotation;
}

void BugFollowing::initialization(void)
{
    for(int d=0;d<dir;d++)
    {
        if( (pos.x+dx[d])>=0 && (pos.x+dx[d])<contours.rows && (pos.y+dy[d])>=0 && (pos.y+dy[d])<contours.cols )
        {
            if(  contours.at<uchar>(pos.x+dx[d],pos.y+dy[d])==0 )
            {
                prev_d=(d+dir/2)%dir;
                int max_dir;

                if(prev_d==0 || prev_d==2 || prev_d==4 || prev_d==6)
                    max_dir=2;
                else
                    max_dir=1;

                if(check_direction(1,d,max_dir))
                {
                    sign=1;
                    save(d);
                    break;
                }
                else
                {
                    if(check_direction(-1,d,max_dir))
                    {
                        sign=-1;
                        save(d);
                        break;
                    }
                    else
                        continue;
                }
            }
        }
    }
}

void BugFollowing::run(void)
{
    bool cont_cond=true;

    while(cont_cond)
    {
        if(chain.size()!=0)
        {
            cont_cond=!iter();
        }
        else
        {
            initialization();
        }
    }
}
