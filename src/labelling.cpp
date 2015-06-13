#include "labelling.hpp"

std::vector<std::vector<cv::Point> >  label(const cv::Mat binary, int conn)
{
    std::vector<std::vector<cv::Point> > blobs;
    blobs.clear();

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32FC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < binary.rows; y++) {
        for(int x=0; x < binary.cols; x++) {
            if((int)label_image.at<float>(y,x) != 1 ) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), conn);

            std::vector<cv::Point>  blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if((int)label_image.at<float>(i,j) != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point(i,j));
                }
            }

            blobs.push_back(blob);

            label_count++;
        }
    }

    return blobs;
}

std::vector<cv::Point> label_seed(const cv::Mat binary, int conn, cv::Point seed)
{
    std::vector<cv::Point> blob;
    blob.clear();

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32FC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    if((int)label_image.at<float>(seed.x,seed.y) != 1 ) {
                return blob;
    }

    cv::Rect rect;
    cv::Point sd=cv::Point(seed.y, seed.x);
    cv::floodFill(label_image, sd, cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), conn);


    for(int i=rect.y; i < (rect.y+rect.height); i++) {
        for(int j=rect.x; j < (rect.x+rect.width); j++) {
            if((int)label_image.at<float>(i,j) != label_count) {
                continue;
            }

            blob.push_back(cv::Point(i,j));
        }
    }

    return blob;
}

