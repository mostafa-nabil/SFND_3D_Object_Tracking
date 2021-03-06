
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;




// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName,4);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    //find matches included in the bounding box
    vector<cv::DMatch> matchesIn;
    double sumDist = 0;
    double meanDist = 0;
    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
    {
        if(boundingBox.roi.contains(kptsCurr[it->trainIdx].pt))
        {
            matchesIn.push_back(*it);
            //sum the distance of the matches
            sumDist += it->distance;
        }
    }
    cout << sumDist << endl;
    //calculate the mean of the distances
    meanDist = sumDist/matchesIn.size();

    //remove outliers

    for(auto it = matchesIn.begin(); it != matchesIn.end(); it++)
    {
        double deviation = abs(it->distance - meanDist)/meanDist;
        if (deviation > 50)
        {
            matchesIn.erase(it);
            it = matchesIn.begin();
        }
    }
    boundingBox.kptMatches = matchesIn;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios;

    
    for (auto it = kptMatches.begin(); it != kptMatches.end() - 1; it++)
    {
        cv::KeyPoint kptsCurrOuter = kptsCurr.at(it->trainIdx);
        cv::KeyPoint kptsPrevOuter = kptsPrev.at(it->queryIdx);

        for (auto it2 = kptMatches.begin()+1; it2 != kptMatches.end(); it2++)
        {
            double minDist = 100.0;
            cv::KeyPoint kptsCurrInner = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kptsPrevInner = kptsPrev.at(it2->queryIdx);

            double distCurr = cv::norm(kptsCurrOuter.pt - kptsCurrInner.pt);
            double distPrev = cv::norm(kptsPrevOuter.pt - kptsPrevInner.pt);
            double distRatio = 0;
            if(distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            {
                distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    if (distRatios.size() == 0)
    {
        cout << "NAAAAAAAAAAAAAAN" << endl;
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1.0 / frameRate;
    TTC = (-dT) / (1 - medDistRatio);
    cout <<"TTC cam " << TTC << endl;
    
    
}

//inserts element into sorted vector
void insertSorted(LidarPoint point, std::vector<double> &list )
{
    
    list.push_back(point.x);

    sort(list.begin(),list.end());

    // if(list.size() > 30)
    // {
    //     list.pop_back();
    // }
    
}

//returns the median of the list
double calcMedian(std::vector<double> &list)
{
    return list[(list.size()-1)/2];
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    //for current and previous lidar points, get the median of the closest points in x direction
    std::vector<double> listPrev;
    std::vector<double> listCurr;

    //for previous Lidar Points
    for(auto it =  lidarPointsPrev.begin(); it != lidarPointsPrev.end(); it++)
    {
        insertSorted(*it,listPrev);

    }
    //for current Lidar Points
    for(auto it =  lidarPointsCurr.begin(); it != lidarPointsCurr.end(); it++)
    {
        insertSorted(*it,listCurr);

    }

    //get median
    double x_medianPrev = calcMedian(listPrev);
    double x_medianCurr = calcMedian(listCurr);

    //get TTC
    double time = 1.0/frameRate;
    double distance = x_medianPrev - x_medianCurr;
    double velocity = distance / time;
    
    TTC = x_medianCurr/velocity;
    cout <<"TTC lidar " << TTC << endl;


}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //define previous and current bounding boxes
    vector<BoundingBox> bbsPrevious = prevFrame.boundingBoxes;
    vector<BoundingBox> bbsCurrent = currFrame.boundingBoxes;
    vector<cv::KeyPoint> kpPrevious = prevFrame.keypoints;
    vector<cv::KeyPoint> kpCurrent = currFrame.keypoints;
    
    // Loop over all combinations of bounding boxes
    //loop over bounding boxes in the previous frame
    for (auto itPrev = bbsPrevious.begin(); itPrev != bbsPrevious.end(); itPrev++)
    {   
        //vectors contained in this bounding box
        vector<cv::DMatch> containedMatches;
        //get matched keypoints contained in this bounding box
        for(auto itMatches = matches.begin(); itMatches!=matches.end(); itMatches++)
        {
            int queryIdx = itMatches->queryIdx;
            int trainIdx = itMatches->trainIdx;
           
            if(itPrev->roi.contains(kpPrevious[queryIdx].pt)== true)
            {
                containedMatches.push_back(*itMatches);
            }
        }
        
        int maxKeypoints = 0;
        int maxIdx = 255;
        //loop over bounding boxes in the current frame
        for(auto itCurr = bbsCurrent.begin(); itCurr != bbsCurrent.end(); itCurr++)
        {
            int currKeyPoints = 0;
            for(auto itMatches = containedMatches.begin(); itMatches!=containedMatches.end(); itMatches++)
            {
                int queryIdx = itMatches->queryIdx;
                int trainIdx = itMatches->trainIdx;
                if(itCurr->roi.contains(kpCurrent[trainIdx].pt))
                {
                    currKeyPoints++;
                }
            }

            if (currKeyPoints > maxKeypoints)
            {
                maxKeypoints = currKeyPoints;
                maxIdx = itCurr->boxID;
            }
        }
        if(maxKeypoints != 0)
        {
            bbBestMatches.insert({itPrev->boxID,maxIdx});
        }

    }
}
