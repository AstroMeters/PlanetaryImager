/*
 * Copyright (C) 2017 Filip Szczerek <ga.software@yahoo.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/opencv.hpp>
#include <QDebug>
#include <QtCore/qrect.h>
#include <mutex>
#include <numeric>
#include <vector>
#include <QMetaType>


#include "solar_detector.h"



struct BlockMatchingTarget
{
    /// Target position; corresponds with the middle of 'refBlock'
    
};


DPTR_IMPL(SolarDetector)
{
//     SolarTrackingMode mode = SolarTrackingMode::Disabled;

    std::mutex guard; ///< Synchronizes accesses to 'targets' and 'centroid'
    solarDiscInfo solar_disc;
    //std::vector<BlockMatchingTarget> targets;
//     struct
//     {
//         /// Fragment of the image to calculate the centroid for
//         /** Keeps its position (upper-left corner) constant relative to 'pos'. */
//         QRect area;
//         QPoint pos; ///< Desired position of the 'area's centroid relative to area's origin
//     } centroid;

//     /** Initially equals (0, 0) and corresponds to the first element in 'targets'. If the first element
//         later gets removed, the value will change to the new first element's coordinates and will be
//         updated with it, so that GetTrackingPosition()'s result does not suddenly change. */
//     QPoint blockMatchingReportedOffset = QPoint{ 0, 0 };

//     FrameConstPtr prevFrame;
};

#define LOCK()  std::lock_guard<std::mutex> lock(d->guard)


SolarDetector::SolarDetector(): dptr()
{

qRegisterMetaType<solarDiscInfo>("solarDiscInfo");
}


SolarDetector::~SolarDetector()
{
}


// bool SolarDetector::addBlockMatchingTarget(const QPoint &pos)
// {
//     if (!d->prevFrame)
//     {
//         qWarning() << "Attempted to start tracking before any image has been received";
//         return false;
//     }

//     constexpr unsigned BBOX_SIZE = 32; //TODO: make it configurable

//     const cv::Rect refBlockRect = cv::Rect{ static_cast<int>(pos.x() - BBOX_SIZE/2),
//                                             static_cast<int>(pos.y() - BBOX_SIZE/2),
//                                             BBOX_SIZE, BBOX_SIZE };

//     const cv::Rect frameRect = cv::Rect{ 0, 0, d->prevFrame->mat().size[1], d->prevFrame->mat().size[0] };

//     const auto intersection = frameRect & refBlockRect;

//     if (intersection.width != refBlockRect.width ||
//         intersection.height != refBlockRect.height)
//     {
//         qWarning() << "Reference block outside image";
//         return false;
//     }

//     //TODO: change frame fragment's endianess if needed
//     BlockMatchingTarget newTarget{ pos, cv::Mat(d->prevFrame->mat(), refBlockRect).clone() };

//     LOCK();
//     d->mode = SolarTrackingMode::BlockMatching;
//     d->targets.push_back(newTarget);

//     return true;
// }


void SolarDetector::doHandle(FrameConstPtr frame)
{
//     //TODO: change frame fragments' endianess if needed
    cv::Mat gray, thr;
    
    std::cout << cv::mean(frame->mat()) << std::endl;
    
    cv::cvtColor(frame->mat(), gray, cv::COLOR_RGB2GRAY );
    cv::medianBlur(gray, gray, 5);
    //std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, _circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/2,  // change this value to detect circles with different distances to each other
                 100, 50, 30, 500 // change the last two parameters (min_radius & max_radius) to detect larger circles
    );


    d->solar_disc.valid = false;
    if(_circles.size() > 0){ 
        d->solar_disc.x = _circles[0][0];
        d->solar_disc.y = _circles[0][1];
        d->solar_disc.radius = _circles[0][2];
        d->solar_disc.valid = true;
    }

    emit detection(d->solar_disc);
    

    return;
}

std::vector<cv::Vec3f> SolarDetector::get_circles()
{
    LOCK();
    return _circles;
}
