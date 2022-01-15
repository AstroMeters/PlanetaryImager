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
#include "commons/configuration.h"
#include <QDebug>
#include <QtCore/qrect.h>
#include <mutex>
#include <numeric>
#include <vector>
#include <QMetaType>
#include <math.h>

#include "solar_detector.h"

using namespace cv;
using namespace std;

DPTR_IMPL(SolarDetector)
{

    const Configuration &configuration;
    SolarDetector *q;
    std::mutex guard;
    solarDiscInfo solar_disc;

};

#define LOCK()  std::lock_guard<std::mutex> lock(d->guard)


SolarDetector::SolarDetector(const Configuration &configuration): dptr(configuration, this)
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
    d->solar_disc.valid = false;

//     //TODO: change frame fragments' endianess if needed
    cv::Mat gray, thr;

    std::cout << cv::mean(frame->mat()) << std::endl;

    // monochromatic
    if(frame->channels() == 1) {
        cv::medianBlur(frame->mat(), gray, 5);
    } else {
        //colour
        cv::cvtColor(frame->mat(), gray, cv::COLOR_RGB2GRAY );
        cv::medianBlur(gray, gray, 5);
    }

    // cv::HoughCircles(gray, _circles, cv::HOUGH_GRADIENT, 1,
    //              gray.rows/2,  // change this value to detect circles with different distances to each other
    //              d->configuration.solar_hough_param1(), d->configuration.solar_hough_param2(), d->configuration.solar_radius_min(), d->configuration.solar_radius_max()
    // );

    threshold(gray, gray, d->configuration.solar_hough_param1(), 255, 0);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(gray, contours, hierarchy, 0, cv::CHAIN_APPROX_NONE);
    vector<vector<Point>> contours_poly(contours.size());
    for (int i = 0; i<contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), contours_poly[i], 15, true);
        cv::Moments m=cv::moments(Mat(contours[i]));
        float area = cv::contourArea(Mat(contours[i]));
        float equi_radius = sqrt(4*area/M_PI)/2;

        int cx = (int) m.m10/m.m00;
        int cy = (int) m.m01/m.m00;

        //std::cout << cx << " ... " << cy << "   rad  " << equi_radius << std::endl;
        //drawContours(gray, contours_poly, i, Scalar(0, 255, 255), 2, 8);

        if(equi_radius > d->configuration.solar_radius_min()){
            d->solar_disc.x = cx;
            d->solar_disc.y = cy;
            d->solar_disc.radius = equi_radius;
            d->solar_disc.valid = true;
        }

    }

    emit detection(d->solar_disc);


    return;
}
