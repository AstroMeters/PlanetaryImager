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

#include "solar_detector.h"


// cv::Rect toCvRect(const QRect &qr)
// {
//     return cv::Rect(qr.x(), qr.y(), qr.width(), qr.height());
// }


// /// Finds new position of 'refBlock' in 'img' using block matching
// QPoint findNewPos(const QPoint &oldPos, ///< Old position of 'refBlock's center in 'img'
//                   cv::Mat refBlock,
//                   cv::Mat img)
// {
//     // At first using a coarse step when trying to match 'refBlock'
//     // with 'img' at different positions. Once an approximate matching
//     // position is determined, the search continues around it repeatedly
//     // using a smaller step, until the step becomes 1.

    
//     unsigned searchStep = 4;
//     const int searchRadius = 32; // TODO: make it configurable

//     // Range of positions where 'refBlock' will be match-tested with 'img'.
//     struct
//     {
//         int xmin, ymin; // inclusive
//         int xmax, ymax; // exclusive
//     } searchRange = { .xmin = oldPos.x() - searchRadius,
//                       .ymin = oldPos.y() - searchRadius,
//                       .xmax = oldPos.x() + searchRadius,
//                       .ymax = oldPos.y() + searchRadius };

//     const int rbwidth = refBlock.cols;
//     const int rbheight = refBlock.rows;

//     QPoint bestPos{ 0, 0 };

//     cv::Mat absDiff = cv::Mat(refBlock.rows, refBlock.cols, refBlock.type());

//     while (searchStep)
//     {
//         // Min. sum of absolute differences between pixel values of
//         // the reference block and the image at candidate positions.
//         double minDiffSum = DBL_MAX;

//         // (x, y) = position in 'img' for which a block match test is performed
//         for (int y = searchRange.ymin; y < searchRange.ymax;  y += searchStep)
//             for (int x = searchRange.xmin; x < searchRange.xmax;  x += searchStep)
//             {
//                 cv::absdiff(refBlock,
//                             cv::Mat(img, cv::Rect{ x - rbwidth/2, y - rbheight/2, rbwidth, rbheight }),
//                             absDiff);

//                 const auto sumAbsDiffChannels = cv::sum(absDiff);
//                 const double sumAbsDiffs = std::accumulate(sumAbsDiffChannels.val, sumAbsDiffChannels.val + 4, 0.0);

//                 if (sumAbsDiffs < minDiffSum)
//                 {
//                     minDiffSum = sumAbsDiffs;
//                     bestPos = QPoint{ x, y };
//                 }
//             }

//         searchRange.xmin = bestPos.x() - searchStep;
//         searchRange.ymin = bestPos.y() - searchStep;
//         searchRange.xmax = bestPos.x() + searchStep;
//         searchRange.ymax = bestPos.y() + searchStep;

//         searchStep /= 2;
//     }

//     return bestPos;
// }


// QPoint findCentroid(cv::Mat img)
// {
//     if (img.channels() > 1)
//         cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);

//     const auto moments = cv::moments(img);
//     if (moments.m00 == 0.0)
//         return QPoint{ 0, 0 };
//     else
//         return QPoint{ static_cast<int>(moments.m10 / moments.m00),
//                        static_cast<int>(moments.m01 / moments.m00) };
// }


// struct BlockMatchingTarget
// {
//     /// Target position; corresponds with the middle of 'refBlock'
//     QPoint pos;
//     cv::Mat refBlock; ///< Reference block
// };


DPTR_IMPL(SolarDetector)
{
//     SolarTrackingMode mode = SolarTrackingMode::Disabled;

//     std::mutex guard; ///< Synchronizes accesses to 'targets' and 'centroid'
//     std::vector<BlockMatchingTarget> targets;
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
    std::cout << "SOLAR DETECTOR start..." << std::endl;
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
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 30, 500 // change the last two parameters (min_radius & max_radius) to detect larger circles
    );

    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        // cv::Point center = cv::Point(c[0], c[1]);
        // // circle center
        // cv::circle( src, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // // circle outline
        // int radius = c[2];
        // cv::circle(frame->mat(), center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
        std::cout << i << ": " <<  c[0] << " " << c[1] << "\t radius:" << c[2] << std::endl;

    }

//     d->prevFrame = frame;
//     if (d->mode == SolarTrackingMode::Disabled)
//         return;

//     LOCK();

//     switch (d->mode) {
//         case SolarTrackingMode::BlockMatching:
//             for (auto &target: d->targets)
//                 target.pos = findNewPos(target.pos, target.refBlock, frame->mat());
//             break;

//         case SolarTrackingMode::Centroid:
//             {
//                 const QPoint newPos = findCentroid(cv::Mat(frame->mat(), toCvRect(d->centroid.area)));
//                 const QPoint delta = newPos - d->centroid.pos;
//                 const QRect oldArea = d->centroid.area;
//                 d->centroid.area.moveTopLeft(d->centroid.area.topLeft() + delta);
//                 const QRect intersection = d->centroid.area.intersected(QRect{ 0, 0, frame->mat().size[1], frame->mat().size[0] });
//                 if (intersection.width() != oldArea.width() ||
//                     intersection.height() != oldArea.height())
//                 {
//                     qWarning() << "Centroid calculation rect outside image";
//                     emit targetLost();
//                     d->mode = SolarTrackingMode::Disabled;
//                 }
//             }
//             break;
//         default:
//             return;
//     }
    return;
}


// void SolarDetector::clear()
// {
//     LOCK();
//     d->mode = SolarTrackingMode::Disabled;
//     d->targets.clear();
// }


// /// Returns current positions of block matching targets
// std::vector<QPoint> SolarDetector::getBlockMatchingTargetPositions()
// {
//     LOCK();
//     std::vector<QPoint> result;
//     for (const auto &target: d->targets)
//         result.push_back(target.pos);

//     return result;
// }


// SolarDetector::SolarTrackingMode SolarDetector::getSolarTrackingMode() const
// {
//     LOCK();
//     return d->mode;
// }


// QPoint SolarDetector::getTrackingPosition() const
// {
//     LOCK();

//     switch (d->mode)
//     {
//         //case SolarTrackingMode::Centroid: return m_Centroid.area.GetTopLeft() + m_Centroid.pos;

//         case SolarTrackingMode::BlockMatching:
//             return d->targets.at(0).pos - d->blockMatchingReportedOffset;
//     }

//     return { 0, 0 };
// }


// bool SolarDetector::setCentroidCalcRect(const QRect &rect)
// {
//     if (!d->prevFrame)
//     {
//         qWarning() << "Attempted to start tracking before any image has been received";
//         return false;
//     }

//     const cv::Rect frameRect = cv::Rect{ 0, 0, d->prevFrame->mat().size[1], d->prevFrame->mat().size[0] };
//     const cv::Rect intersection = frameRect & cv::Rect{ rect.x(), rect.y(), rect.width(), rect.height() };
//     if (intersection.width != rect.width() ||
//         intersection.height != rect.height())
//     {
//         qWarning() << "Centroid calculation rectangle outside image";
//         return false;
//     }

//     LOCK();
//     d->targets.clear();
//     d->mode = SolarTrackingMode::Centroid;
//     d->blockMatchingReportedOffset = QPoint{ 0, 0 };
//     d->centroid.area = rect;
//     //TODO: change frame fragment's endianess if needed
//     d->centroid.pos = findCentroid(cv::Mat(d->prevFrame->mat(), toCvRect(rect)));

//     return true;
// }


// std::tuple<QRect, QPoint> SolarDetector::getCentroidAreaAndPos() const
// {
//     return std::make_tuple(d->centroid.area, d->centroid.pos);
// }
