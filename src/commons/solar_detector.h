
#ifndef SOLAR_DETECTOR_H
#define SOLAR_DETECTOR_H

#include <QtCore/qpoint.h>
#include <tuple>

#include "commons/frame.h"
#include "image_handlers/imagehandler.h"


FWD_PTR(SolarDetector)

struct solarDiscInfo {
        bool valid = false;
        int x;
        int y;
        float radius;
    };

class SolarDetector: public QObject, public ImageHandler
{
    Q_OBJECT
    DPTR 

public:
    SolarDetector();
    ~SolarDetector();

    //std::mutex guard;


    SolarDetector(const SolarDetector &)             = delete;
    SolarDetector(SolarDetector &&)                  = delete;
    SolarDetector &operator =(const SolarDetector &) = delete;
    SolarDetector &operator =(SolarDetector &&)      = delete;

    std::vector<cv::Vec3f> get_circles();
    std::vector<cv::Vec3f> _circles;

    

    


signals:
    void detection(const solarDiscInfo);

private:
    void doHandle(FrameConstPtr  frame) override;
};


Q_DECLARE_METATYPE(solarDiscInfo)

// /// Tracks image movement
// /** Works in block matching or centroid mode. In case of block matching, multiple tracking targets
//     may be specified (to protect against local disturbances, e.g. due to a passing bird/satellite).
//     Centroid mode is best suited for planets.

//     Tracking position is updated on every call to ImageHandler::doHandle().
// */
// class SolarDetector: public QObject, public ImageHandler
// {
//     Q_OBJECT
//     DPTR

// public:

//     enum class SolarTrackingMode { Disabled, Centroid, BlockMatching };

//     SolarDetector();
//     ~SolarDetector();

//     SolarDetector(const SolarDetector &)             = delete;
//     SolarDetector(SolarDetector &&)                  = delete;
//     SolarDetector &operator =(const SolarDetector &) = delete;
//     SolarDetector &operator =(SolarDetector &&)      = delete;

//     SolarTrackingMode getSolarTrackingMode() const;

//     /// Sets the centroid calculation area
//     /** Removes any block-matching targets. Returns 'false' on failure. */
//     bool setCentroidCalcRect(const QRect &rect);

//     /// Returns either the centroid position or the block matching targets' common position
//     QPoint getTrackingPosition() const;

//     /// Returns centroid calculation area and centroid position in the image
//     std::tuple<QRect, QPoint> getCentroidAreaAndPos() const;

//     /// Adds new target to track via block matching
//     /** Cancels centroid tracking (if enabled). Returns 'false' on failure. */
//     bool addBlockMatchingTarget(const QPoint &pos);

//     /// Cancels tracking and removes all targets
//     void clear();

//     /// Returns current positions of block matching targets
//     std::vector<QPoint> getBlockMatchingTargetPositions();

// signals:
//     /// Emitted when tracking target is lost (moves outside image or moves away too fast)
//     void targetLost();

// private:
//     void doHandle(FrameConstPtr  frame) override;
// };

#endif // SOLAR_DETECTOR_H
