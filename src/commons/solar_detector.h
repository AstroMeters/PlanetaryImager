
#ifndef SOLAR_DETECTOR_H
#define SOLAR_DETECTOR_H

#include <QtCore>
#include <QtCore/qpoint.h>
#include <tuple>

#include "commons/frame.h"
#include "image_handlers/imagehandler.h"
#include "commons/configuration.h"


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
    SolarDetector(const Configuration &configuration);
    ~SolarDetector();

    SolarDetector(const SolarDetector &)             = delete;
    SolarDetector(SolarDetector &&)                  = delete;
    SolarDetector &operator =(const SolarDetector &) = delete;
    SolarDetector &operator =(SolarDetector &&)      = delete;


signals:
    void detection(const solarDiscInfo);

private:
    void doHandle(FrameConstPtr  frame) override;
};


Q_DECLARE_METATYPE(solarDiscInfo)

#endif // SOLAR_DETECTOR_H
