/*
 * Copyright (C) 2016  Marco Gulino <marco@gulinux.net>
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

#ifndef DISPLAYIMAGE_H
#define DISPLAYIMAGE_H

#include <QObject>
#include "dptr.h"
#include "image_handlers/imagehandler.h"
#include "commons/configuration.h"
#include "commons/solar_detector.h"
#include <opencv2/opencv.hpp>

class DisplayImage : public QObject, public ImageHandler
{
Q_OBJECT
public:
    ~DisplayImage();
    DisplayImage(const Configuration &configuration, QObject* parent = 0);
    void setRecording(bool recording);
    QRect imageRect() const;
signals:
  void gotImage(const QImage &);
  void displayFPS(double fps);
public slots:
  void create_qimages();
  void detectEdges(bool detect);
  void histogramEqualization(bool enable);
  void maximumSaturation(bool enable);
  //void updateSolarPosition(std::vector<cv::Vec3f> circles);
  void updateSolarPosition(solarDiscInfo circle);
  void quit();
  void read_settings();
private:

  void doHandle(FrameConstPtr frame) override;
  
  DPTR
};

#endif // DISPLAYIMAGE_H
