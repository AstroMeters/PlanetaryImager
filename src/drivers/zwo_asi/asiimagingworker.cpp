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

#include "asiimagingworker.h"
#include "zwoexception.h"
#include <atomic>
#include "commons/frame.h"
#include <array>

#define FRAMES_BUFFER 5
using namespace std;

DPTR_IMPL(ASIImagingWorker) {
  ASI_IMG_TYPE format;
  ASI_CAMERA_INFO info;
  int bin;
  QRect roi;
  atomic_long exposure_timeout;

  std::vector<uint8_t> buffer;
  size_t calcBufferSize();
  int getCVImageType();
  Frame::ColorFormat color_format;
  Frame::ColorFormat colorFormat() const;
#ifdef FRAMES_BUFFER
  array<FramePtr, FRAMES_BUFFER> frames;
#endif
  uint8_t current_frame = 0;
  FramePtr new_frame() const;
  uint8_t next_index();
};

ASIImagingWorker::ASIImagingWorker(const QRect& roi, int bin, const ASI_CAMERA_INFO& info, ASI_IMG_TYPE format)
    : dptr(format, info, bin, roi)
{
    qDebug() << "Starting imaging: imageFormat=" << d->format << ", d->roi: " << d->roi << ", bin: " << d->bin;
    cout << "Starting imaging: imageFormat=" << d->format << ", bin: " << d->bin;
    ASI_CHECK << ASISetROIFormat(d->info.CameraID, d->roi.width(), d->roi.height(), d->bin, d->format) << "Set format";
    ASI_CHECK << ASISetStartPos(d->info.CameraID, d->roi.x(), d->roi.y()) << "Set ROI position";
    ASI_CHECK << ASIStartVideoCapture(d->info.CameraID) << "Start video capture";
    d->buffer.resize(d->calcBufferSize());
    qDebug() << "Imaging started: imageFormat=" << d->format << ", roi: " << d->roi << ", bin: " << d->bin;

    static std::map<ASI_BAYER_PATTERN, Frame::ColorFormat> color_formats{
    {ASI_BAYER_RG, Frame::Bayer_RGGB},
    {ASI_BAYER_BG, Frame::Bayer_BGGR},
    {ASI_BAYER_GR, Frame::Bayer_GRBG},
    {ASI_BAYER_GB, Frame::Bayer_GBRG},
  };
  if(info.IsColorCam) {
    d->color_format = color_formats[info.BayerPattern];
  } else {
    d->color_format = Frame::Mono;
  }
  calc_exposure_timeout();
#ifdef FRAMES_BUFFER
  generate(begin(d->frames), end(d->frames), bind(&Private::new_frame, d.get()));
#endif
}

FramePtr ASIImagingWorker::Private::new_frame() const {
  // ASI CAMs are little endian
  return make_shared<Frame>( format == ASI_IMG_RAW16 ? 16 : 8,  colorFormat(), QSize{roi.width(), roi.height()}, Frame::LittleEndian);
}

uint8_t ASIImagingWorker::Private::next_index() {
#ifdef FRAMES_BUFFER
    return (current_frame++) % frames.size();
#else
    return 0;
#endif
}

ASIImagingWorker::~ASIImagingWorker()
{
    ASI_CHECK << ASIStopVideoCapture(d->info.CameraID) << "Stop capture";
    qDebug() << "Imaging stopped.";
}

void ASIImagingWorker::calc_exposure_timeout()
{
  long value;
  ASI_BOOL isAuto;
  ASI_CHECK << ASIGetControlValue(d->info.CameraID, ASI_EXPOSURE, &value, &isAuto) << "Getting exposure value";
  if(isAuto)
    d->exposure_timeout = -1;
  else
    d->exposure_timeout = value*2 / 1000 + 500;
  qDebug() << "Exposure timeout: exposure=" << value << ", timeout=" << d->exposure_timeout;
}



FramePtr ASIImagingWorker::shoot()
{
#ifdef FRAMES_BUFFER
  FramePtr frame = d->frames[d->next_index()];
#else
  FramePtr frame = d->new_frame();
#endif
  ASI_CHECK << ASIGetVideoData(d->info.CameraID, frame->data(), frame->size(), d->exposure_timeout) << "Capture frame";
  return frame;
}

size_t ASIImagingWorker::Private::calcBufferSize()
{
    auto base_size = roi.width() * roi.height();
    switch(format) {
    case ASI_IMG_RAW8:
    case ASI_IMG_Y8:
        return base_size;
    case ASI_IMG_RAW16:
        return base_size * 2;
    case ASI_IMG_RGB24:
        return base_size * 3;
    default:
        throw runtime_error("Format not supported");
    }
}

int ASIImagingWorker::Private::getCVImageType()
{
    switch(format) {
    case ASI_IMG_RAW8:
    case ASI_IMG_Y8:
        return CV_8UC1;
    case ASI_IMG_RAW16:
        return CV_16UC1;
    case ASI_IMG_RGB24:
        return CV_8UC3;
    default:
        throw runtime_error("Format not supported");
    }
}

Frame::ColorFormat ASIImagingWorker::Private::colorFormat() const {
  switch(format){
      case ASI_IMG_RGB24:
        return Frame::BGR;
      case ASI_IMG_Y8:
        return Frame::Mono;
    }
    return info.IsColorCam ? color_format : Frame::Mono;
}

QRect ASIImagingWorker::roi() const
{
  return d->roi;
}

ASI_IMG_TYPE ASIImagingWorker::format() const
{
  return d->format;
}

int ASIImagingWorker::bin() const
{
  return d->bin;
}


