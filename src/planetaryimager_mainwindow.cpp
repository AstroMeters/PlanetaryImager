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

#include "planetaryimager_mainwindow.h"
#include "drivers/imager.h"
#include "ui_planetaryimager_mainwindow.h"
#include "commons/filesystembrowser.h"
#include <functional>
#include "commons/utils.h"
#include "widgets/statusbarinfowidget.h"
#include <QLabel>
#include <QDoubleSpinBox>
#include <QThread>
#include <QFileDialog>
#include <QDateTime>
#include <QMetaType>
#include <QtConcurrent/QtConcurrent>
#include "commons/fps_counter.h"
#include "widgets/cameracontrolswidget.h"
#include "widgets/configurationdialog.h"
#include "commons/configuration.h"
#include <QMutex>
#include <QMessageBox>
#include "widgets/recordingpanel.h"
#include "widgets/camerainfowidget.h"
#include "widgets/histogramwidget.h"
#include "widgets/mount_widget.h"
#include "Qt/zoomableimage.h"
#include <QGridLayout>
#include <QToolBar>
#include <QWhatsThis>
#include "Qt/qt_strings_helper.h"
#include "Qt/qt_functional.h"
#include <QGraphicsScene>
#include <QFileInfo>
#include <QDesktopServices>
#include <memory>
#include <vector>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <opencv2/opencv.hpp>
#include <vector>

#include "widgets/editroidialog.h"

#include "image_handlers/imagehandler.h"
#include "image_handlers/frontend/histogram.h"
#include "image_handlers/frontend/displayimage.h"
#include "image_handlers/saveimages.h"
#include "image_handlers/threadimagehandler.h"

#include "commons/messageslogger.h"
#include "commons/definitions.h"
#include "c++/stlutils.h"
#include "commons/exposuretimer.h"
#include "Qt/qt_functional.h"
#include "commons/tracking.h"
#include "commons/solar_detector.h"
#include <opencv2/opencv.hpp>

#include "planetaryimager.h"
#include "drivers/driver.h"
#include "mainwindowwidgets.h"

#include <iostream>

using namespace GuLinux;
using namespace std;
using namespace std::placeholders;

Q_DECLARE_METATYPE(cv::Mat)
//Q_DECLARE_METATYPE(SolarDetector::solarDiscInfo)

DPTR_IMPL(PlanetaryImagerMainWindow)
{

  PlanetaryImagerPtr planetaryImager;
  FilesystemBrowserPtr filesystemBrowser;
  MainWindowWidgetsPtr main_window_widgets;
  static PlanetaryImagerMainWindow *q;
  unique_ptr<Ui::PlanetaryImagerMainWindow> ui;
  Imager *imager = nullptr;
  void rescan_devices();
  void saveWindowGeometry();

  StatusBarInfoWidget *statusbar_info_widget;
  shared_ptr<DisplayImage> displayImage;
  HistogramPtr histogram;
  shared_ptr<ImgTracker> imgTracker;
  shared_ptr<SolarDetector> solarDetector;
  CameraControlsWidget *cameraSettingsWidget = nullptr;
  CameraInfoWidget *cameraInfoWidget = nullptr;
  HistogramWidget *histogramWidget = nullptr;
  ConfigurationDialog *configurationDialog;
  EditROIDialog *editROIDialog;

  RecordingPanel *recording_panel;
  ExposureTimer exposure_timer;

  MountWidget *mount_widget;
  ImageHandlerPtr imageHandler;

  /// Contains elements of the "informational overlay", added to the graphics scene of 'displayImage'
  struct
  {
    std::vector<QGraphicsEllipseItem *> trackingTargets;
    QGraphicsRectItem *centroidArea = nullptr;
    //TODO: show centroid position
  } infoOverlay;

  //std::vector<cv::Vec3f> detectet_solar_disc;
  void cameraDisconnected();
  void enableUIWidgets(bool cameraConnected);
  void editROI();
  ZoomableImage *image_widget;

  void onImagerInitialized(Imager * imager);
  void onCamerasFound();

  enum class SelectionMode
  {
    None,
    ROI,
    VirtualROI,
    AddTrackingTarget,
    SelectCentroidRect
  } selection_mode = SelectionMode::None;
};

PlanetaryImagerMainWindow *PlanetaryImagerMainWindow::Private::q = nullptr;

void PlanetaryImagerMainWindow::updateInfoOverlay()
{
  switch (d->imgTracker->getTrackingMode())
  {
  case ImgTracker::TrackingMode::BlockMatching:
  {
    const auto positions = d->imgTracker->getBlockMatchingTargetPositions();
    for (size_t i = 0; i < d->infoOverlay.trackingTargets.size(); i++)
    {
      d->infoOverlay.trackingTargets[i]->setPos(d->image_widget->getImgTransform().map(positions.at(i)));
    }
  }
  break;

  case ImgTracker::TrackingMode::Centroid:
  {
    const auto imgT = d->image_widget->getImgTransform();
    const auto centroid = d->imgTracker->getCentroidAreaAndPos();
    d->infoOverlay.centroidArea->setRect(imgT.mapRect(std::get<0>(centroid)));
  }
  break;

  case ImgTracker::TrackingMode::Disabled:
    for (auto *i : d->infoOverlay.trackingTargets)
    {
      d->image_widget->scene()->removeItem(i);
      delete i;
    }
    d->infoOverlay.trackingTargets.clear();

    if (d->infoOverlay.centroidArea)
    {
      d->image_widget->scene()->removeItem(d->infoOverlay.centroidArea);
      delete d->infoOverlay.centroidArea;
      d->infoOverlay.centroidArea = nullptr;
    }
    break;
  }
}

PlanetaryImagerMainWindow::~PlanetaryImagerMainWindow()
{
  LOG_F_SCOPE
  if (d->imager)
  {
    d->imager->destroy();
  }
  d->displayImage->quit();
  d->planetaryImager->quit();
}

void PlanetaryImagerMainWindow::Private::saveWindowGeometry()
{
  planetaryImager->configuration().set_main_window_geometry(q->saveGeometry());
}

PlanetaryImagerMainWindow::PlanetaryImagerMainWindow(
    const PlanetaryImagerPtr &planetaryImager,
    const ImageHandlersPtr &imageHandlers,
    const FilesystemBrowserPtr &filesystemBrowser,
    const QString &logFilePath,
    QWidget *parent,
    Qt::WindowFlags flags)
    : QMainWindow(parent, flags), dptr(planetaryImager, filesystemBrowser)
{
qRegisterMetaType<solarDiscInfo>("solarDiscInfo");


  Private::q = this;
  d->ui.reset(new Ui::PlanetaryImagerMainWindow);

  d->ui->setupUi(this);
  d->main_window_widgets = make_shared<MainWindowWidgets>(this, d->ui->menuWindow, planetaryImager->configuration());

  restoreGeometry(d->planetaryImager->configuration().main_window_geometry());
  setWindowIcon(QIcon::fromTheme("planetary_imager"));
  d->ui->recording->setWidget(d->recording_panel = new RecordingPanel{d->planetaryImager->configuration(), filesystemBrowser});
  d->configurationDialog = new ConfigurationDialog(d->planetaryImager->configuration(), this);
  d->displayImage = make_shared<DisplayImage>(d->planetaryImager->configuration());
  d->histogram = make_shared<Histogram>(d->planetaryImager->configuration());
  d->ui->histogram->setWidget(d->histogramWidget = new HistogramWidget(d->histogram, d->planetaryImager->configuration()));
  d->ui->statusbar->addPermanentWidget(d->statusbar_info_widget = new StatusBarInfoWidget(), 1);

  if (HAVE_LIBINDI == 1)
  {
    d->ui->mount->setWidget(d->mount_widget = new MountWidget());
  }

  d->imgTracker = make_shared<ImgTracker>();
  d->solarDetector = make_shared<SolarDetector>(d->planetaryImager->configuration());

  imageHandlers->push_back(d->displayImage);
  imageHandlers->push_back(d->histogram);
  imageHandlers->push_back(d->imgTracker);
  imageHandlers->push_back(d->solarDetector);

  connect(d->planetaryImager.get(), &PlanetaryImager::camerasChanged, this, bind(&Private::onCamerasFound, d.get()));
  connect(d->planetaryImager.get(), &PlanetaryImager::cameraConnected, this, [=]
          { d->onImagerInitialized(d->planetaryImager->imager()); });
  d->ui->image->setLayout(new QGridLayout);
  d->ui->image->layout()->setMargin(0);
  d->ui->image->layout()->setSpacing(0);
  d->ui->image->layout()->addWidget(d->image_widget = new ZoomableImage(false));
  d->image_widget->scene()->setBackgroundBrush(QBrush{Qt::black, Qt::Dense4Pattern});
  connect(d->image_widget, &ZoomableImage::zoomLevelChanged, d->statusbar_info_widget, &StatusBarInfoWidget::zoom);
  d->statusbar_info_widget->zoom(d->image_widget->zoomLevel());
  for (auto item : d->image_widget->actions())
    d->ui->menuView->insertAction(d->ui->actionEdges_Detection, item);
  d->ui->menuView->insertSeparator(d->ui->actionEdges_Detection);

  d->image_widget->actions()[ZoomableImage::Actions::ZoomIn]->setShortcut({Qt::CTRL + Qt::Key_Plus});
  d->image_widget->actions()[ZoomableImage::Actions::ZoomIn]->setIcon(QIcon{":/resources/zoom_in.png"});
  d->image_widget->actions()[ZoomableImage::Actions::ZoomOut]->setShortcut({Qt::CTRL + Qt::Key_Minus});
  d->image_widget->actions()[ZoomableImage::Actions::ZoomOut]->setIcon(QIcon{":/resources/zoom_out.png"});
  d->image_widget->actions()[ZoomableImage::Actions::ZoomFit]->setShortcut({Qt::CTRL + Qt::Key_Space});
  d->image_widget->actions()[ZoomableImage::Actions::ZoomFit]->setIcon(QIcon{":/resources/fit_window.png"});
  d->image_widget->actions()[ZoomableImage::Actions::ZoomRealSize]->setShortcut({Qt::CTRL + Qt::Key_Backspace});
  d->image_widget->actions()[ZoomableImage::Actions::ZoomRealSize]->setIcon(QIcon{":/resources/real_size.png"});
  d->image_widget->toolbar()->setWindowTitle("Image Control");
  d->image_widget->toolbar()->setObjectName("imageToolbar");

  d->main_window_widgets->add_toolbar(d->ui->imageManipulation);
  d->main_window_widgets->add_toolbar(d->ui->ROIToolbar);
  d->main_window_widgets->add_toolbar(d->image_widget->toolbar(), true);
  QToolBar *helpToolBar = new QToolBar(tr("Help"));
  auto whatsThis = QWhatsThis::createAction();
  whatsThis->setIcon(QIcon{":/resources/help.png"});
  helpToolBar->addAction(whatsThis);
  helpToolBar->setObjectName("help");
  d->main_window_widgets->add_toolbar(helpToolBar, true);

  d->image_widget->toolbar()->setFloatable(true);
  d->image_widget->toolbar()->setMovable(true);
  connect(d->ui->actionAbout, &QAction::triggered, bind(&QMessageBox::about, this, tr("About"), tr("%1 version %2.\nFast imaging capture software for planetary imaging").arg(qApp->applicationDisplayName()).arg(qApp->applicationVersion())));
  connect(d->ui->actionAbout_Qt, &QAction::triggered, &QApplication::aboutQt);
  connect(d->ui->action_devices_rescan, &QAction::triggered, bind(&Private::rescan_devices, d.get()));
  connect(d->ui->actionShow_settings, &QAction::triggered, bind(&QDialog::show, d->configurationDialog));
  connect(d->configurationDialog, &QDialog::accepted, this, bind(&DisplayImage::read_settings, d->displayImage), Qt::DirectConnection);
  connect(d->configurationDialog, &QDialog::accepted, this, bind(&Histogram::read_settings, d->histogram), Qt::DirectConnection);
  connect(d->configurationDialog, &QDialog::accepted, this,
          [this]()
          {
            auto *imager = d->planetaryImager->imager();
            if (imager)
              imager->setCaptureEndianess(d->planetaryImager->configuration().capture_endianess());
          });

  connect(d->recording_panel, &RecordingPanel::start, [=]
          { d->planetaryImager->saveImages()->startRecording(d->imager); });
  connect(d->recording_panel, &RecordingPanel::stop, bind(&SaveImages::endRecording, d->planetaryImager->saveImages()));
  connect(d->recording_panel, &RecordingPanel::setPaused, bind(&SaveImages::setPaused, d->planetaryImager->saveImages(), _1));

  connect(d->planetaryImager->saveImages().get(), &SaveImages::recording, this, bind(&DisplayImage::setRecording, d->displayImage, true), Qt::QueuedConnection);
  connect(d->planetaryImager->saveImages().get(), &SaveImages::recording, this, bind(&Histogram::setRecording, d->histogram, true), Qt::QueuedConnection);
  connect(d->planetaryImager->saveImages().get(), &SaveImages::recording, d->recording_panel, bind(&RecordingPanel::recording, d->recording_panel, true, _1), Qt::QueuedConnection);
  connect(d->planetaryImager->saveImages().get(), &SaveImages::finished, d->recording_panel, bind(&RecordingPanel::recording, d->recording_panel, false, QString{}), Qt::QueuedConnection);
  connect(
      d->planetaryImager->saveImages().get(), &SaveImages::finished, this, [=]
      {
        QTimer::singleShot(5000, bind(&DisplayImage::setRecording, d->displayImage, false));
        QTimer::singleShot(5000, bind(&Histogram::setRecording, d->histogram, false));
      },
      Qt::QueuedConnection);

  d->main_window_widgets->add_dock(d->ui->chipInfoWidget);
  d->main_window_widgets->add_dock(d->ui->camera_settings);
  d->main_window_widgets->add_dock(d->ui->recording);
  d->main_window_widgets->add_dock(d->ui->histogram);
  if (DISABLE_TRACKING == 0 && HAVE_LIBINDI == 1)
  {
    d->main_window_widgets->add_dock(d->ui->mount);
  }
  else
  {
    d->ui->mount->hide();
    delete d->ui->mount;
  }
  d->main_window_widgets->load();

  qDebug() << "file " << logFilePath << "exists: " << QFile::exists(logFilePath);
  d->ui->actionOpen_log_file_folder->setMenuRole(QAction::ApplicationSpecificRole);
  d->ui->actionOpen_log_file_folder->setVisible(!logFilePath.isEmpty() && QFile::exists(logFilePath));
  connect(d->ui->actionOpen_log_file_folder, &QAction::triggered, this, [=]
          {
#ifdef Q_OS_MAC
            QStringList args;
            args << "-e";
            args << "tell application \"Finder\"";
            args << "-e";
            args << "activate";
            args << "-e";
            args << "select POSIX file \"" + logFilePath + "\"";
            args << "-e";
            args << "end tell";
            QProcess::startDetached("osascript", args);
#else
        auto logFileDirectory = QFileInfo{logFilePath}.dir().path();
        if (!QDesktopServices::openUrl(logFileDirectory))
        {
          MessagesLogger::instance()->queue(MessagesLogger::Warning, tr("Log file"), tr("Unable to open your log file. You can open it manually at this position: %1") % logFileDirectory);
        }
#endif
          });
  connect(MessagesLogger::instance(), &MessagesLogger::message, this, bind(&PlanetaryImagerMainWindow::notify, this, _1, _2, _3, _4), Qt::QueuedConnection);
  connect(d->displayImage.get(), &DisplayImage::gotImage, this, bind(&ZoomableImage::setImage, d->image_widget, _1), Qt::QueuedConnection);
  connect(d->displayImage.get(), &DisplayImage::gotImage, this, bind(&PlanetaryImagerMainWindow::updateInfoOverlay, this), Qt::QueuedConnection);
  connect(d->imgTracker.get(), &ImgTracker::targetLost, this, bind(&PlanetaryImagerMainWindow::updateInfoOverlay, this), Qt::QueuedConnection);

  connect(d->ui->actionNight_Mode, &QAction::toggled, this, [=](bool checked)
          { qApp->setStyleSheet(checked ? R"_(
        * { background-color: rgb(40, 0, 0); color: rgb(220, 220, 220); }
      )_"
                                        : ""); });

  connect(d->displayImage.get(), &DisplayImage::displayFPS, d->statusbar_info_widget, &StatusBarInfoWidget::displayFPS, Qt::QueuedConnection);
  connect(d->planetaryImager->saveImages().get(), &SaveImages::saveFPS, d->recording_panel, &RecordingPanel::saveFPS, Qt::QueuedConnection);
  connect(d->planetaryImager->saveImages().get(), &SaveImages::meanFPS, d->recording_panel, &RecordingPanel::meanFPS, Qt::QueuedConnection);
  connect(d->planetaryImager->saveImages().get(), &SaveImages::savedFrames, d->recording_panel, &RecordingPanel::saved, Qt::QueuedConnection);
  connect(d->planetaryImager->saveImages().get(), &SaveImages::droppedFrames, d->recording_panel, &RecordingPanel::dropped, Qt::QueuedConnection);
  connect(d->ui->actionDisconnect, &QAction::triggered, d->planetaryImager.get(), &PlanetaryImager::closeImager);
  connect(d->solarDetector.get(), &SolarDetector::detection, d->displayImage.get(), &DisplayImage::updateSolarPosition);

  connect(d->ui->actionQuit, &QAction::triggered, this, &QWidget::close);
  connect(d->ui->actionQuit, &QAction::triggered, this, &PlanetaryImagerMainWindow::quit);
  d->enableUIWidgets(false);

  QtConcurrent::run(bind(&DisplayImage::create_qimages, d->displayImage));

  connect(d->ui->actionEdges_Detection, &QAction::toggled, d->displayImage.get(), &DisplayImage::detectEdges);
  connect(d->ui->actionStretch_histogram, &QAction::toggled, d->displayImage.get(), &DisplayImage::histogramEqualization);
  connect(d->ui->actionStretch_colour_saturation, &QAction::toggled, d->displayImage.get(), &DisplayImage::maximumSaturation);

  connect(d->ui->actionClear_ROI, &QAction::triggered, [&]
          { d->imager->clearROI(); });
  connect(d->ui->actionSelect_ROI, &QAction::triggered, [&]
          {
            d->selection_mode = Private::SelectionMode::ROI;
            d->image_widget->startSelectionMode(ZoomableImage::SelectionMode::Rect);
          });

  //connect(d->ui->actionClear_ROIVirtual, &QAction::triggered, [&]  { d->imager->clearROIVirtual(); });
  connect(d->ui->actionSelect_ROIVirtual, &QAction::triggered, [&]
          {
            d->selection_mode = Private::SelectionMode::VirtualROI;
            d->image_widget->startSelectionMode(ZoomableImage::SelectionMode::Rect);
          });

  connect(d->ui->actionEdit_ROI, &QAction::triggered, this, bind(&Private::editROI, d.get()));
  QMap<Private::SelectionMode, function<void(const QRect &)>> handle_selection{
      {Private::SelectionMode::None, [](const QRect &) {}},
      {Private::SelectionMode::ROI, [&](const QRect &rect)
       { d->imager->setROI(rect.normalized()); }},
      {Private::SelectionMode::VirtualROI, [&](const QRect &rect)
       {
         std::cout << "RECT " << rect.normalized().top() << std::endl;
         //d->imager->setROI(rect.normalized());
       }},
      {Private::SelectionMode::SelectCentroidRect, [&](const QRect &rect)
       {
         if (d->imgTracker->setCentroidCalcRect(rect))
         {
           for (auto *i : d->infoOverlay.trackingTargets)
           {
             d->image_widget->scene()->removeItem(i);
             delete i;
           }
           d->infoOverlay.trackingTargets.clear();

           if (d->infoOverlay.centroidArea)
           {
             d->image_widget->scene()->removeItem(d->infoOverlay.centroidArea);
             delete d->infoOverlay.centroidArea;
           }

           auto *r = new QGraphicsRectItem(d->image_widget->getImgTransform().mapRect(rect));
           r->setPen(QPen{QColor{0, 255, 0, 255}});
           r->setBrush(QBrush{Qt::NoBrush});
           r->setZValue(1);

           d->infoOverlay.centroidArea = r;
           d->image_widget->scene()->addItem(d->infoOverlay.centroidArea);

           d->ui->actionDisableTracking->setEnabled(true);
         }
       }},
  };
  connect(d->image_widget, &ZoomableImage::selectedRect, [this, handle_selection](const QRectF &rect)
          {
            handle_selection[d->selection_mode](rect.toRect());
            d->selection_mode = Private::SelectionMode::None;
          });
  connect(&d->exposure_timer, &ExposureTimer::progress, [=](double, double elapsed, double remaining)
          { d->statusbar_info_widget->showMessage("Exposure: %1s, remaining: %2s"_q % QString::number(elapsed, 'f', 1) % QString::number(remaining, 'f', 1), 1000); });
  connect(&d->exposure_timer, &ExposureTimer::finished, [=]
          { d->statusbar_info_widget->clearMessage(); });

  d->editROIDialog = new EditROIDialog(this);
  connect(d->editROIDialog, &EditROIDialog::roiSelected, this, [=](const QRect &roi)
          { d->imager->setROI(roi); });
  auto readTemperature = new QTimer{this};
  connect(readTemperature, &QTimer::timeout, this, [=]
          {
            if (d->imager)
              QMetaObject::invokeMethod(d->imager, "readTemperature", Qt::QueuedConnection);
          });

  connect(d->ui->actionAddTrackingTarget, &QAction::triggered, [&]
          {
            d->selection_mode = Private::SelectionMode::AddTrackingTarget;
            d->image_widget->startSelectionMode(ZoomableImage::SelectionMode::Point);
          });

  connect(d->ui->actionSetCentroidArea, &QAction::triggered, [&]
          {
            d->selection_mode = Private::SelectionMode::SelectCentroidRect;
            d->image_widget->startSelectionMode(ZoomableImage::SelectionMode::Rect);
          });

  connect(d->image_widget, &ZoomableImage::selectedPoint, [this](const QPointF &p)
          {
            if (d->selection_mode == Private::SelectionMode::AddTrackingTarget)
            {
              if (d->imgTracker->addBlockMatchingTarget(p.toPoint()))
              {
                if (d->infoOverlay.centroidArea)
                {
                  d->image_widget->scene()->removeItem(d->infoOverlay.centroidArea);
                  delete d->infoOverlay.centroidArea;
                  d->infoOverlay.centroidArea = nullptr;
                }

                auto item = new QGraphicsEllipseItem(-10, -10, 20, 20);
                item->setPos(d->image_widget->getImgTransform().map(p));
                item->setPen(QPen{QColor{0, 255, 0, 255}});
                item->setBrush(QBrush{Qt::NoBrush});
                item->setZValue(1);

                d->infoOverlay.trackingTargets.push_back(item);
                d->image_widget->scene()->addItem(item);

                d->ui->actionDisableTracking->setEnabled(true);
              }
            }
          });

  connect(d->ui->actionDisableTracking, &QAction::triggered, [&]
          {
            d->imgTracker->clear();
            d->ui->actionDisableTracking->setEnabled(false);
            updateInfoOverlay();
          });

  readTemperature->start(2000);
  d->rescan_devices();
  if (DISABLE_TRACKING == 1)
  {
    delete d->ui->menuTracking;
    delete d->ui->trackingToolBar;
  }
}

void PlanetaryImagerMainWindow::showEvent(QShowEvent *event)
{
  QMainWindow::showEvent(event);
}

void PlanetaryImagerMainWindow::closeEvent(QCloseEvent *event)
{
  d->main_window_widgets->save();
  d->saveWindowGeometry();
  QMainWindow::closeEvent(event);
  emit quit();
}

ImageHandlerPtr PlanetaryImagerMainWindow::imageHandler() const
{
  return d->imageHandler;
}

Imager *PlanetaryImagerMainWindow::imager() const
{
  return d->imager;
}

void PlanetaryImagerMainWindow::Private::rescan_devices()
{
  ui->menu_device_load->clear();
  planetaryImager->scanCameras();
}

void PlanetaryImagerMainWindow::Private::onCamerasFound()
{
  ui->menu_device_load->clear();
  auto message = tr("Found %1 devices").arg(planetaryImager->cameras().size());
  for (auto device : planetaryImager->cameras())
  {
    qDebug() << message << ", Device name: " << device->name();
    statusbar_info_widget->showMessage(message, 10'000);
    QAction *action = ui->menu_device_load->addAction(device->name());
    QObject::connect(action, &QAction::triggered, bind(&PlanetaryImager::open, planetaryImager.get(), device));
  }
  for (auto device : planetaryImager->cameras())
  {
    if(device->name() == planetaryImager->configuration().camera_name() ){
      planetaryImager->open(device);
      break;
    }
  }
}

void PlanetaryImagerMainWindow::connectCamera(const CameraPtr &camera)
{
  d->planetaryImager->open(camera);
}

void PlanetaryImagerMainWindow::setImager(Imager *imager)
{
  d->onImagerInitialized(imager);
}

void PlanetaryImagerMainWindow::Private::onImagerInitialized(Imager *imager)
{
  GuLinux::Scope scope{[=]
                       { emit q->imagerChanged(); }};
  this->imager = imager;
  exposure_timer.set_imager(imager);
  imager->startLive();
  statusbar_info_widget->deviceConnected(imager->name());
  connect(imager, &Imager::disconnected, q, bind(&Private::cameraDisconnected, this), Qt::QueuedConnection);
  connect(imager, &Imager::fps, statusbar_info_widget, &StatusBarInfoWidget::captureFPS, Qt::QueuedConnection);
  connect(imager, &Imager::temperature, statusbar_info_widget, bind(&StatusBarInfoWidget::temperature, statusbar_info_widget, _1, false), Qt::QueuedConnection);

  ui->settings_container->setWidget(cameraSettingsWidget = new CameraControlsWidget(imager, planetaryImager->configuration(), filesystemBrowser));
  ui->chipInfoWidget->setWidget(cameraInfoWidget = new CameraInfoWidget(imager));
  enableUIWidgets(true);
  ui->actionSelect_ROI->setEnabled(imager->supports(Imager::ROI));
  ui->actionEdit_ROI->setEnabled(imager->supports(Imager::ROI));
  ui->actionClear_ROI->setEnabled(imager->supports(Imager::ROI));
  ui->actionSelect_ROIVirtual->setEnabled(true);
  // ui->actionEdit_ROI->setEnabled(imager->supports(Imager::ROI));
  ui->actionClear_ROIVirtual->setEnabled(true);
  ui->actionAddTrackingTarget->setEnabled(true);
  ui->actionSetCentroidArea->setEnabled(true);
}

// TODO: sync issues when images are sent after the imagerDisconnected signal
void PlanetaryImagerMainWindow::Private::cameraDisconnected()
{
  imager = nullptr;
  qDebug() << "camera disconnected";
  enableUIWidgets(false);
  ui->actionSelect_ROI->setEnabled(false);
  ui->actionEdit_ROI->setEnabled(false);
  ui->actionClear_ROI->setEnabled(false);
  ui->actionAddTrackingTarget->setEnabled(false);
  ui->actionSetCentroidArea->setEnabled(false);
  ui->actionDisableTracking->setEnabled(false);

  delete cameraSettingsWidget;
  cameraSettingsWidget = nullptr;
  delete cameraInfoWidget;
  cameraInfoWidget = nullptr;
  statusbar_info_widget->captureFPS(0);
  statusbar_info_widget->temperature(0, true);
  image_widget->setImage({});
}

void PlanetaryImagerMainWindow::Private::enableUIWidgets(bool cameraConnected)
{
  ui->actionDisconnect->setEnabled(cameraConnected);
  ui->recording->setEnabled(cameraConnected);
  ui->chipInfoWidget->setEnabled(cameraConnected);
  ui->camera_settings->setEnabled(cameraConnected);
}

void PlanetaryImagerMainWindow::notify(const QDateTime &when, MessagesLogger::Type notification_type, const QString &title, const QString &message)
{
  static QHash<MessagesLogger::Type, function<void(const QString &, const QString &)>> types_map{
      {MessagesLogger::Warning, [](const QString &title, const QString &message)
       { QMessageBox::warning(nullptr, title, message); }},
      {MessagesLogger::Error, [](const QString &title, const QString &message)
       { QMessageBox::critical(nullptr, title, message); }},
      {MessagesLogger::Info, [](const QString &title, const QString &message)
       { QMessageBox::information(nullptr, title, message); }},
  };
  types_map[notification_type](title, message);
}

void PlanetaryImagerMainWindow::Private::editROI()
{
  auto resolution = imager->properties().resolution();
  editROIDialog->setResolution(resolution);
  editROIDialog->show();
  //TODO check for resolution not existing
  // TODO check for current ROI
}
