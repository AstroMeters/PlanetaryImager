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

#include "recordingpanel.h"
#include "ui_recordingpanel.h"
#include "configuration.h"
#include <QFileDialog>
#include <Qt/functional.h>
using namespace std;

class RecordingPanel::Private {
public:
  Private(RecordingPanel *q);
    unique_ptr<Ui::RecordingPanel> ui;
    bool recording;
private:
  RecordingPanel *q;
};

RecordingPanel::Private::Private(RecordingPanel* q)
  : q{q}
{

}


RecordingPanel::~RecordingPanel()
{
}

RecordingPanel::RecordingPanel(Configuration& configuration, QWidget* parent) : QWidget{parent}, dptr(this)
{
  d->ui.reset(new Ui::RecordingPanel);
  d->ui->setupUi(this);
  recording(false);
  d->ui->save_info_file->setChecked(configuration.save_info_file());
  d->ui->saveDirectory->setText(configuration.save_directory());
  d->ui->filePrefix->setText(configuration.save_file_prefix());
  d->ui->fileSuffix->setText(configuration.save_file_suffix());
  d->ui->videoOutputType->setCurrentIndex(configuration.save_format() == Configuration::SER ? 0 : 1);
  connect(d->ui->videoOutputType, F_PTR(QComboBox, currentIndexChanged, int), [&](int index) {
    if(index == 0)
      configuration.set_save_format(Configuration::SER);
    if(index == 1)
      configuration.set_save_format(Configuration::Video);
  });
  connect(d->ui->saveDirectory, &QLineEdit::textChanged, [&configuration](const QString &directory){
    configuration.set_save_directory(directory);
  });
  connect(d->ui->filePrefix, &QLineEdit::textChanged, [&configuration](const QString &prefix){
    configuration.set_save_file_prefix(prefix);
  });
  connect(d->ui->fileSuffix, &QLineEdit::textChanged, [&configuration](const QString &suffix){
    configuration.set_save_file_suffix(suffix);
  });
  
  d->ui->limitType->setCurrentIndex(configuration.recording_limit_type());
  connect(d->ui->limitType, F_PTR(QComboBox, currentIndexChanged, int), d->ui->limitsWidgets, &QStackedWidget::setCurrentIndex);
  connect(d->ui->limitType, F_PTR(QComboBox, currentIndexChanged, int), [&configuration](int index){ configuration.set_recording_limit_type(static_cast<Configuration::RecordingLimit>(index)); });
  d->ui->limitsWidgets->setCurrentIndex(d->ui->limitType->currentIndex());
  
  d->ui->saveFramesLimit->setCurrentText(QString::number(configuration.recording_frames_limit()));
  connect(d->ui->saveFramesLimit, &QComboBox::currentTextChanged, [&configuration](const QString &text){
      configuration.set_recording_frames_limit(text.toLongLong());
  });
  
  d->ui->duration_limit->setValue(configuration.recording_seconds_limit());
  connect(d->ui->duration_limit, F_PTR(QDoubleSpinBox, valueChanged, double), [&configuration](double seconds){
      configuration.set_recording_seconds_limit(seconds);
  });
  
  connect(d->ui->save_info_file, &QCheckBox::toggled, [&configuration](bool checked) { configuration.set_save_info_file(checked); });
  connect(d->ui->start_stop_recording, &QPushButton::clicked, [=]{
    if(d->recording)
      emit stop();
    else
      emit start();
  });
  
  auto pickDirectory = d->ui->saveDirectory->addAction(QIcon(":/resources/folder.png"), QLineEdit::TrailingPosition);
  connect(pickDirectory, &QAction::triggered, [&]{
    QFileDialog *filedialog = new QFileDialog(this);
    filedialog->setFileMode(QFileDialog::Directory);
    filedialog->setDirectory(configuration.save_directory());
    filedialog->setOption(QFileDialog::ShowDirsOnly);
    connect(filedialog, SIGNAL(fileSelected(QString)), d->ui->saveDirectory, SLOT(setText(QString)));
    connect(filedialog, SIGNAL(finished(int)), filedialog, SLOT(deleteLater()));
    filedialog->show();
  });
  auto check_directory = [&] {
    d->ui->start_stop_recording->setEnabled(QDir(configuration.save_directory()).exists());
  };
  check_directory();
  connect(d->ui->saveDirectory, &QLineEdit::textChanged, check_directory);
}

void RecordingPanel::recording(bool recording, const QString& filename)
{
  d->recording = recording;
  saveFPS(0);
  dropped(0);
  d->ui->recordingBox->setVisible(recording);
  d->ui->filename->setText(filename);
  d->ui->start_stop_recording->setText(recording ? tr("Stop") : tr("Start"));
  for(auto widget: QList<QWidget*>{d->ui->saveDirectory, d->ui->filePrefix, d->ui->fileSuffix, d->ui->saveFramesLimit})
    widget->setEnabled(!recording);
}

void RecordingPanel::saveFPS(double fps)
{
  d->ui->fps->setText(QString::number(fps, 'f', 2));
}

void RecordingPanel::meanFPS(double fps)
{
  d->ui->mean_fps->setText(QString::number(fps, 'f', 2));
}


void RecordingPanel::dropped(int frames)
{
  d->ui->dropped->setText(QString::number(frames));
}

void RecordingPanel::saved(int frames)
{
  d->ui->frames->setText(QString::number(frames));
}



