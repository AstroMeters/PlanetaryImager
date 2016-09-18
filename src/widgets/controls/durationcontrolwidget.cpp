/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
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
#include "durationcontrolwidget.h"
#include <QDoubleSpinBox>
#include <QComboBox>
using namespace std;
using namespace std::chrono_literals;

struct DurationControlWidget::Private {
  QDoubleSpinBox *edit;
  QComboBox *unit_combo;
  DurationControlWidget *q;
  void valueChanged();
  void updateWidgets();
  typedef chrono::duration<double> seconds;
  seconds min, max, step, device_unit, value;
  int decimals;
};

DurationControlWidget::DurationControlWidget(QWidget *parent) : ControlWidget(parent), dptr(new QDoubleSpinBox, new QComboBox, this)
{
  layout()->addWidget(d->edit);
  layout()->addWidget(d->unit_combo);
  d->unit_combo->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::QSizePolicy::Fixed);
  d->edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  connect(d->edit, F_PTR(QDoubleSpinBox, valueChanged, double), this, bind(&Private::valueChanged, d.get() ));
  connect(d->unit_combo, F_PTR(QComboBox, activated, int), this, bind(&Private::updateWidgets, d.get() ));
  d->unit_combo->addItem("s", 1.);
  d->unit_combo->addItem("ms", 1./1000.);
  d->unit_combo->addItem("us", 1./1000./1000.);
}

DurationControlWidget::~DurationControlWidget()
{
}

void DurationControlWidget::update(const Imager::Control &control)
{
  d->decimals = control.decimals;
  d->device_unit = control.duration_unit;
  
  d->min = control.min * d->device_unit;
  d->max = control.max * d->device_unit;
  d->step = (control.step != 0 ? control.step : 0.1) * d->device_unit; // TODO: move this
  d->value = control.value * d->device_unit;
   
  int unit_index = 0;
  double display_value = d->value.count();
  
  while(display_value < 1. && ++unit_index < d->unit_combo->count() ) {
    display_value = d->value.count() / d->unit_combo->itemData(unit_index).toDouble();
  }
  d->unit_combo->setCurrentIndex(min(unit_index, d->unit_combo->count()-1));
  d->updateWidgets();
}

void DurationControlWidget::Private::valueChanged()
{
  q->valueChanged( q->value() );
}

double DurationControlWidget::value() const
{
  double unit = d->unit_combo->currentData().toDouble();
  return (d->edit->value() *unit ) / d->device_unit.count();
}


void DurationControlWidget::Private::updateWidgets()
{
  double unit = unit_combo->currentData().toDouble();
  // currently it's best to ignore reported values for decimals and single step.
  edit->setDecimals(2);
  edit->setMinimum(min.count()/unit);
  edit->setMaximum(max.count()/unit);
  edit->setSingleStep(std::max(step.count()/unit, 0.1));
  edit->setValue(value.count()/unit);
  qDebug() << "selected: " << unit_combo->currentText() << ", decimals: " << decimals << ", unit: " << unit << ", min: " << min.count() << ", max: " << max.count() << ", step: " << step.count() << ", value: " << value.count();
  qDebug() << "corrected_values: min: " << min.count()/unit << ", max: " << max.count()/unit << ", step: " << step.count()/unit << ", value: " << value.count()/unit;
}
