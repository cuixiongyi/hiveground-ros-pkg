/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 */

#include <hg_inspector_arm/inspector_arm.h>
#include <angles/angles.h>


bool InspectorArm::initializePropertyEditor()
{
  double_manager_ = new QtDoublePropertyManager(this);
  string_manager_ = new QtStringPropertyManager(this);
  color_manager_ = new QtColorPropertyManager(this);
  font_manager_ = new QtFontPropertyManager(this);
  point_manager_ = new QtPointPropertyManager(this);
  size_manager_ = new QtSizePropertyManager(this);
  group_manager_ = new QtGroupPropertyManager(this);

  connect(double_manager_, SIGNAL(valueChanged(QtProperty *, double)),
          this, SLOT(valueChanged(QtProperty *, double)));
  connect(string_manager_, SIGNAL(valueChanged(QtProperty *, const QString &)),
          this, SLOT(valueChanged(QtProperty *, const QString &)));
  connect(color_manager_, SIGNAL(valueChanged(QtProperty *, const QColor &)),
          this, SLOT(valueChanged(QtProperty *, const QColor &)));
  connect(font_manager_, SIGNAL(valueChanged(QtProperty *, const QFont &)),
          this, SLOT(valueChanged(QtProperty *, const QFont &)));
  connect(point_manager_, SIGNAL(valueChanged(QtProperty *, const QPoint &)),
          this, SLOT(valueChanged(QtProperty *, const QPoint &)));
  connect(size_manager_, SIGNAL(valueChanged(QtProperty *, const QSize &)),
          this, SLOT(valueChanged(QtProperty *, const QSize &)));

  QtDoubleSpinBoxFactory *doubleSpinBoxFactory = new QtDoubleSpinBoxFactory(this);
  QtCheckBoxFactory *checkBoxFactory = new QtCheckBoxFactory(this);
  QtSpinBoxFactory *spinBoxFactory = new QtSpinBoxFactory(this);
  QtLineEditFactory *lineEditFactory = new QtLineEditFactory(this);
  QtEnumEditorFactory *comboBoxFactory = new QtEnumEditorFactory(this);

  property_editor_ = new QtTreePropertyBrowser(ui.propertyDock);
  property_editor_->setFactoryForManager(double_manager_, doubleSpinBoxFactory);
  property_editor_->setFactoryForManager(string_manager_, lineEditFactory);
  property_editor_->setFactoryForManager(color_manager_->subIntPropertyManager(), spinBoxFactory);
  property_editor_->setFactoryForManager(font_manager_->subIntPropertyManager(), spinBoxFactory);
  property_editor_->setFactoryForManager(font_manager_->subBoolPropertyManager(), checkBoxFactory);
  property_editor_->setFactoryForManager(font_manager_->subEnumPropertyManager(), comboBoxFactory);
  property_editor_->setFactoryForManager(point_manager_->subIntPropertyManager(), spinBoxFactory);
  property_editor_->setFactoryForManager(size_manager_->subIntPropertyManager(), spinBoxFactory);
  ui.propertyDock->setWidget(property_editor_);

  current_item_ = 0;

  connect(this, SIGNAL(inspectionPointClickedSignal(InspectionPointItem *)),
          this, SLOT(inspectionPointClicked(InspectionPointItem *)));
  connect(this, SIGNAL(inspectionPointMovedSignal(InspectionPointItem *)),
          this, SLOT(inspectionPointMoved(InspectionPointItem *)));

  inspectionPointClicked(0);
  return true;
}

void InspectorArm::updateExpandState()
{
    QList<QtBrowserItem *> list = property_editor_->topLevelItems();
    QListIterator<QtBrowserItem *> it(list);
    while (it.hasNext()) {
        QtBrowserItem *item = it.next();
        QtProperty *prop = item->property();
        id_to_expanded_[property_to_id_[prop]] = property_editor_->isExpanded(item);
    }
}

void InspectorArm::inspectionPointClicked(InspectionPointItem* item)
{
  updateExpandState();

  QMap<QtProperty *, QString>::ConstIterator it_prop = property_to_id_.constBegin();
  while (it_prop != property_to_id_.constEnd())
  {
    delete it_prop.key();
    it_prop++;
  }

  property_to_id_.clear();
  id_to_property_.clear();

  current_item_ = item;
  if (!current_item_)
  {
    return;
  }

  QtProperty *property;
  property = string_manager_->addProperty(tr("Name"));
  string_manager_->setValue(property, item->name());
  addProperty(property, QLatin1String("name"));

  property = double_manager_->addProperty(tr("Position X"));
  double_manager_->setDecimals(property, 3);
  double_manager_->setSingleStep(property, 0.001);
  double_manager_->setRange(property, -5, 5);
  double_manager_->setValue(property, item->x());
  addProperty(property, QLatin1String("pose_x"));

  property = double_manager_->addProperty(tr("Position Y"));
  double_manager_->setDecimals(property, 3);
  double_manager_->setSingleStep(property, 0.001);
  double_manager_->setRange(property, -5, 5);
  double_manager_->setValue(property, item->y());
  addProperty(property, QLatin1String("pose_y"));

  property = double_manager_->addProperty(tr("Position Z"));
  double_manager_->setDecimals(property, 3);
  double_manager_->setSingleStep(property, 0.001);
  double_manager_->setRange(property, -5, 5);
  double_manager_->setValue(property, item->z());
  addProperty(property, QLatin1String("pose_z"));

  property = double_manager_->addProperty(tr("Roll"));
  double_manager_->setDecimals(property, 2);
  double_manager_->setSingleStep(property, 0.01);
  double_manager_->setRange(property, -180.0, 180.0);
  double_manager_->setValue(property, angles::to_degrees(item->roll()));
  addProperty(property, QLatin1String("pose_roll"));

  property = double_manager_->addProperty(tr("Pitch"));
  double_manager_->setDecimals(property, 2);
  double_manager_->setSingleStep(property, 0.01);
  double_manager_->setRange(property, -180.0, 180.0);
  double_manager_->setValue(property, angles::to_degrees(item->pitch()));
  addProperty(property, QLatin1String("pose_pitch"));

  property = double_manager_->addProperty(tr("Yaw"));
  double_manager_->setDecimals(property, 2);
  double_manager_->setSingleStep(property, 0.01);
  double_manager_->setRange(property, -180.0, 180.0);
  double_manager_->setValue(property, angles::to_degrees(item->yaw()));
  addProperty(property, QLatin1String("pose_yaw"));
}

void InspectorArm::addProperty(QtProperty *property, const QString &id)
{
  property_to_id_[property] = id;
  id_to_property_[id] = property;
  QtBrowserItem *item = property_editor_->addProperty(property);
  if (id_to_expanded_.contains(id))
    property_editor_->setExpanded(item, id_to_expanded_[id]);
}

void InspectorArm::inspectionPointMoved(InspectionPointItem* item)
{
  if(item != current_item_)
  {
    ROS_INFO("wrong item");
    return;
  }

  double_manager_->setValue(id_to_property_[QLatin1String("pose_x")], item->x());
  double_manager_->setValue(id_to_property_[QLatin1String("pose_y")], item->y());
  double_manager_->setValue(id_to_property_[QLatin1String("pose_z")], item->z());
  double_manager_->setValue(id_to_property_[QLatin1String("pose_roll")], angles::to_degrees(item->roll()));
  double_manager_->setValue(id_to_property_[QLatin1String("pose_pitch")], angles::to_degrees(item->pitch()));
  double_manager_->setValue(id_to_property_[QLatin1String("pose_yaw")], angles::to_degrees(item->yaw()));


}

void InspectorArm::valueChanged(QtProperty *property, double value)
{
  if (!property_to_id_.contains(property))
    return;

  if (!current_item_)
    return;

  QString id = property_to_id_[property];
  if (id == QLatin1String("pose_x"))
  {
    current_item_->setX(value);
  }
  else if (id == QLatin1String("pose_y"))
  {
    current_item_->setY(value);
  }
  else if (id == QLatin1String("pose_z"))
  {
    current_item_->setZ(value);
  }
  else if (id == QLatin1String("pose_roll"))
  {
    current_item_->setRoll(angles::from_degrees(value));
  }
  else if (id == QLatin1String("pose_pitch"))
  {
    current_item_->setPitch(angles::from_degrees(value));
  }
  else if (id == QLatin1String("pose_yaw"))
  {
    current_item_->setYaw(angles::from_degrees(value));
  }
}

void InspectorArm::valueChanged(QtProperty *property, const QString &value)
{

}

void InspectorArm::valueChanged(QtProperty *property, const QColor &value)
{

}

void InspectorArm::valueChanged(QtProperty *property, const QFont &value)
{

}

void InspectorArm::valueChanged(QtProperty *property, const QPoint &value)
{

}

void InspectorArm::valueChanged(QtProperty *property, const QSize &value)
{

}


