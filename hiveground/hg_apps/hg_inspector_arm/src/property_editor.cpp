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

bool InspectorArm::initializePropertyEditor()
{
  double_manager_ = new QtDoublePropertyManager(this);
  string_manager_ = new QtStringPropertyManager(this);
  color_manager_ = new QtColorPropertyManager(this);
  font_manager_ = new QtFontPropertyManager(this);
  point_manager_ = new QtPointPropertyManager(this);
  size_manager_ = new QtSizePropertyManager(this);


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
  return true;
}




