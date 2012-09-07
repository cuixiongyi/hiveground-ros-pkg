/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Imai Laboratory, Keio University.
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

#ifndef PRW_H
#define PRW_H

#include <prw/prw_utils.h>

#include <boost/thread.hpp>


#include <qevent.h>
#include <qdialog.h>

#include "ui_prw.h"

class ParameterDialog: public QDialog
{
public:
  hg::WorkspaceEditorParameters* parameters_;
  ParameterDialog(hg::WorkspaceEditorParameters* parameters, QWidget* parent = NULL) :
    QDialog(parent)
  {
    parameters_ = parameters;
    setMinimumWidth(640);
    setup();
  }

  void setup();
};

class PRW : public QMainWindow, public hg::WorkspaceEditor
{
  Q_OBJECT
public:

  PRW(const hg::WorkspaceEditorParameters& parameters, QWidget *parent = 0, Qt::WFlags flags = 0);
  ~PRW();
  void initialize();

public slots:
  //Qt
  void on_bt_go_clicked();
  void on_bt_reset_clicked();
  void on_cb_enable_teleop_clicked();



  void on_new_scene_clicked();
  void on_new_mpr_clicked();
  void on_plan_clicked();

  //prw
  void endEffectorSlideUpdate();
  void endEffectorValueUpdate();

protected:
  void closeEvent(QCloseEvent *event);

public:
  Ui::PRW ui;


  bool quit_threads_;
};

#endif
