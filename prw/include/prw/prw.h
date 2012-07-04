#ifndef PRW_H
#define PRW_H

#include <QtGui/QMainWindow>
#include "ui_prw.h"


class PRW : public QMainWindow
{
	Q_OBJECT

public:
	PRW(QWidget *parent = 0, Qt::WFlags flags = 0);    
	~PRW();

private:
  Ui::PRW ui;

};


#endif
