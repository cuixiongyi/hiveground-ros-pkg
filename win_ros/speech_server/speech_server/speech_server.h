#ifndef SPEECH_SERVER_H
#define SPEECH_SERVER_H

#include <QtGui/QMainWindow>
#include "ui_speech_server.h"

class speech_server : public QMainWindow
{
    Q_OBJECT

public:
    speech_server(QWidget *parent = 0, Qt::WFlags flags = 0);
    ~speech_server();

private:
    Ui::speech_serverClass ui;
};

#endif // SPEECH_SERVER_H
