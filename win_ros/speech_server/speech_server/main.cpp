#include "speech_server.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    speech_server w;
    w.show();
    return a.exec();
}
