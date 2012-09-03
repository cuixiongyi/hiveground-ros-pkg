#ifndef VELIB_GLOBAL_H
#define VELIB_GLOBAL_H

#include <QtCore/qglobal.h>
#include <QObject>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QStyleOptionGraphicsItem>
#include <QWheelEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>

#define VELIB_TYPE (80) //Hg mass

#ifdef VELIB_LIB
# define VELIB_EXPORT Q_DECL_EXPORT
#else
# define VELIB_EXPORT Q_DECL_IMPORT
#endif



#endif // VELIB_GLOBAL_H
