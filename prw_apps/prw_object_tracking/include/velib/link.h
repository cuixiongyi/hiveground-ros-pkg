#ifndef VELIB_LINK_H
#define VELIB_LINK_H

#include <velib/velib_global.h>

namespace ve
{
class Node;

class VELIB_EXPORT Link : public QObject, public QGraphicsItem
{
    Q_OBJECT
    //Q_PROPERTY(QString Name READ name WRITE set_name)
    Q_INTERFACES(QGraphicsItem)

public:
    Link();
    Link(const QPointF& point_out, const QPointF& point_in);
    Link(Node* node_out, Node* node_in);
    virtual ~Link();    

    enum { Type = UserType + VELIB_TYPE + 1};
    int type() const { return Type; }
    //const QString name() const { return objectName(); }
    

    void adjust();
	void initialize();
    void prepare_for_delete();

public Q_SLOTS:
    //void set_name(const QString& name) { setObjectName(name); update(); }




protected:
    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, 
        QWidget *widget);

    void hoverEnterEvent(QGraphicsSceneHoverEvent * event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);

    void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

protected:
    friend class Node;
    friend class Scene;

private:    
    QPointF point_out_;
    QPointF point_in_;
    //bool edit_mode_;

    Node* node_out_;
    Node* node_in_;
    QPolygonF arrow_head_;


};

}
#endif
