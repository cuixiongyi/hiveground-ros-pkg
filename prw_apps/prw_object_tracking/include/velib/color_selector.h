#include <qframe.h>
#include <qevent.h>
#include <qpainter.h>

namespace ve
{

static inline void rgb2hsv(QRgb rgb, int &h, int &s, int &v)
{
    QColor c;
    c.setRgb(rgb);
    c.getHsv(&h, &s, &v);
}


struct ColorRange
{
  QColor max_;
  QColor min_;
};

class ColorPicker : public QFrame
{
  Q_OBJECT
public:
  ColorPicker(QWidget* parent);
  ~ColorPicker();

  void addColorRange(const QString& name, int h_max, int h_min, int s_max, int s_min);
  void updateColorRange(const QString& name, int h, int s);
  void updateColorRange(const QString& name, int h_max, int h_min, int s_max, int s_min, int v_max, int v_min);
  void removeColorRange(const QString& name);
  bool setCurrentColor(const QString& name);
  bool getColorRange(const QString& name, ColorRange& color_range);
  QString getCurrentColorName() { return current_color_; }

public slots:
  void setCol(int h, int s);

signals:
  void newCol(int h, int s);

protected:
  QSize sizeHint() const;
  void paintEvent(QPaintEvent*);
  void mouseMoveEvent(QMouseEvent *);
  void mousePressEvent(QMouseEvent *);
  void resizeEvent(QResizeEvent *);

private:
  int hue;int sat;

  QPoint colPt();
  int huePt(const QPoint &pt);
  int satPt(const QPoint &pt);
  void setCol(const QPoint &pt);
  int ptHue(int h);
  int ptSat(int h);

  QPixmap pix;

  QMap<QString, ColorRange> color_range_map_;
  QString current_color_;
};

class ColorLuminancePicker : public QWidget
{
    Q_OBJECT
public:
    ColorLuminancePicker(QWidget* parent=0);
    ~ColorLuminancePicker();

public slots:
    void setCol(int h, int s, int v);
    void setCol(int h, int s);
    void setRange(int v_min, int v_max);

signals:
    void newHsv(int h, int s, int v);
    void newRange(int v_min, int v_max);

protected:
    void paintEvent(QPaintEvent*);
    void mouseMoveEvent(QMouseEvent *);
    void mousePressEvent(QMouseEvent *);

    void handleMouse(QMouseEvent *);

private:
    enum { foff = 3, coff = 4 }; //frame and contents offset
    int val, val_min, val_max;
    int hue;
    int sat;

    int y2val(int y);
    int val2y(int val);
    void setVal(int v);

    QPixmap *pix;
};

}
