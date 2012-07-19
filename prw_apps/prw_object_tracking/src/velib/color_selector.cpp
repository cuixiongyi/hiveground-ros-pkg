#include <velib/color_selector.h>
#include <QDebug>

using namespace ve;

static int pWidth = 360;
static int pHeight = 200;

QPoint ColorPicker::colPt()
{
  QRect r = contentsRect();
  return QPoint((360 - hue) * (r.width() - 1) / 360, (255 - sat) * (r.height() - 1) / 255);
}

int ColorPicker::huePt(const QPoint &pt)
{
  QRect r = contentsRect();
  return 360 - pt.x() * 360 / (r.width() - 1);
}

int ColorPicker::satPt(const QPoint &pt)
{
  QRect r = contentsRect();
  return 255 - pt.y() * 255 / (r.height() - 1);
}

void ColorPicker::setCol(const QPoint &pt)
{
  setCol(huePt(pt), satPt(pt));
}

int ColorPicker::ptHue(int h)
{
  QRect r = contentsRect();
  return r.width() - ((h / 360.0) * r.width());
}

int ColorPicker::ptSat(int s)
{
  QRect r = contentsRect();
  return r.height() - ((s / 255.0) * r.height());
}

ColorPicker::ColorPicker(QWidget* parent) :
    QFrame(parent)
{
  hue = 0;
  sat = 0;
  setCol(150, 255);

  setAttribute(Qt::WA_NoSystemBackground);
  setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
}

ColorPicker::~ColorPicker()
{
}

void ColorPicker::addColorRange(const QString& name, int h_max, int h_min, int s_max, int s_min)
{
  ColorRange range;
  range.max_ = QColor::fromHsv(h_max, s_max, 255);
  range.min_ = QColor::fromHsv(h_min, s_min, 0);
  color_range_map_[name] = range;
  update();
}

void ColorPicker::updateColorRange(const QString& name, int h, int s)
{
  if (color_range_map_.find(name) != color_range_map_.end())
  {
    ColorRange range = color_range_map_[name];
    int h_max = range.max_.hue();
    int s_max = range.max_.saturation();
    int h_min = range.min_.hue();
    int s_min = range.min_.saturation();

    int dh = qMin(abs(h - h_max), abs(h - h_min));
    int ds = qMin(abs(s - s_max), abs(s - s_min));

    if(dh < ds)
    {
      if (abs(h - h_max) < abs(h - h_min))
      {
        h_max = h;
      }
      else
      {
        h_min = h;
      }
    }
    else
    {
      if (abs(s - s_max) < abs(s - s_min))
      {
        s_max = s;
      }
      else
      {
        s_min = s;
      }
    }
    color_range_map_[name].max_ = QColor::fromHsv(h_max, s_max, 255);
    color_range_map_[name].min_ = QColor::fromHsv(h_min, s_min, 0);
  }
  update();
}

void ColorPicker::updateColorRange(const QString& name, int h_max, int h_min, int s_max, int s_min, int v_max,
                                   int v_min)
{
  if (color_range_map_.find(name) == color_range_map_.end())
    return;
  ColorRange range;
  range.max_ = QColor::fromHsv(h_max, s_max, v_max);
  range.min_ = QColor::fromHsv(h_min, s_min, v_min);
  color_range_map_[name] = range;
  update();
}

void ColorPicker::removeColorRange(const QString& name)
{
  color_range_map_.remove(name);
  update();
}

bool ColorPicker::setCurrentColor(const QString& name)
{
  if (color_range_map_.find(name) == color_range_map_.end())
  {
    current_color_ = "";
    update();
    return false;
  }
  current_color_ = name;
  update();
  return true;
}

bool ColorPicker::getColorRange(const QString& name, ColorRange& color_range)
{
  if (color_range_map_.find(name) == color_range_map_.end())
    return false;
  color_range = color_range_map_[name];
  return true;
}

QSize ColorPicker::sizeHint() const
{
  return QSize(pWidth + 2 * frameWidth(), pHeight + 2 * frameWidth());
}

void ColorPicker::setCol(int h, int s)
{
  int nhue = qMin(qMax(0, h), 359);
  int nsat = qMin(qMax(0, s), 255);
  if (nhue == hue && nsat == sat)
    return;

  QRect r(colPt(), QSize(20, 20));
  hue = nhue;
  sat = nsat;
  r = r.united(QRect(colPt(), QSize(20, 20)));
  r.translate(contentsRect().x() - 9, contentsRect().y() - 9);
  //    update(r);
  repaint(r);
}

void ColorPicker::mouseMoveEvent(QMouseEvent *m)
{
  QPoint p = m->pos() - contentsRect().topLeft();
  setCol(p);
  if (!current_color_.isNull())
    updateColorRange(current_color_, hue, sat);
  emit newCol(hue, sat);
}

void ColorPicker::mousePressEvent(QMouseEvent *m)
{
  QPoint p = m->pos() - contentsRect().topLeft();
  setCol(p);
  if (!current_color_.isNull())
    updateColorRange(current_color_, hue, sat);
  emit newCol(hue, sat);
}

void ColorPicker::paintEvent(QPaintEvent*)
{
  QPainter p(this);
  drawFrame(&p);
  QRect r = contentsRect();

  p.drawPixmap(r.topLeft(), pix);
  QPoint pt = colPt() + r.topLeft();



  QMap<QString, ColorRange>::iterator it;
  for (it = color_range_map_.begin(); it != color_range_map_.end(); it++)
  {

    if(it.key() == current_color_)
    {
      p.setPen(Qt::white);
    }
    else
    {
      p.setPen(Qt::black);
    }


    int point_h_max = ptHue(it->max_.hue());
    int point_h_min = ptHue(it->min_.hue());
    int point_s_max = ptSat(it->max_.saturation());
    int point_s_min = ptSat(it->min_.saturation());

    if (point_h_max > point_h_min)
    {
      p.drawRect(QRect(-1, point_s_max, point_h_min, point_s_min - point_s_max));
      p.drawRect(QRect(point_h_max, point_s_max, r.width() - point_h_max, point_s_min - point_s_max));

    }
    else
    {
      p.drawRect(QRect(point_h_max, point_s_max, point_h_min - point_h_max, point_s_min - point_s_max));
    }

  }

  p.fillRect(pt.x() - 9, pt.y(), 20, 2, Qt::black);
  p.fillRect(pt.x(), pt.y() - 9, 2, 20, Qt::black);

}

void ColorPicker::resizeEvent(QResizeEvent *ev)
{
  QFrame::resizeEvent(ev);

  int w = width() - frameWidth() * 2;
  int h = height() - frameWidth() * 2;
  QImage img(w, h, QImage::Format_RGB32);
  int x, y;
  uint *pixel = (uint *)img.scanLine(0);
  for (y = 0; y < h; y++)
  {
    const uint *end = pixel + w;
    x = 0;
    while (pixel < end)
    {
      QPoint p(x, y);
      QColor c;
      c.setHsv(huePt(p), satPt(p), 200);
      *pixel = c.rgb();
      ++pixel;
      ++x;
    }
  }
  pix = QPixmap::fromImage(img);
}

int ColorLuminancePicker::y2val(int y)
{
  int d = height() - 2 * coff - 1;
  return 255 - (y - coff) * 255 / d;
}

int ColorLuminancePicker::val2y(int v)
{
  int d = height() - 2 * coff - 1;
  return coff + (255 - v) * d / 255;
}

ColorLuminancePicker::ColorLuminancePicker(QWidget* parent) :
    QWidget(parent)
{
  hue = 100;
  val = 100;
  sat = 100;
  pix = 0;
  //    setAttribute(WA_NoErase, true);
}

ColorLuminancePicker::~ColorLuminancePicker()
{
  delete pix;
}

void ColorLuminancePicker::mouseMoveEvent(QMouseEvent *m)
{
  setVal(y2val(m->y()));
}
void ColorLuminancePicker::mousePressEvent(QMouseEvent *m)
{
  setVal(y2val(m->y()));
}

void ColorLuminancePicker::setVal(int v)
{
  if (val == v)
    return;
  val = qMax(0, qMin(v, 255));
  delete pix;
  pix = 0;
  repaint();
  emit newHsv(hue, sat, val);
}

//receives from a hue,sat chooser and relays.
void ColorLuminancePicker::setCol(int h, int s)
{
  setCol(h, s, val);
}

void ColorLuminancePicker::paintEvent(QPaintEvent *)
{
  int w = width() - 5;

  QRect r(0, foff, w, height() - 2 * foff);
  int wi = r.width() - 2;
  int hi = r.height() - 2;
  if (!pix || pix->height() != hi || pix->width() != wi)
  {
    delete pix;
    QImage img(wi, hi, QImage::Format_RGB32);
    int y;
    uint *pixel = (uint *)img.scanLine(0);
    for (y = 0; y < hi; y++)
    {
      const uint *end = pixel + wi;
      while (pixel < end)
      {
        QColor c;
        c.setHsv(hue, sat, y2val(y + coff));
        *pixel = c.rgb();
        ++pixel;
      }
    }
    pix = new QPixmap(QPixmap::fromImage(img));
  }
  QPainter p(this);
  p.drawPixmap(1, coff, *pix);
  const QPalette &g = palette();
  qDrawShadePanel(&p, r, g, true);
  p.setPen(g.foreground().color());
  p.setBrush(g.foreground());
  QPolygon a;
  int y = val2y(val);
  a.setPoints(3, w, y, w + 5, y + 5, w + 5, y - 5);
  p.eraseRect(w, 0, 5, height());
  p.drawPolygon(a);
}

void ColorLuminancePicker::setCol(int h, int s, int v)
{
  val = v;
  hue = h;
  sat = s;
  delete pix;
  pix = 0;
  repaint();
  emit newHsv(h, s, val);
}

