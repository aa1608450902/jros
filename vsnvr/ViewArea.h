#ifndef __VIEWAREA_H__
#define __VIEWAREA_H__

#include "header.h"

class ViewArea : public QWidget {
    Q_OBJECT
public:
    explicit ViewArea(QWidget *parent = nullptr);
    virtual ~ViewArea();
    QList<QFrame *>& getPlayWnds() {return _playWnds;}
    void paintEvent(QPaintEvent *event);
    void keyPressEvent(QKeyEvent *event);
signals:
    void exitFullScreen();
public slots:
    void singleWnd();
    void multiWnd();
    void fullScreen();
    void exitFullScreenP();
private:
    int _wndCount = 1;
    int _beginFullScreenWidth;
    int _beginFullScreenHeight;
//    QFrame *_playWnd;
    int _selectedIndex = 0;
    QList<QFrame *> _playWnds;
};

#endif /* __VIEWAREA_H__ */