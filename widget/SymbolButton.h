#ifndef __SYMBOLBUTTON_H__
#define __SYMBOLBUTTON_H__

#include <QWidget>
#include <QPushButton>
#include <QImage>

class SymbolButton : public QPushButton {
    Q_OBJECT
public:
    explicit SymbolButton(QWidget *parent = nullptr, bool flag = false);
    virtual ~SymbolButton() = default;
signals:
    void moveMini();
    void recoverMini();
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
//    void focusInEvent(QFocusEvent *);
//    void focusOutEvent(QFocusEvent *);
private:
    QImage *image;
};

#endif /* __SYMBOLBUTTON_H__ */