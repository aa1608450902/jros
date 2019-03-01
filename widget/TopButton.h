#ifndef __TOPBUTTON_H__
#define __TOPBUTTON_H__

#include <QWidget>
#include <QLabel>
#include <QTableWidget>
#include <QPushButton>

class TopButton : public QWidget {
Q_OBJECT
public:
    explicit TopButton(const QString& text, QWidget *parent = nullptr);
    virtual ~TopButton() = default;
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
signals:
    void clicked();
private:
    bool folded = true;
    QLabel *label;
    QImage image;
};

#endif