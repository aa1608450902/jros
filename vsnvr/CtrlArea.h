#ifndef __CTRLAREA_H__
#define __CTRLAREA_H__

#include "header.h"

class CtrlArea : public QWidget {
    Q_OBJECT
public:
    explicit CtrlArea(QWidget *parent = nullptr);
    virtual ~CtrlArea();
    QPushButton* getFoldButton();
private:
    QPushButton *_fold;
};

#endif /* __CTRLAREA_H__ */