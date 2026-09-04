#ifndef ROBOTWIDGET_H
#define ROBOTWIDGET_H

#include <QWidget>
#include <QBrush>
#include <QFont>
#include <QPen>
#include "common/State.h"

class RobotWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RobotWidget(QWidget *parent = nullptr);
    void setState(State const& state);
    State const& getState() { return state; }

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    State state;
    QBrush background;
    QBrush robotBrush;
    QPen robotPen;
    QBrush mouthBrush;
    QBrush particleBrush;
    QPen particlePen;
    QFont textFont;
    QPen textPen;
};

#endif // ROBOTWIDGET_H
