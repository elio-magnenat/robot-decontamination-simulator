#ifndef ROBOTSUI_H
#define ROBOTSUI_H

#include <QMainWindow>
#include <QTimer>
#include <QElapsedTimer>
#include <QKeyEvent>
#include "common/State.h"
#include "common/TimeLine.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class RobotsUI;
}

QT_END_NAMESPACE

class RobotsUI : public QMainWindow {
    Q_OBJECT

public:
    explicit RobotsUI(QWidget* parent = nullptr);
    ~RobotsUI() override;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    static void on_actionQuit_triggered();
    void on_actionOpen_Timeline_triggered();

    void on_PlayButton_clicked();
    void on_RewindButton_clicked();

    void on_TimeLineSlider_valueChanged(int value) const;
    void on_TimeSpinBox_valueChanged(double time);

    void onTimerTick();

private:
    void OpenState();
    void SaveState();
    void OpenTimeline();
    void clearSimulation();
    void setControlsEnabled(bool enabled) const;
    void nudgeTime(double deltaSeconds) const;

    int timeToSliderValue(double time) const;
    double sliderValueToTime(int value) const;

    void setState(State const& state) const;
    void updateScore(State const& state) const;
    State computeStateAtTime(Time time) const;

    void stopPlayback();
    double timeOrigin() const;

    Ui::RobotsUI* ui;

    State currentState;
    State initialState;
    bool hasState = false;

    TimeLine timeline;
    bool hasTimeline = false;

    bool playing = false;
    QTimer* animationTimer;
    QElapsedTimer playbackClock;
};
#endif // ROBOTSUI_H