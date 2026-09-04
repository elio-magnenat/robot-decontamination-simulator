#include "robotsui.h"
#include "./ui_robotsui.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QFileInfo>
#include <fstream>

#include <iostream>
#include <iomanip>

#include "common/StateUtils.h"

using namespace std;

RobotsUI::RobotsUI(QWidget* parent) : QMainWindow(parent), ui(new Ui::RobotsUI) {
    ui->setupUi(this);

    animationTimer = new QTimer(this);
    animationTimer->setInterval(INTERVAL);
    connect(animationTimer, &QTimer::timeout, this, &RobotsUI::onTimerTick);

    ui->SpeedSpinBox->setRange(1, 10);
    ui->SpeedSpinBox->setValue(1);

    ui->TimeLineSlider->setRange(0, kSliderResolution);

    setControlsEnabled(false);

    qApp->installEventFilter(this);

    setWindowTitle("PIN-2026");
}

void RobotsUI::setControlsEnabled(const bool enabled) const {
    ui->PlayButton->setEnabled(enabled);
    ui->RewindButton->setEnabled(enabled);
    ui->TimeSpinBox->setEnabled(enabled);
    ui->TimeLineSlider->setEnabled(enabled);
}

RobotsUI::~RobotsUI() {
    qApp->removeEventFilter(this);
    delete ui;
}


void RobotsUI::OpenState() {
    const QString fileName = QFileDialog::getOpenFileName(this,
                                                          tr("Open State File"), "", tr("Json Files (*.stat)"));
    if (fileName.isNull())
        return;

    ifstream ifs(fileName.toStdString());
    if (!ifs) {
        clearSimulation();
        cout << "Erreur : impossible d'ouvrir le fichier " << fileName.toStdString() << endl;
        QMessageBox::warning(this, tr("Erreur"), tr("Impossible d'ouvrir %1").arg(fileName));
        return;
    }

    State state;
    try {
        ifs >> state;
    }
    catch (exception const& e) {
        clearSimulation();
        cout << "Erreur : fichier State invalide - " << e.what() << endl;
        QMessageBox::warning(this, tr("Fichier invalide"), QString::fromStdString(e.what()));
        return;
    }

    if (playing)
        stopPlayback();

    initialState = state;
    hasState = true;
    hasTimeline = false;

    ui->TimeSpinBox->setRange(0.0, kSingleStateDuration);
    ui->TimeSpinBox->setValue(0.0);
    on_TimeSpinBox_valueChanged(ui->TimeSpinBox->value());

    setControlsEnabled(true);

    setWindowTitle(QFileInfo(fileName).fileName());
    statusBar()->showMessage(tr("%1 robot(s), %2 particule(s)")
                             .arg(state.robots.size())
                             .arg(state.particles.size()));
}

void RobotsUI::setState(State const& state) const {
    ui->widget->setState(state);
}

void RobotsUI::clearSimulation() {
    stopPlayback();

    hasState = false;
    hasTimeline = false;

    currentState = State{};
    initialState = State{};
    timeline.states.clear();

    setState(currentState);

    ui->TimeSpinBox->blockSignals(true);
    ui->TimeSpinBox->setRange(0.0, kSingleStateDuration);
    ui->TimeSpinBox->setValue(0.0);
    ui->TimeSpinBox->blockSignals(false);

    ui->TimeLineSlider->blockSignals(true);
    ui->TimeLineSlider->setValue(0);
    ui->TimeLineSlider->blockSignals(false);

    ui->ScoreLabel->setText("Score : 0 / 0");

    setControlsEnabled(false);

    setWindowTitle("PIN-2026");
    statusBar()->clearMessage();
}

int RobotsUI::timeToSliderValue(const double time) const {
    const double minT = ui->TimeSpinBox->minimum();
    const double maxT = ui->TimeSpinBox->maximum();
    if (maxT <= minT)
        return 0;
    return qRound((time - minT) / (maxT - minT) * kSliderResolution);
}

double RobotsUI::sliderValueToTime(const int value) const {
    const double minT = ui->TimeSpinBox->minimum();
    const double maxT = ui->TimeSpinBox->maximum();
    return minT + (maxT - minT) * value / static_cast<double>(kSliderResolution);
}

void RobotsUI::SaveState() {
    QString fileName = QFileDialog::getSaveFileName(
        this,
        tr("Save State File"),
        "",
        tr("Json Files (*.stat)")
    );

    if (fileName.isNull())
        return;

    ofstream ofs(fileName.toStdString());

    if (!ofs) {
        cout << "Erreur : impossible d'ecrire le fichier " << fileName.toStdString() << endl;
        QMessageBox::warning(
            this,
            tr("Erreur"),
            tr("Impossible d'ecrire %1").arg(fileName)
        );
        return;
    }
    ofs << setw(4) << ui->widget->getState();
}

bool RobotsUI::eventFilter(QObject* watched, QEvent* event) {
    if (event->type() == QEvent::KeyPress) {
        const auto* keyEvent = static_cast<QKeyEvent*>(event);
        switch (keyEvent->key()) {
        case Qt::Key_Space:
            on_PlayButton_clicked();
            return true;
        case Qt::Key_Right:
            nudgeTime(1.0);
            return true;
        case Qt::Key_Left:
            nudgeTime(-1.0);
            return true;
        default:
            break;
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

void RobotsUI::nudgeTime(const double deltaSeconds) const {
    if (!hasState && !hasTimeline)
        return;

    const double newTime = qBound(ui->TimeSpinBox->minimum(),
                                  ui->TimeSpinBox->value() + deltaSeconds,
                                  ui->TimeSpinBox->maximum());
    ui->TimeSpinBox->setValue(newTime);
}

void RobotsUI::on_actionOpen_triggered() {
    this->OpenState();
}

void RobotsUI::on_actionSave_triggered() {
    this->SaveState();
}

void RobotsUI::on_actionQuit_triggered() {
    qApp->exit(0);
}

void RobotsUI::OpenTimeline() {
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Timeline File"), "", tr("Json Files (*.tlin)"));
    if (fileName.isNull())
        return;

    ifstream ifs(fileName.toStdString());
    if (!ifs) {
        clearSimulation();
        cout << "Erreur : impossible d'ouvrir le fichier " << fileName.toStdString() << endl;
        QMessageBox::warning(this, tr("Erreur"), tr("Impossible d'ouvrir %1").arg(fileName));
        return;
    }
    TimeLine tl;
    try {
        ifs >> tl;
    }
    catch (const std::exception& e) {
        clearSimulation();
        cout << "Erreur : fichier Timeline invalide - " << e.what() << endl;
        QMessageBox::warning(this, tr("Fichier invalide"), QString::fromStdString(e.what()));
        return;
    }

    if (tl.states.empty()) {
        clearSimulation();
        cout << "Warning : la timeline ne contient aucun etat." << endl;
        QMessageBox::warning(
            this,
            tr("Fichier invalide"),
            tr("La timeline ne contient aucun état.")
        );
        return;
    }

    if (playing)
        stopPlayback();

    timeline = tl;
    hasTimeline = true;
    hasState = false;

    setWindowTitle(QFileInfo(fileName).fileName());

    ui->TimeSpinBox->setRange(timeline.states.front().time, timeline.states.back().time);
    ui->TimeSpinBox->setValue(timeline.states.front().time);
    on_TimeSpinBox_valueChanged(ui->TimeSpinBox->value());

    setControlsEnabled(true);
}

void RobotsUI::on_actionOpen_Timeline_triggered() {
    this->OpenTimeline();
}

void RobotsUI::on_PlayButton_clicked() {
    if (!hasTimeline && !hasState)
        return;

    playing = !playing;
    if (playing) {
        ui->PlayButton->setText("| |");
        playbackClock.start();
        animationTimer->start();
    }
    else {
        ui->PlayButton->setText("|>");
        animationTimer->stop();
    }
}

void RobotsUI::on_RewindButton_clicked() {
    if (playing)
        stopPlayback();

    ui->TimeSpinBox->setValue(timeOrigin());
}

void RobotsUI::on_TimeLineSlider_valueChanged(const int value) const {
    ui->TimeSpinBox->setValue(sliderValueToTime(value));
}

void RobotsUI::on_TimeSpinBox_valueChanged(const double time) {
    if (!hasTimeline && !hasState)
        return;

    if (hasState) {
        currentState = getStateAfterTime(initialState, time);
        currentState.time = time;
    }
    else {
        currentState = computeStateAtTime(time);
    }

    setState(currentState);
    updateScore(currentState);

    ui->TimeLineSlider->blockSignals(true);
    ui->TimeLineSlider->setValue(timeToSliderValue(time));
    ui->TimeLineSlider->blockSignals(false);
}

void RobotsUI::onTimerTick() {
    const double realDt = playbackClock.restart() / 1000.0;
    const double dt = realDt * ui->SpeedSpinBox->value();
    const double newTime = ui->TimeSpinBox->value() + dt;

    const double endTime = hasTimeline ? timeline.states.back().time : kSingleStateDuration;

    if (newTime >= endTime) {
        ui->TimeSpinBox->setValue(endTime);
        stopPlayback();
        return;
    }

    ui->TimeSpinBox->setValue(newTime);
}


void RobotsUI::updateScore(State const& state) const {
    const Score robotScore = getTotalRobotScore(state);
    const Score particleEnergy = getRemainingParticleEnergy(state);

    ui->ScoreLabel->setText(QString("Score : %1 / %2")
                            .arg(robotScore, 0, 'f', 0)
                            .arg(particleEnergy, 0, 'f', 0));
}

State RobotsUI::computeStateAtTime(const Time time) const {
    auto state = timeline.getDisplayStateAt(time);
    if (!state.has_value()) {
        cout << "Warning : aucun etat trouve dans la timeline pour t=" << time << endl;
        return State{};
    }
    return *state;
}

void RobotsUI::stopPlayback() {
    animationTimer->stop();
    playing = false;
    ui->PlayButton->setText("|>");
}

double RobotsUI::timeOrigin() const {
    return hasTimeline ? timeline.states.front().time : 0.0;
}
