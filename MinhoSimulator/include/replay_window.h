#ifndef REPLAYWINDOW_H
#define REPLAYWINDOW_H

#include <QWidget>
#include <QDebug>

namespace Ui {
class Window;
}

class ReplayWindow : public QWidget
{
    Q_OBJECT

public:
    explicit ReplayWindow(QString *file, QWidget *parent = 0);
    ~ReplayWindow();
private slots:

private:
    void setupUI();

private:
    Ui::Window *ui;
    QString home;
};

#endif // WIDGET_H

