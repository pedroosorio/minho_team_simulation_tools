#ifndef FIELDBUILDERDIALOG_H
#define FIELDBUILDERDIALOG_H

#include <QDialog>
#include <QDebug>
#include <QCloseEvent>
#include <QFileDialog>

namespace Ui {
class FieldBuilderDialog;
}

class FieldBuilderDialog : public QDialog
{
    Q_OBJECT

public:
    explicit FieldBuilderDialog(QWidget *parent = 0);
    ~FieldBuilderDialog();
    bool checkConfigValidity();
    void saveLOD(bool is_save, QString path);
private slots:
    void accept();
    void reject();
    void closeEvent(QCloseEvent *event);

    void on_bt_save_clicked();

    void on_bt_default_clicked();

private:
    Ui::FieldBuilderDialog *ui;
};

#endif // FIELDBUILDERDIALOG_H
