#include "fieldbuilderdialog.h"
#include "ui_fieldbuilderdialog.h"

FieldBuilderDialog::FieldBuilderDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FieldBuilderDialog)
{
    ui->setupUi(this);
    ui->lb_image->setPixmap(QPixmap("://resources/images/field_dims.png"));

}

FieldBuilderDialog::~FieldBuilderDialog()
{
    delete ui;
}

bool FieldBuilderDialog::checkConfigValidity()
{
    if(ui->A->value()>ui->B->value()){
        if(ui->B->value()>ui->C->value()){
            if(ui->C->value()>ui->D->value()){
                if((ui->A->value()/3.0)>ui->E->value()){
                    if(ui->E->value()>ui->F->value()){
                        if(ui->I->value()>ui->E->value()){
                            if((ui->A->value()/3.0)>ui->H->value()){
                                if((ui->H->value()/3.0)>ui->G->value()){
                                    if(0.22>ui->K->value() && 0.22>ui->J->value()){
                                        return true;
                                    } else return false;
                                } else return false;
                            } else return false;
                        } else return false;
                    } else return false;
                } else return false;
            } else return false;
        } else return false;
    } else return false;
}

void FieldBuilderDialog::accept()
{
    // Verify dimensions
    on_bt_save_clicked();
    this->close();
}

void FieldBuilderDialog::reject()
{
    this->close();
}

void FieldBuilderDialog::closeEvent(QCloseEvent *event)
{
    event->accept();
}

void FieldBuilderDialog::on_bt_save_clicked()
{
    // Verify dimensions
    if(checkConfigValidity()){
        // Write to file
        QUrl file = QFileDialog::getSaveFileUrl(this,tr("Save Field SDF File"), QUrl(getenv("HOME")), tr("SDF Files (*.sdf)"));
        if(!file.isEmpty()){

        }
    }
}

void FieldBuilderDialog::on_bt_default_clicked()
{
    // load default official sized field dimensions
    QUrl file = QFileDialog::getOpenFileUrl(this,tr("Load Field XML File"), QUrl(getenv("HOME")), tr("XML Files (*.xml)"));
    if(!file.isEmpty()){

    }
}
