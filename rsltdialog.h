#ifndef RSLTDIALOG_H
#define RSLTDIALOG_H

#include <QDialog>
//#include <QImage>
#include <QGraphicsScene>
//#include <QGraphicsPixmapItem>
#include "rrt.h"

namespace Ui {
class rsltDialog;
}

class rsltDialog : public QDialog
{
    Q_OBJECT

public:
    explicit rsltDialog(QWidget *parent, int w, int h);
    void plotPath(QImage &img, vector<node*> &path);
    void plotObs(QImage &img, vector<pair<double, double> > &obstacles);
    void dispAll(QImage &img, vector<node*> &path, vector<pair<double, double> > &obstacles);
    ~rsltDialog();

private slots:
    void on_saveImgButton_clicked();

    void on_closeButton_clicked();

private:
    Ui::rsltDialog *ui;
    QGraphicsScene scn;
};

#endif // RSLTDIALOG_H
