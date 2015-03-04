#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include <QList>
#include <QGraphicsScene>
#include <QAction>
#include <QContextMenuEvent>
#include "rrt.h"
//#include "rsltdialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

    void plotTree(QImage &img, vector<node*> &path);

    void plotObs(QImage &img);

    void plotAll(QImage& img, vector<node*> &path);

    //void resetData();
    
protected:
    void contextMenuEvent(QContextMenuEvent *event);

private slots:
    void on_runButton_clicked();

    void on_addObsButton_clicked();

    void on_rmObsButton_clicked();

    void on_clearButton_clicked();

    void on_viewButton_clicked();

    void on_saveImg_clicked();

    void resetData();

private:
    Ui::MainWindow *ui;
    vector<node*> total_nodes;
    vector<node*> true_path;
    vector<pair<double, double> > obstacles;
    QList<QImage> imgList;
    QGraphicsScene scn;
    QAction resetMap;
};

#endif // MAINWINDOW_H
