#include <QPixmap>
#include <QImage>
#include <QList>
#include <QColor>
#include <QPainter>
#include <QRectF>
#include <QPointF>
#include <QFileDialog>
#include <QDir>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <fstream>
#include "rsltdialog.h"
#include "ui_rsltdialog.h"

rsltDialog::rsltDialog(QWidget *parent, int w, int h) :
    QDialog(parent), ui(new Ui::rsltDialog)
{
    ui->setupUi(this);
    ui->graphicsView->setScene(&scn);
}

rsltDialog::~rsltDialog()
{
    delete ui;
}

void rsltDialog::on_saveImgButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this, "save image", QDir::currentPath(),"Images(*.png)");
    /*//item->pixmap().toImage().save(filename);
    rrt.save(filename);
    filename = QFileDialog::getSaveFileName(this, "save image", QDir::currentPath(),"Images(*.png)");
    rrt_first.save(filename);
    filename = QFileDialog::getSaveFileName(this, "save image", QDir::currentPath(),"Images(*.png)");
    rrt_shorten.save(filename);
    filename = QFileDialog::getSaveFileName(this, "save image", QDir::currentPath(),"Images(*.png)");
    rrt_kill.save(filename);*/
    QList<QGraphicsItem*> its=scn.items();
    QGraphicsPixmapItem* picItem = (QGraphicsPixmapItem*)its[0];
    picItem->pixmap().toImage().save(filename);
}

/*void rsltDialog::plot(QImage &img, vector<node*> &total_nodes, vector<pair<double, double> > &obstacles )
{
    QPainter pntr;
    pntr.setPen(Qt::black);
    pntr.begin(&img);
    for(unsigned int i = 1; i < total_nodes.size(); ++i)
    {
        double x1 = total_nodes[i]->x * 100;
        double y1 = total_nodes[i]->y * 100;
        double x2 = total_nodes[i]->parent->x * 100;
        double y2 = total_nodes[i]->parent->y * 100;
        pntr.drawLine((int)x1, (int)y1, (int)x2, (int)y2);
        pntr.drawRect((int)(x1-2), (int)(y1-2), 5, 5);
        pntr.drawRect((int)(x2-2), (int)(y2-2), 5, 5);
    }
    pntr.setPen(Qt::red);
    for(unsigned int i = 0; i < obstacles.size(); ++i)
    {
        double x = obstacles[i].first*100 - 50;
        double y = obstacles[i].second*100 - 50;
        QRectF rect1(x,y,100,100);
        pntr.drawEllipse(rect1);
    }
    pntr.end();
    //QImage mir_img = img.mirrored();
    img = img.mirrored();
    //item->setPixmap(QPixmap::fromImage(mir_img));
    item->setPixmap(QPixmap::fromImage(img));
}*/

void rsltDialog::plotObs(QImage &img, vector<pair<double, double> > &obstacles)
{
    QPainter pntr;
    pntr.begin(&img);
    pntr.setPen(Qt::red);
    pntr.setBrush(Qt::NoBrush);
    std::ofstream out("obs_img.txt");
    for(unsigned int i = 0; i < obstacles.size(); ++i)
    {
        qreal x = obstacles[i].first*100.0 - 50.0;
        qreal y = obstacles[i].second*100.0 - 50.0;
        QRectF rect1(x,y,100,100);
        pntr.drawEllipse(rect1);
        out << "x, y: " << x << ", " << y << "\n";
    }
    pntr.end();
}

void rsltDialog::plotPath(QImage &img, vector<node*> &path)
{
    QPainter pntr;
    pntr.begin(&img);
    pntr.setPen(Qt::blue);
    pntr.setBrush(Qt::SolidPattern);
    std::ofstream out("path_img.txt");
    for(unsigned int i = 0; i < path.size(); ++i)
    {
        if (path[i]->parent != NULL)
        {
            qreal x1 = path[i]->x * 100.0;
            qreal y1 = path[i]->y * 100.0;
            qreal x2 = path[i]->parent->x * 100.0;
            qreal y2 = path[i]->parent->y * 100.0;
            QPointF p1(x1,y1);
            QPointF p2(x2,y2);
            /*pntr.drawLine((int)x1, (int)y1, (int)x2, (int)y2);
            pntr.drawRect((int)(x1-2), (int)(y1-2), 4, 4);
            pntr.drawRect((int)(x2-2), (int)(y2-2), 4, 4);*/
            pntr.drawLine(p1,p2);
            out << "x, y: " << x1 << ", " << y1 << "; ";
            out << x2 << ", " << y2 << "\n";
        }
        //double tx = path[i]->x * 100.0;
        //double ty = path[i]->y * 100.0;
        //out << "x, y: " << tx << ", " << ty << "\n";
    }

    pntr.end();
}
void rsltDialog::dispAll(QImage &img, vector<node*> &path, vector<pair<double, double> > &obstacles)
{
    plotPath(img, path);
    plotObs(img, obstacles);
    img = img.mirrored();
    QList<QGraphicsItem*> its=scn.items();
    for (int i=0; i<its.size(); ++i)
    {
        scn.removeItem(its[i]);\
    }
    scn.addPixmap(QPixmap::fromImage(img));
}

void rsltDialog::on_closeButton_clicked()
{
    this->close();
}
