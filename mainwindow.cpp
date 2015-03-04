#include "mainwindow.h"
#include <QString>
#include <QTableWidgetSelectionRange>
#include <QFileDialog>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QMenu>
#include "ui_mainwindow.h"
#include "rrt.h"
#include <fstream>
using std::ofstream;
using std::endl;
using std::make_pair;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::MainWindow), resetMap(QString("Clean Map"), this)
{
    ui->setupUi(this);
    srand(time(NULL));
    this->setWindowTitle(QString("RRT"));
    ui->graphicsView->setScene(&scn);
    int w=this->width();
    int h=this->height();
    this->setFixedSize(w,h);
    connect(&resetMap, SIGNAL(triggered()), this, SLOT(resetData()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_runButton_clicked()
{
        resetData();
        QImage tmpImg(800, 800, QImage::Format_RGB32);
        node* start = new node;
        start->x = ui->startXEdit->text().toDouble();
        start->y = ui->startYEdit->text().toDouble();
        start->parent = NULL;
        node* goal = new node;
        goal->x = ui->goalXEdit->text().toDouble();
        goal->y = ui->goalYEdit->text().toDouble();
        QTableWidgetItem* item1;
        QTableWidgetItem* item2;
        for (int i=0;i<=ui->tableWidget->rowCount();++i)
        {
            item1= ui->tableWidget->item(i,0);
            item2= ui->tableWidget->item(i,1);
            if (item1 != 0 && item2 != 0)
            {
                double x = item1->text().toDouble();
                double y = item2->text().toDouble();
                obstacles.push_back(make_pair(x, y));
            }
        }

        ofstream obsTxt("obstacles.txt");
        for (unsigned int i=0; i<obstacles.size(); ++i)
        {
            obsTxt << obstacles[i].first << ", " << obstacles[i].second << endl;
        }
        node* node_ptr;
        node_ptr = start;
        total_nodes.push_back(node_ptr);
        while(1)
        {
            if(t_connect(total_nodes, goal, obstacles))
                break;	//reach the goal, break the loop*/
            double x = rand() % 800 ;
            double y = rand() % 800 ;
            x /= 100; y /= 100;
            node_ptr = new node;
            node_ptr -> x = x; node_ptr -> y = y;
            //connect
            if(!t_connect(total_nodes, node_ptr, obstacles))
                delete node_ptr;
        }
        tmpImg.fill(Qt::white);
        plotAll(tmpImg, total_nodes);
        imgList.append(tmpImg);
        ofstream waypnts("nodes.txt");
        waypnts << "tree:" << endl;
        for(unsigned int i = 0; i < total_nodes.size(); ++i)
        {
            waypnts << total_nodes[i] << ", ";
            waypnts << "x: " << total_nodes[i]->x << ", y: " << total_nodes[i]->y;// << endl;
            waypnts << ", parent addr: " << total_nodes[i]->parent << endl;
        }
        waypnts << "correct path:" << endl;
        node* tmp = goal;
        while(tmp->x != start->x || tmp->y != start->y)
        {
            waypnts << tmp << ", ";
            waypnts << "x: " << tmp->x << ", y: " << tmp->y;// << endl;
            waypnts << ", parent addr: " << tmp->parent << endl;
            tmp = tmp->parent;
        }
        waypnts << tmp << ", " << "x: " << tmp->x << ", y: " << tmp->y << endl;
        true_path = tracePath(start, goal);
        tmpImg.fill(Qt::white);
        plotAll(tmpImg, true_path);
        imgList.append(tmpImg);
        if(true_path.size()>2)
        {
            int cnt = 550;
            for(int i = 0; i < cnt; ++i)
            {
                pathShorten(true_path, total_nodes, obstacles); //add two nodes as shortcut if they exist
                true_path = tracePath(start, goal);
            }
        }
        tmpImg.fill(Qt::white);
        plotAll(tmpImg, true_path);
        imgList.append(tmpImg);
        waypnts << "shorten path:" << endl;
        tmp = goal;
        while(tmp->x != start->x || tmp->y != start->y)
        {
            waypnts << tmp << ", ";
            waypnts << "x: " << tmp->x << ", y: " << tmp->y;// << endl;
            waypnts << ", parent addr: " << tmp->parent << endl;
            tmp = tmp->parent;
        }
        waypnts << tmp << ", " << "x: " << tmp->x << ", y: " << tmp->y << endl;
        true_path =  killRedund(true_path,0.5);
        tmpImg.fill(Qt::white);
        plotAll(tmpImg, true_path);
        imgList.append(tmpImg);
        QImage final_img(800, 800, QImage::Format_RGB32);
        final_img.fill(Qt::white);
        plotAll(final_img, true_path);
        scn.addPixmap(QPixmap::fromImage(final_img));
        waypnts << "final path:" << endl;
        for(unsigned int i= 0; i < true_path.size(); ++i)
        {
            waypnts << true_path[i];
            waypnts << ", x: " << true_path[i]->x << ", y: " << true_path[i]->y;
            waypnts << ", parent addr: " << true_path[i]->parent << endl;
        }
}

void MainWindow::on_addObsButton_clicked()
{
    int r=ui->tableWidget->rowCount();
    QTableWidgetItem *item = new QTableWidgetItem(QTableWidgetItem::Type);
    QTableWidgetItem *item2 = new QTableWidgetItem(QTableWidgetItem::Type);
    QTableWidgetItem *vitem = new QTableWidgetItem(QTableWidgetItem::Type);
    ui->tableWidget->setRowCount(r+1);
    ui->tableWidget->setVerticalHeaderItem(r+1, vitem);
    ui->tableWidget->setItem(r+1, 0, item);
    ui->tableWidget->setItem(r+1, 1, item2);
}

void MainWindow::on_rmObsButton_clicked()
{
    QList<QTableWidgetSelectionRange> selected=ui->tableWidget->selectedRanges();
    for(int i=0; i < selected.size(); ++i)
    {
        for(int c=selected[i].bottomRow();c>=selected[i].topRow();--c)
        {
            ui->tableWidget->removeRow(c);
        }
    }
}

void MainWindow::on_clearButton_clicked()
{
    for (int i=ui->tableWidget->rowCount(); i>=0; --i)
    {
        ui->tableWidget->removeRow(i);
    }
}

void MainWindow::on_viewButton_clicked()
{
    resetData();
    node* start = new node;
    start->x = ui->startXEdit->text().toDouble();
    start->y = ui->startYEdit->text().toDouble();
    start->parent = NULL;
    node* goal = new node;
    goal->x = ui->goalXEdit->text().toDouble();
    goal->y = ui->goalYEdit->text().toDouble();
    goal->parent = start;
    total_nodes.push_back(start);
    total_nodes.push_back(goal);
    QTableWidgetItem* item1;
    QTableWidgetItem* item2;
    for (int i=0;i<=ui->tableWidget->rowCount();++i)
    {
        item1= ui->tableWidget->item(i,0);
        item2= ui->tableWidget->item(i,1);
        if (item1 != 0 && item2 != 0)
        {
            double x = item1->text().toDouble();
            double y = item2->text().toDouble();
            obstacles.push_back(make_pair(x, y));
        }
    }
    ofstream obsTxt("obstacles.txt");
    for (unsigned int i=0; i<obstacles.size(); ++i)
    {
        obsTxt << obstacles[i].first << ", " << obstacles[i].second << endl;
    }
    QImage img(800, 800, QImage::Format_RGB32);
    img.fill(Qt::white);
    plotAll(img, total_nodes);
    scn.addPixmap(QPixmap::fromImage(img));
}

void MainWindow::resetData()
{
    clearAll(total_nodes);
    obstacles.clear();
    total_nodes.clear();
    true_path.clear();
    imgList.clear();
    QList<QGraphicsItem*> its=scn.items();
    for (int i=0; i<its.size(); ++i)
    {
        scn.removeItem(its[i]);
    }
}

void MainWindow::plotTree(QImage &img, vector<node*> &path)
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
            //pntr.drawLine((int)x1, (int)y1, (int)x2, (int)y2);
            pntr.drawRect((int)(x1-2), (int)(y1-2), 4, 4);
            pntr.drawRect((int)(x2-2), (int)(y2-2), 4, 4);
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

void MainWindow::plotObs(QImage& img)
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

void MainWindow::plotAll(QImage& img, vector<node*> &path)
{
    plotTree(img, path);
    plotObs(img);
    img = img.mirrored();
    //scn.addPixmap(QPixmap::fromImage(img));
}

void MainWindow::on_saveImg_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, "save image", QDir::currentPath(),"Images(*.png)");
    QList<QGraphicsItem*> its=scn.items();
    QGraphicsPixmapItem* picItem = (QGraphicsPixmapItem*)its[0];
    picItem->pixmap().toImage().save(fileName);
    QString part = fileName.left(fileName.lastIndexOf("."));
    int num=1;
    for (QList<QImage>::Iterator i=imgList.begin(); i!=imgList.end(); ++i)
    {
        fileName = part + QString("-log")+QString::number(num++)+QString(".png");
        i->save(fileName);
    }
}

void MainWindow::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    menu.addAction(&resetMap);
    menu.exec(event->globalPos());
}
