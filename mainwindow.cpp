#include "mainwindow.h"
#include "ui_mainwindow.h"

cv::Rect range;
//cv::BackgroundSubtractorMOG bgSubtractor(5, 10, 0.5, false);
cv::BackgroundSubtractorMOG2 bgSubtractor;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    qDebug()<<"MainWindow";
//    rectType=DEFALT;
//    range.x=0;
//    range.y=0;
//    range.width=640;
//    range.height=300;
    ui->setupUi(this);
    connect(&timer,&QTimer::timeout,this,&MainWindow::UpdateImage);
}


void MainWindow::showCamera()
{
    if (capture.open(CAM_NO))
    {
        capture>>CamImg;
        cv::cvtColor(CamImg,CamImg,CV_BGR2RGB);
        ui->status->setText(tr("Camera open."));
        timer.start(30);
        ui->pushButton->setEnabled(false);
        ui->pushButton_2->setEnabled(true);
        ui->pushButton_3->setEnabled(true);
        ui->Button_decetion->setEnabled(true);
        ui->Button_retangle->setEnabled(true);
        ui->Button_polygon->setEnabled(true);
        ui->Button_circle->setEnabled(true);
    }
    else
    {
        ui->status->setText(tr("Camera open FAILED."));
    }
}

void MainWindow::pauseCamera()
{
    if (ui->pushButton_2->text()=="PauseCamera")
    {
        ui->pushButton_2->setText(tr("ContinueCamera"));
        ui->status->setText(tr("Pause"));
        timer.stop();

    }else
    {
        timer.start(30);
        ui->pushButton_2->setText(tr("PauseCamera"));
        ui->status->setText(tr("Camera open."));
    }

}

void MainWindow::closeCamera()
{
    timer.stop();
    setDecetion(false);
    rectType=DEFALT;
    ui->label->resize(0,0);
    ui->status->setText(tr("Camera closed."));
    ui->pushButton->setEnabled(true);
    ui->pushButton_2->setEnabled(false);
    ui->pushButton_2->setText(tr("PauseCamera"));
    ui->pushButton_3->setEnabled(false);
    ui->Button_decetion->setEnabled(false);
    ui->Button_retangle->setEnabled(false);
    ui->Button_polygon->setEnabled(false);
    ui->Button_circle->setEnabled(false);
    capture.release();
}

void MainWindow::paintEvent(QPaintEvent *e)
{
    QImage image2 = QImage((uchar*)(CamImg.data), CamImg.cols, CamImg.rows, QImage::Format_RGB888);
    ui->label->setPixmap(QPixmap::fromImage(image2));
    ui->label->resize(image2.size());
    ui->label->show();
}

void MainWindow::UpdateImage()
{
    capture>>CamImg;
    if (CamImg.data)
    {
        ui->status->setText(tr("Camera open."));

        switch (rectType)
        {
        case RETANGLE:
        {
            if (isPress) cv::rectangle(CamImg,
                          cv::Point(m_pointStart.x(),m_pointStart.y()),
                          cv::Point(m_pointEnd.x(),m_pointEnd.y()),
                                       cv::Scalar(0,0,255),5);
            int width=m_pointEnd.x()-m_pointStart.x();
            int height=m_pointEnd.y()-m_pointStart.y();
            range.x=width>0?m_pointStart.x():m_pointEnd.x();
            range.y=height>0?m_pointStart.y():m_pointEnd.y();
            range.width=abs(m_pointEnd.x()-m_pointStart.x());
            range.height=abs(m_pointEnd.y()-m_pointStart.y());
            qDebug()<<range.x<<range.y<<range.width<<range.height;
            break;
        }
        case CIRCLE:
        {
            int width=m_pointEnd.x()-m_pointStart.x();
            int height=m_pointEnd.y()-m_pointStart.y();
            range.x=width>0?m_pointStart.x():m_pointEnd.x();
            range.y=height>0?m_pointStart.y():m_pointEnd.y();

            range.width=abs(m_pointEnd.x()-m_pointStart.x());
            range.height=abs(m_pointEnd.y()-m_pointStart.y());

            roRect.center=cv::Point2f(range.x+range.width/2, range.y+range.height/2);
            roRect.size=cv::Size2f(range.width, range.height);
            roRect.angle=0;  //可放局部变量。（考虑减少一下全局变量的数量）
            cv::ellipse(CamImg, roRect, cv::Scalar(0, 0, 255), 3);

            break;
        }
        case POLYGON:
            if (isPolyClosed && polyPoints.size()>=3)
            {
                int min_x=CamImg.rows,max_x=0,min_y=CamImg.cols,max_y=0;
                for (auto i=polyPoints.begin();i!=polyPoints.end()-1;i++)
                {
                    min_x=(*i).x<min_x?(*i).x:min_x;
                    max_x=(*i).x>max_x?(*i).x:max_x;
                    min_y=(*i).y<min_y?(*i).y:min_y;
                    max_y=(*i).y>max_y?(*i).y:max_y;

                    cv::line(CamImg,*i,*(i+1),cv::Scalar(0,0,255),3);
                }
                cv::line(CamImg,*(polyPoints.end()-1),polyPoints[0],cv::Scalar(0,0,255),3);
                range.x=min_x;
                range.y=min_y;
                range.width=max_x-min_x;
                range.height=max_y-min_y;
            }
            else if (polyPoints.size()&&polyPoints.size()>1)
            {
                isPolyClosed=false;
                for (auto i=polyPoints.begin();i!=polyPoints.end()-1;i++)
                {
                    cv::line(CamImg,*i,*(i+1),cv::Scalar(0,0,255),3);
                }
            }else if(polyPoints.size()==1)
            {
                isPolyClosed=false;
                cv::circle(CamImg,polyPoints[0],1,cv::Scalar(0,0,255),3);
            }
            break;
        case DEFALT:
            range.x=0;
            range.y=0;
            range.width=640;
            range.height=300;
            break;
        }

        if (decetion) ProcessImage();
        cv::cvtColor(CamImg,CamImg,CV_BGR2RGB);
        this->update();
    }
    else
    {
        closeCamera();
        ui->status->setText(tr("Camera lost."));
    }
}

void MainWindow::ProcessImage()
{
    cv::Mat rect_range;
    switch (rectType)
    {
    case RETANGLE:
    {
        rect_range=CamImg(range);
        break;
    }
    case CIRCLE:
    {
        cv::Mat circle_mask;
        circle_mask = cv::Mat::zeros(CamImg.rows, CamImg.cols, CV_8UC1);
        cv::ellipse(circle_mask, roRect, cv::Scalar(255, 255, 255), -1);
        cv::bitwise_and(CamImg, CamImg, rect_range,circle_mask);
        rect_range=rect_range(range);
        break;
    }
    case POLYGON:
    {
        cv::Mat polygon_mask;
        polygon_mask=cv::Mat::zeros(CamImg.rows,CamImg.cols,CV_8UC1);
        const cv::Point* ppt[1] = { &polyPoints[0] };
        int npt[] = { polyPoints.size() };
        fillPoly(polygon_mask, ppt, npt, 1, cv::Scalar(255, 255, 255));

        cv::bitwise_and(CamImg, CamImg, rect_range,polygon_mask);
        rect_range=rect_range(range);

        break;
    }
    case DEFALT:
        break;

    }

    bgSubtractor(rect_range, mask, 0.0005);
    threshold(mask, mask, 180, 255, CV_THRESH_BINARY);

    int Point_Count = 0;
    for (int i = 0; i < range.height; i++)
    {
        for (int j = 0; j < range.width; j++)
        {
            if (mask.at<uchar>(i, j) == 255)
            {
                Point_Count++;
                circle(CamImg, cv::Point(range.x+j, range.y+i), 1, cv::Scalar(0, 0, 255));
            }
        }
    }
    if (Point_Count) qDebug()<<Point_Count;
    int thresold = range.height*range.width*0.05;
    if (Point_Count>thresold)
    {
        ui->status->setText("Alarming!!!");
    }

}

void MainWindow::mousePressEvent(QMouseEvent *event)
{//考虑左右键press问题
    isPress=false;
    switch (rectType)
    {
    case RETANGLE:
    case CIRCLE:
        setDecetion(false);
        if (event->x()-ui->label->x()>=ui->label->width())
            m_pointStart.setX(ui->label->x()+ui->label->width()-15);
        else if(event->x()<ui->label->x())
            m_pointStart.setX(ui->label->x()-7);
            else
            m_pointStart.setX(event->x()-ui->label->x());

        if(event->y()-ui->label->y()>=ui->label->height())
            m_pointStart.setY(ui->label->y()+ui->label->height()-15);
        else if(event->y()<ui->label->y())
            m_pointStart.setY(ui->label->y()-7);
        else
            m_pointStart.setY(event->y()-ui->label->y());
        m_pointEnd=m_pointStart;
        isPress=true;
        break;
    case POLYGON:
        {
        if (isPolyClosed)
        {
            setDecetion(false);
            isPolyClosed=false;
            polyPoints.clear();
        }


        if (event->x()-ui->label->x()>=ui->label->width())
            tmp.x=ui->label->x()+ui->label->width()-15;
        else if(event->x()<ui->label->x())
            tmp.x=ui->label->x()-7;
            else
            tmp.x=event->x()-ui->label->x();

        if(event->y()-ui->label->y()>=ui->label->height())
           tmp.y=ui->label->y()+ui->label->height()-15;
        else if(event->y()<ui->label->y())
            tmp.y=ui->label->y()-7;
        else
            tmp.y=event->y()-ui->label->y();

        isPress=true;


        break;
        }
    case DEFALT:
        break;
    }

    qDebug()<<"mousePress"<<m_pointStart;
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    switch (rectType)
    {
    case RETANGLE:
    case CIRCLE:
        //get QPoint->opencvPoint && not out of bounding.
        if (event->x()-ui->label->x()>=ui->label->width())
            m_pointEnd.setX(ui->label->x()+ui->label->width()-15);
        else if(event->x()<ui->label->x())
            m_pointEnd.setX(ui->label->x()-7);
        else
            m_pointEnd.setX(event->x()-ui->label->x());

        if(event->y()-ui->label->y()>=ui->label->height())
            m_pointEnd.setY(ui->label->y()+ui->label->height()-15);
        else if(event->y()<ui->label->y())
            m_pointEnd.setY(ui->label->y()-7);
        else
            m_pointEnd.setY(event->y()-ui->label->y());
        break;

    case POLYGON:
        break;
    case DEFALT:
        break;
    }

    qDebug()<<"width:"<<ui->label->width()<<"minus:"<<event->x()-ui->label->x();
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{

    switch (rectType)
    {
    case RETANGLE:
    case CIRCLE:
        setDecetion(true);
        break;
    case POLYGON:
        if (event->x()-ui->label->x()>=ui->label->width())
            lineEndPoint.x=ui->label->x()+ui->label->width()-15;
        else if(event->x()<ui->label->x())
            lineEndPoint.x=ui->label->x()-7;
        else
            lineEndPoint.x=event->x()-ui->label->x();

        if(event->y()-ui->label->y()>=ui->label->height())
            lineEndPoint.y=ui->label->y()+ui->label->height()-15;
        else if(event->y()<ui->label->y())
            lineEndPoint.y=ui->label->y()-7;
        else
            lineEndPoint.y=event->y()-ui->label->y();
        //自动闭合(auto closed)
        if (polyPoints.size()>2 &&
                abs(lineEndPoint.x-polyPoints[0].x)<=20 && abs(lineEndPoint.y-polyPoints[0].y)<=20)
        {
           polyPoints.push_back(tmp);
           isPolyClosed=true;
           setDecetion(true);
        }
        if (isPress && !isPolyClosed) polyPoints.push_back(tmp);

        break;
    case DEFALT:
        break;
    }

}

void MainWindow::mouseDoubleClickEvent(QMouseEvent *event)
{
    qDebug()<<"DoubleClick";
    if (rectType==POLYGON)
    {
        polyPoints.push_back(tmp);
        isPolyClosed=true;
    }
    setDecetion(true);
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    showCamera();
}

void MainWindow::on_pushButton_2_clicked()
{
    pauseCamera();
}

void MainWindow::on_pushButton_3_clicked()
{
    closeCamera();
}

void MainWindow::on_Button_decetion_clicked()
{
    if (ui->Button_decetion->text()=="StartDecetion")
    {
        setDecetion(true);
    }
    else
    {
        setDecetion(false);
    }
}

void MainWindow::setDecetion(bool onoff)
{
    if (onoff)
    {
        decetion=true;
        ui->Button_decetion->setText(tr("StopDecetion"));
    }
    else
    {
        decetion=false;
        ui->Button_decetion->setText(tr("StartDecetion"));
    }
}

void MainWindow::on_Button_retangle_clicked()
{
    if (rectType!=RETANGLE)
    {
        m_pointStart.setX(0);
        m_pointStart.setY(0);
        m_pointEnd.setX(0);
        m_pointEnd.setY(0);
        setDecetion(false);
    }
    rectType=RETANGLE;
}

void MainWindow::on_Button_circle_clicked()
{
    if (rectType!=CIRCLE)
    {
        m_pointStart.setX(0);
        m_pointStart.setY(0);
        m_pointEnd.setX(0);
        m_pointEnd.setY(0);
        setDecetion(false);
    }
    rectType=CIRCLE;
}

void MainWindow::on_Button_polygon_clicked()
{
    if (rectType!=POLYGON)
    {

        m_pointStart.setX(0);
        m_pointStart.setY(0);
        m_pointEnd.setX(0);
        m_pointEnd.setY(0);

    }
    setDecetion(false);
    isPolyClosed=false;
    polyPoints.clear();
    rectType=POLYGON;

}
