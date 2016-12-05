/*****************************************************************************
* Copyright (c) 2016 GZHU_EENB_LAB629 Corporation
* All Rights Reserved.
*
* Project Name         :   PreventJumping
* File Name            :   main.cpp
* Abstract Description :   Call "MainWindow" to show UI.
*
* Create Date          :   2016/11/28
* Author               :   Zhu Zhihong
* Address              :   Guangzhou University(HEMC)

******************************************************************************/

#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
