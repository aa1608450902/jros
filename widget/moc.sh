#!/bin/sh

$QT5/moc TopWnd.h -o moc_TopWnd.cpp
$QT5/moc CtrlWnd.h -o moc_CtrlWnd.cpp
$QT5/moc ViewWnd.h -o moc_ViewWnd.cpp

echo "-- Generated moc files"
