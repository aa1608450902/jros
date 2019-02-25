#!/bin/sh
$QT5/moc TopWnd.h -o moc_TopWnd.cpp
$QT5/moc HeadWnd.h -o moc_HeadWnd.cpp
$QT5/moc ViewWnd.h -o moc_ViewWnd.cpp
$QT5/moc ManageWnd.h -o moc_ManageWnd.cpp
$QT5/moc ViewArea.h -o moc_ViewArea.cpp
$QT5/moc CtrlArea.h -o moc_CtrlArea.cpp
$QT5/moc AddLinkDialog.h -o moc_AddLinkDialog.cpp
echo "-- Generated moc files"
