#!/bin/sh

$QT5/moc TopWnd.h -o moc_TopWnd.cpp
$QT5/moc CtrlWnd.h -o moc_CtrlWnd.cpp
$QT5/moc ViewWnd.h -o moc_ViewWnd.cpp
$QT5/moc PressButton.h -o moc_PressButton.cpp
$QT5/moc ManageWnd.h -o moc_ManageWnd.cpp
$QT5/moc SymbolButton.h -o moc_SymbolButton.cpp
$QT5/moc LinkWnd.h -o moc_LinkWnd.cpp
$QT5/moc TopButton.h -o moc_TopButton.cpp
$QT5/moc AddLinkWnd.h -o moc_AddLinkWnd.cpp

echo "-- Generated moc files"
