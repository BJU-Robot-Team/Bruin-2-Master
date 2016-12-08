import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.0


    Menu {
        id: locations
        MenuItem {
            text: "Location 1"
            onTriggered: if (popupWin.visible == true) {
                             popupWin.visible = false
                             popLocation.text = "Location 1";
                         } else {
                             popupWin.visible = true
                             popLocation.text = "Location 1";
                         }
        }
        MenuItem {
            text: "Location 2"
            onTriggered: if (popupWin.visible == true) {
                             popupWin.visible = false
                             popLocation.text = "Location 2";
                         } else {
                             popupWin.visible = true
                             popLocation.text = "Location 2";
                         }
        }
        MenuItem {
            text: "Location 3"
            onTriggered: if (popupWin.visible == true) {
                             popupWin.visible = false
                             popLocation.text = "Location 3";
                         } else {
                             popupWin.visible = true
                             popLocation.text = "Location 3";
                         }
        }
        MenuItem {
            text: "Location 4"
            onTriggered: if (popupWin.visible == true) {
                             popupWin.visible = false
                             popLocation.text = "Location 4";
                         } else {
                             popupWin.visible = true
                             popLocation.text = "Location 4";
                         }
        }
        MenuItem {
            text: "Location 5"
            onTriggered: if (popupWin.visible == true) {
                             popupWin.visible = false
                             popLocation.text = "Location 5";
                         } else {
                             popupWin.visible = true
                             popLocation.text = "Location 5";
                         }
        }
        MenuItem {
            text: "Location 6"
            onTriggered: if (popupWin.visible == true) {
                             popupWin.visible = false
                             popLocation.text = "Location 6";
                         } else {
                             popupWin.visible = true
                             popLocation.text = "Location 6";
                         }
        }
    }
