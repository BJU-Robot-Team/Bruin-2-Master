import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.0

Item {
        property bool debugVis: false
        Rectangle {
            id: dWindow
            y: 380
            width: 200
            height: 200
            color: "#ffffff"
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 20
            anchors.left: parent.left
            anchors.leftMargin: 30
            visible: debugVis

            Text {
                id: text2
                x: 46
                y: 8
                text: qsTr("Sensor Information")
                font.bold: true
                font.pixelSize: 12
            }

            Text {
                id: text3
                x: 8
                text: qsTr("Text")
                anchors.top: text2.bottom
                anchors.topMargin: 10
                font.pixelSize: 12
            }

            Text {
                id: text4
                x: 8
                width: 24
                height: 11
                text: qsTr("Text")
                anchors.top: text3.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }

            Text {
                id: text5
                x: 8
                text: qsTr("Text")
                anchors.top: text4.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }

            Text {
                id: text6
                x: 8
                text: qsTr("Text")
                anchors.top: text5.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }

            Text {
                id: text7
                x: 8
                text: qsTr("Text")
                anchors.top: text6.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }
        }
    }
