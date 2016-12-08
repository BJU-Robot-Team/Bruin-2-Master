// ENG_406_Bruin2_GUI
// Author: Caleb Wiggins

import QtQuick 2.0
import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.0

Window {
    id: mainWin
    visible: true
    width: 1024 //Screen.desktopAvailableHeight
    height: 600 //Screen.desktopAvailableWidth
    property alias text8: text8
    title: qsTr("ENG_406_GUI")

//----------------------------------------Globals------------------------------------------
    property string state:      "Waiting"
    property int    latitude:   0
    property int    longitude:  0
    property int    speed:      0


//----------------------------------------Globals------------------------------------------
    Image {
        id: testArea
        x: 0
        y: 0
        width: parent.width
        z: 2
        height: parent.height
        fillMode: Image.Stretch
        source: "qrc:/imgs/OFFICIAL_GUI_MAP.png"

        Rectangle {
            id: logo
            x: 782
            width: 212
            height: 92
            color: "#02BAFA"
            radius: 20
            anchors.right: parent.right
            anchors.rightMargin: 30
            anchors.top: parent.top
            anchors.topMargin: 20

            Text {
                id: text1
                x: 29
                y: 20
                color: "#ffffff"
                text: qsTr("Bruin 2")
                font.italic: true
                font.bold: true
                font.pixelSize: 45
            }
        }
//--------------------------------Debug Window------------------------------------

        Button {            
            id: debug 
            y: 497
            width: 100
            height: 50
            text: "Debug"
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 20
            anchors.left: parent.left
            anchors.leftMargin: 30
            onClicked: dWindow.visible = true

            background: Rectangle {
                radius: 20
                color: "orange"
                width: debug.width
                height: debug.height
            }                    
        }

        Rectangle {
            id: dWindow
            y: 382
            width: 200
            height: 250
            color: "#c8cac7"
            z: 10
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 20
            anchors.left: parent.left
            anchors.leftMargin: 30
            visible: false
            radius: 5

            Text {
                id: text2
                text: qsTr("Sensor Information")
                styleColor: "#000000"
                anchors.top: parent.top
                anchors.topMargin: 10
                anchors.left: parent.left
                anchors.leftMargin: 30
                font.bold: true
                font.pixelSize: 12
            }

            Text {
                id: text3
                x: 8
                text: qsTr("Heading: ")
                anchors.top: text2.bottom
                anchors.topMargin: 10
                font.pixelSize: 12
            }

            Text {
                id: text4
                x: 8
                width: 24
                height: 11
                text: qsTr("GPS Coords: ")
                anchors.top: text3.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }

            Text {
                id: text5
                x: 8
                text: qsTr("Speed [mph]: ")
                anchors.top: text4.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }

            Text {
                id: text6
                x: 8
                text: qsTr("Obstacle Detected?")
                anchors.top: text5.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }

            Text {
                id: text7
                x: 8
                text: qsTr("Brake Actuating?")
                anchors.top: text6.bottom
                anchors.topMargin: 20
                font.pixelSize: 12
            }

//          ----------------------------------
            Button {
                id: closePop
                x: 149
                y: -9
                width: 61
                height: 54
                text: ""
                anchors.verticalCenter: image1.verticalCenter
                anchors.horizontalCenter: image1.horizontalCenter
                visible: true
                transformOrigin: Item.Center
                onClicked: dWindow.visible = false
                opacity: 0
            }

            Image {
                id: image1
                x: 167
                y: 8
                width: 25
                height: 25
                z: 2
                source: "qrc:/imgs/close-button.png"
            }

            /*Button {
                id: button5
                x: 44
                y: 203
                text: qsTr("Enable Tests")
                anchors.horizontalCenter: parent.horizontalCenter
            }*/
        }

//---------------------------------------------Menu Button---------------------------------------------------

        Button {
            id: menuButton
            width: 106
            height: 33
            text: qsTr("Locations")
            anchors.top: parent.top
            anchors.topMargin: 20
            anchors.left: parent.left
            anchors.leftMargin: 30
            topPadding: 6


            onClicked: locations.open()
            Menu {
                id: locations
                MenuItem {
                    text: "Location 1"
                    onTriggered: if (popupWin.visible == true) {
                                     popupWin.visible = false
                                     popLocation.text = "Location 1";
                                 } else {
                                     popupWin.visible = true
                                     if (mainWin.state == "Waiting") {
                                      popLocation.text = "Location 1";
                                     } else {
                                         popLocation.text = ""
                                     }
                                 }
                }
                MenuItem {
                    text: "Location 2"
                    onTriggered: if (popupWin.visible == true) {
                                     popupWin.visible = false
                                     popLocation.text = "Location 2";
                                 } else {
                                     popupWin.visible = true
                                     if (mainWin.state == "Waiting") {
                                      popLocation.text = "Location 2";
                                     } else {
                                         popLocation.text = ""
                                     }
                                 }
                }
                MenuItem {
                    text: "Location 3"
                    onTriggered: if (popupWin.visible == true) {
                                     popupWin.visible = false
                                     popLocation.text = "Location 3";
                                 } else {
                                     popupWin.visible = true
                                     if (mainWin.state == "Waiting") {
                                      popLocation.text = "Location 3";
                                     } else {
                                         popLocation.text = ""
                                     }
                                 }
                }
                MenuItem {
                    text: "Location 4"
                    onTriggered: if (popupWin.visible == true) {
                                     popupWin.visible = false
                                     popLocation.text = "Location 4";
                                 } else {
                                     popupWin.visible = true
                                     if (mainWin.state == "Waiting") {
                                      popLocation.text = "Location 4";
                                     } else {
                                         popLocation.text = ""
                                     }
                                 }
                }
                MenuItem {
                    text: "Location 5"
                    onTriggered: if (popupWin.visible == true) {
                                     popupWin.visible = false
                                     popLocation.text = "Location 5";
                                 } else {
                                     popupWin.visible = true
                                     if (mainWin.state == "Waiting") {
                                      popLocation.text = "Location 5";
                                     } else {
                                         popLocation.text = ""
                                     }
                                 }
                }
                MenuItem {
                    text: "Location 6"
                    onTriggered: if (popupWin.visible == true) {
                                     popupWin.visible = false
                                     popLocation.text = "Location 6";
                                 } else {
                                     popupWin.visible = true
                                     if (mainWin.state == "Waiting") {
                                      popLocation.text = "Location 6";
                                     } else {
                                         popLocation.text = ""
                                     }
                                 }
                }
            }
        }

//----------------------Current Location/ Location Images---------------------------

        //----------Current Location Icon--------------
        Image {
            id: currentLoc
            property int currentlocX: 378
            property int currentlocY: 194
            x: (mainWin.width/1024)*currentlocX
            y: (mainWin.height/600)*currentlocY
            width: 30
            height: 30
            source: "qrc:/imgs/c_loc.png"
        }
        //--------------------------------------------

        Locations { locx: 384; locy: 54; winWdth: mainWin.width; winHt: mainWin.height; locationName: "Location 1" }
        Locations { locx: 177; locy: 167; winWdth: mainWin.width; winHt: mainWin.height; locationName: "Location 2" }
        Locations { locx: 418; locy: 275; winWdth: mainWin.width; winHt: mainWin.height; locationName: "Location 3" }
        Locations { locx: 418; locy: 448; winWdth: mainWin.width; winHt: mainWin.height; locationName: "Location 4" }
        Locations { locx: 688; locy: 417; winWdth: mainWin.width; winHt: mainWin.height; locationName: "Location 5" }
        Locations { locx: 539; locy: 208; winWdth: mainWin.width; winHt: mainWin.height; locationName: "Location 6" }

//-----------------------------------------------Popup Window----------------------------------------------------
        Rectangle {
            id: popupWin
            x: 365
            y: 225
            width: 250
            height: 150
            color: "#121D3B"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            visible: false

            Button {
                id: button1
                x: 31
                y: 90
                width: 70
                height: 50
                text: qsTr("Yes")
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 10
                onClicked: if (mainWin.state == "Waiting") {
                               popupWin.visible = false
                               mainWin.state = "Preparing to Travel"
                           }
                opacity: 0

            }
            Rectangle {
                id: rectangle1
                y: 1
                width: 70
                height: 30
                color: "#ffffff"
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 20
                anchors.left: parent.left
                anchors.leftMargin: 30
                anchors.verticalCenterOffset: 40
                anchors.horizontalCenterOffset: -59
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                visible: if (mainWin.state == "Waiting") {
                             true
                         } else {
                             false
                         }

                Text {
                    id: text8
                    x: 23
                    y: 8
                    text: qsTr("Yes")
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 12
                }
            }

            Button {
                id: button2
                x: 149
                y: 90
                width: 70
                height: 50
                anchors.verticalCenter: button1.verticalCenter
                onClicked: popupWin.visible = false;
                opacity: 0
            }

            Rectangle {
                id: rectangle2
                x: 149
                y: 100
                width: 70
                height: 30
                color: "#ffffff"
                anchors.rightMargin: 30
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 20
                anchors.right: parent.right
                visible: if (mainWin.state == "Waiting") {
                             true
                         } else {
                             false
                         }

                Text {
                    id: text9
                    x: 27
                    y: 8
                    text: qsTr("No")
                    anchors.verticalCenterOffset: 0
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    font.pixelSize: 12
                }
            }

            Text {
                id: popUpWarning
                x: 31
                y: 23
                width: 188
                height: 35
                color: "#ffffff"
                text: if (mainWin.state == "Waiting") {
                          qsTr("Are you sure you want to travel to this location? ")
                      } else {
                          if (mainWin.state == "Preparing to Travel") { qsTr("WARNING: Please wait to select a new location until the vehicle has arrived at the current destination") }
                      }
                anchors.horizontalCenter: parent.horizontalCenter
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 12
            }

            Text {
                id: popLocation
                x: 96
                y: 68
                width: 59
                height: 15
                color: "#ffffff"
                text: if (mainWin.state == "Waiting") {
                          qsTr("Text")
                      } else {
                          qsTr("")
                      }
                anchors.horizontalCenter: parent.horizontalCenter
                font.italic: true
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 12
            }

            Button {
                id: button4
                y: 315
                height: 50
                anchors.left: popupWin.right
                anchors.leftMargin: -160
                anchors.right: popupWin.left
                anchors.rightMargin: -160
                anchors.bottom: popupWin.top
                anchors.bottomMargin: -140
                anchors.verticalCenter: button1.verticalCenter
                onClicked: if (mainWin.state != "Waiting") {
                               popupWin.visible = false;
                           }
                opacity: 0
            }

            Rectangle {
                id: errOK
                x: 455
                y: 326
                width: 70
                height: 30
                color: "#ffffff"
                anchors.bottom: popupWin.top
                anchors.bottomMargin: -130
                anchors.horizontalCenter: popupWin.horizontalCenter
                visible: if (mainWin.state == "Waiting") {
                             false
                         } else {
                             true
                         }

                Text {
                    id: text10
                    x: 23
                    y: 8
                    text: qsTr("OK")
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    font.pixelSize: 12
                }
            }
        }

//--------------------------------State Box------------------------------------
        Rectangle {
            id: stateBox
            x: 870
            y: 511
            width: stateText.width + 20
            height: stateText.height + 20
            color: "#121D3B"
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 20
            anchors.right: parent.right
            anchors.rightMargin: 30

            Text {
                id: stateText
                x: 39
                y: 14
                color: "#ffffff"
                text: mainWin.state
                font.italic: true
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter   //qsTr(state)
                font.bold: true
                styleColor: "#000000"
                font.pixelSize: 20
            }
        }

//--------------------------------Random Contstruction------------------------------------


        Rectangle {
            id: testBox
            x: 794
            y: 200
            width: 200
            height: 200
            color: "#c8cac7"
            anchors.right: parent.right
            anchors.rightMargin: 30
            visible: false

            Button {
                id: button3
                x: 809
                text: qsTr("State_Cycle")
                anchors.top: parent.top
                anchors.topMargin: 10
                anchors.horizontalCenterOffset: 0
                anchors.horizontalCenter: parent.horizontalCenter
                z: 11
                onClicked: if (mainWin.state == "Waiting") {
                               mainWin.state = "Preparing to Travel"
                           } else {
                               mainWin.state = "Waiting"
                           }
            }

            TextInput {
                id: textInput1
                x: 8
                y: 90
                width: 80
                height: 20
                text: qsTr("Latitude")
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 12
            }

            Text {
                id: text11
                x: 88
                y: 69
                text: qsTr("Test Coords")
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 12
            }

            TextInput {
                id: textInput2
                x: 112
                y: 90
                width: 80
                height: 20
                text: qsTr("Longitude")
                horizontalAlignment: Text.AlignHCenter
                font.pixelSize: 12
            }
        }

    }

}
