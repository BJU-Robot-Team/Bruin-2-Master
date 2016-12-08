import QtQuick 2.0
import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.0

Item {
    property string locationName: "Location"
    property int locx: 0
    property int locy: 0
    property int winWdth: 0
    property int winHt: 0

    width: 40
    height: 40
    x: (winWdth/1024)*locx
    y: (winHt/600)*locy
    Button {
        id: locIcon
        width: 40
        height: 40
        onClicked: if (popupWin.visible == true) {
                       popupWin.visible = false
                       popLocation.text = locationName;
                   } else {
                       popupWin.visible = true
                       if (mainWin.state == "Waiting") {
                        popLocation.text = locationName;
                       } else {
                           popLocation.text = ""
                       }
                   }
        background: Image {
            x: 0
            y: 0
            source: "qrc:/imgs/location.png"
            width: locIcon.width
            height: locIcon.height
        }
        Text {
            id: locText
            x: 0
            y: 46
            text: locationName
            font.italic: true
            font.bold: true
            font.pixelSize: 20
        }
    }
}
