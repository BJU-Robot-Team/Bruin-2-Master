import QtQuick 2.0
import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.0

MenuItem {
    property string menuloc: "Location"
    text: menuloc
    onTriggered: if (popupWin.visible == true) {
                           popupWin.visible = false
                           popLocation.text = menuloc;
                       } else {
                           popupWin.visible = true
                           if (mainWin.state == "Waiting") {
                            popLocation.text = menuloc;
                           } else {
                               popLocation.text = ""
                           }
                       }
}

