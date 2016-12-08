import QtQuick 2.0
import QtQuick 2.0
import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.0


Item {
    property string locationName: "Location"
    MenuItem {
        text: locationName
        onTriggered: if (popupWin.visible == true) {
                           popupWin.visible = false
                           popLocation.text = locationName;
                       } else {
                           popupWin.visible = true
                           popLocation.text = locationName;
                       }
    }
}
