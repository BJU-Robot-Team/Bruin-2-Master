#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QObject>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/Map.qml")));

    return app.exec();
}

/*class ApplicationData : public QObject
{
   Q_OBJECT
public:
    Q_INVOKABLE QDateTime getState() const {
        return debug.string::state;
    }
}; */
