#pragma once

#include <QObject>
#include <memory>

// Connect a configured camera WLAN and restore the previous profile on exit.
class CameraWifiSession : public QObject
{
    Q_OBJECT
public:
    explicit CameraWifiSession(const QString &profile, QObject *parent = nullptr);
    ~CameraWifiSession() override;
    void start();
signals:
    void ready();
private:
    struct State;
    std::unique_ptr<State> m_state;
    void poll();
    void applyConnection(const QString &connected);
};
