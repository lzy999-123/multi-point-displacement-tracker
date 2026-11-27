#include "camerawifisession.h"

#include <QDebug>
#include <QNetworkInterface>
#include <QTimer>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <wlanapi.h>

struct CameraWifiSession::State {
    HANDLE handle = nullptr;
    GUID guid{};
    QString profile, previous;
    QTimer timer;
    QFutureWatcher<QString> query;
    QElapsedTimer lastAttempt;
    bool guarded = false, announced = false, restoreScan = false;
    BOOL originalScan = TRUE;

    void connectProfile(const QString &name) {
        WLAN_CONNECTION_PARAMETERS connection{};
        connection.wlanConnectionMode = wlan_connection_mode_profile;
        connection.strProfile = reinterpret_cast<LPCWSTR>(name.utf16());
        connection.dot11BssType = dot11_BSS_type_infrastructure;
        const DWORD error = WlanConnect(handle, &guid, &connection, nullptr);
        if (error != ERROR_SUCCESS) qWarning() << "Camera WLAN connect:" << error;
    }
    void set(WLAN_INTF_OPCODE opcode, BOOL value) {
        const DWORD error = WlanSetInterface(handle, &guid, opcode, sizeof(value), &value, nullptr);
        if (error != ERROR_SUCCESS) qWarning() << "Camera WLAN hint:" << int(opcode) << error;
    }
};

CameraWifiSession::CameraWifiSession(const QString &profile, QObject *parent)
    : QObject(parent), m_state(std::make_unique<State>())
{
    m_state->profile = profile;
    m_state->timer.setInterval(1000);
    connect(&m_state->timer, &QTimer::timeout, this, &CameraWifiSession::poll);
    connect(&m_state->query, &QFutureWatcher<QString>::finished, this, [this] {
        applyConnection(m_state->query.result());
    });
}

void CameraWifiSession::start()
{
    State &s = *m_state;
    DWORD version;
    if (WlanOpenHandle(2, nullptr, &version, &s.handle) != ERROR_SUCCESS) return;
    PWLAN_INTERFACE_INFO_LIST interfaces = nullptr;
    if (WlanEnumInterfaces(s.handle, nullptr, &interfaces) != ERROR_SUCCESS) return;
    if (!interfaces->dwNumberOfItems) { WlanFreeMemory(interfaces); return; }
    s.guid = interfaces->InterfaceInfo[0].InterfaceGuid;
    // Prefer the connected adapter on machines with more than one WLAN interface.
    for (DWORD i = 0; i < interfaces->dwNumberOfItems; ++i)
        if (interfaces->InterfaceInfo[i].isState == wlan_interface_state_connected)
            s.guid = interfaces->InterfaceInfo[i].InterfaceGuid;
    WlanFreeMemory(interfaces);
    DWORD size;
    WLAN_OPCODE_VALUE_TYPE type;
    PVOID data = nullptr;
    if (WlanQueryInterface(s.handle, &s.guid, wlan_intf_opcode_current_connection,
                           nullptr, &size, &data, &type) == ERROR_SUCCESS) {
        s.previous = QString::fromWCharArray(static_cast<PWLAN_CONNECTION_ATTRIBUTES>(data)->strProfileName);
        WlanFreeMemory(data);
    }
    if (!s.profile.isEmpty() && s.previous != s.profile) {
        s.connectProfile(s.profile);
        s.lastAttempt.start();
    }
    s.timer.start();
    QTimer::singleShot(0, this, &CameraWifiSession::poll);
}

void CameraWifiSession::poll()
{
    State &s = *m_state;
    if (!s.handle || s.query.isRunning()) return;
    const HANDLE handle = s.handle;
    const GUID guid = s.guid;
    s.query.setFuture(QtConcurrent::run([handle, guid] {
        DWORD size;
        WLAN_OPCODE_VALUE_TYPE type;
        PVOID data = nullptr;
        QString connected;
        if (WlanQueryInterface(handle, &guid, wlan_intf_opcode_current_connection,
                               nullptr, &size, &data, &type) == ERROR_SUCCESS) {
            connected = QString::fromWCharArray(static_cast<PWLAN_CONNECTION_ATTRIBUTES>(data)->strProfileName);
            WlanFreeMemory(data);
        }
        return connected;
    }));
}

void CameraWifiSession::applyConnection(const QString &connected)
{
    State &s = *m_state;
    if (connected.isEmpty() || (!s.profile.isEmpty() && connected != s.profile)) {
        // WLAN restores these per-interface hints automatically on disconnect.
        s.guarded = false;
        s.announced = false;
        if (!s.profile.isEmpty() && (!s.lastAttempt.isValid() || s.lastAttempt.elapsed() >= 6000)) {
            s.connectProfile(s.profile);
            s.lastAttempt.restart();
        }
        return;
    }
    if (!s.guarded) {
        DWORD size;
        WLAN_OPCODE_VALUE_TYPE type;
        PVOID data = nullptr;
        if (WlanQueryInterface(s.handle, &s.guid, wlan_intf_opcode_background_scan_enabled,
                               nullptr, &size, &data, &type) == ERROR_SUCCESS) {
            s.originalScan = *static_cast<BOOL *>(data);
            s.restoreScan = true;
            WlanFreeMemory(data);
        }
        s.set(wlan_intf_opcode_media_streaming_mode, TRUE);
        if (s.restoreScan) s.set(wlan_intf_opcode_background_scan_enabled, FALSE);
        s.guarded = true;
    }
    if (!s.announced) {
        bool addressReady = s.profile.isEmpty();
        // 手机热点的 DHCP 网段会变化，不能只接受相机 AP 的 192.168.4.x。
        // 只检查已运行的无线接口，排除 VPN、回环和 DHCP 尚未完成时的链路本地地址。
        for (const auto &iface : QNetworkInterface::allInterfaces()) {
            if (iface.type() != QNetworkInterface::Wifi ||
                !(iface.flags() & QNetworkInterface::IsUp) ||
                !(iface.flags() & QNetworkInterface::IsRunning)) continue;
            for (const auto &entry : iface.addressEntries()) {
                const auto ip = entry.ip();
                addressReady |= ip.protocol() == QAbstractSocket::IPv4Protocol &&
                                !ip.isNull() && !ip.isLoopback() &&
                                !ip.isInSubnet(QHostAddress("169.254.0.0"), 16);
            }
        }
        if (addressReady) { s.announced = true; emit ready(); }
    }
}

CameraWifiSession::~CameraWifiSession()
{
    State &s = *m_state;
    s.timer.stop();
    // The worker only captures the native handle/GUID. Join its bounded query
    // before closing the handle, so shutdown cannot race a WLAN API call.
    s.query.waitForFinished();
    if (s.handle) {
        if (s.guarded) {
            if (s.restoreScan) s.set(wlan_intf_opcode_background_scan_enabled, s.originalScan);
            s.set(wlan_intf_opcode_media_streaming_mode, FALSE);
        }
        if (!s.profile.isEmpty() && !s.previous.isEmpty() && s.previous != s.profile)
            s.connectProfile(s.previous);
        WlanCloseHandle(s.handle, nullptr);
    }
}
