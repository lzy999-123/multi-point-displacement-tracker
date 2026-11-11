#include "camerawifisession.h"

#include <QDebug>
#include <QNetworkInterface>
#include <QTimer>
#include <QElapsedTimer>
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
    if (!s.handle) return;
    DWORD size;
    WLAN_OPCODE_VALUE_TYPE type;
    PVOID data = nullptr;
    QString connected;
    if (WlanQueryInterface(s.handle, &s.guid, wlan_intf_opcode_current_connection,
                           nullptr, &size, &data, &type) == ERROR_SUCCESS) {
        connected = QString::fromWCharArray(static_cast<PWLAN_CONNECTION_ATTRIBUTES>(data)->strProfileName);
        WlanFreeMemory(data);
    }
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
        data = nullptr;
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
        for (const auto &ip : QNetworkInterface::allAddresses())
            addressReady |= ip.protocol() == QAbstractSocket::IPv4Protocol &&
                            ip.isInSubnet(QHostAddress("192.168.4.0"), 24);
        if (addressReady) { s.announced = true; emit ready(); }
    }
}

CameraWifiSession::~CameraWifiSession()
{
    State &s = *m_state;
    s.timer.stop();
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
