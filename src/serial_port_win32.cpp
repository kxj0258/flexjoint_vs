#include "serial_port.hpp"

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <string>

SerialPort::SerialPort(const std::string& device, int baud_rate)
    : device_(device), baud_rate_(baud_rate), handle_(INVALID_HANDLE_VALUE)
{}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open()
{
    const std::string port_name = normalize_device_name(device_);
    HANDLE handle = CreateFileA(port_name.c_str(),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                nullptr,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL,
                                nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "SerialPort: cannot open %s (Win32 error %lu)\n",
                port_name.c_str(), GetLastError());
        return false;
    }

    handle_ = handle;

    SetupComm(handle, 10240, 10240);

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 500;
    timeouts.WriteTotalTimeoutConstant = 2000;
    if (!SetCommTimeouts(handle, &timeouts)) {
        fprintf(stderr, "SerialPort: SetCommTimeouts failed (Win32 error %lu)\n",
                GetLastError());
        close();
        return false;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(handle, &dcb)) {
        fprintf(stderr, "SerialPort: GetCommState failed (Win32 error %lu)\n",
                GetLastError());
        close();
        return false;
    }

    dcb.BaudRate = static_cast<DWORD>(baud_rate_);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;

    if (!SetCommState(handle, &dcb)) {
        fprintf(stderr, "SerialPort: SetCommState failed (Win32 error %lu)\n",
                GetLastError());
        close();
        return false;
    }

    PurgeComm(handle, PURGE_TXCLEAR | PURGE_RXCLEAR);
    return true;
}

void SerialPort::close()
{
    HANDLE handle = static_cast<HANDLE>(handle_);
    if (handle != INVALID_HANDLE_VALUE) {
        CloseHandle(handle);
        handle_ = INVALID_HANDLE_VALUE;
    }
}

int SerialPort::write(const uint8_t* data, size_t len)
{
    HANDLE handle = static_cast<HANDLE>(handle_);
    DWORD written = 0;
    if (!WriteFile(handle, data, static_cast<DWORD>(len), &written, nullptr)) {
        fprintf(stderr, "SerialPort: WriteFile failed (Win32 error %lu)\n",
                GetLastError());
        return -1;
    }
    return static_cast<int>(written);
}

int SerialPort::read(uint8_t* buf, size_t max_len, int timeout_ms)
{
    HANDLE handle = static_cast<HANDLE>(handle_);

    COMMTIMEOUTS previous = {};
    if (!GetCommTimeouts(handle, &previous)) {
        fprintf(stderr, "SerialPort: GetCommTimeouts failed (Win32 error %lu)\n",
                GetLastError());
        return -1;
    }

    COMMTIMEOUTS timeouts = previous;
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(std::max(timeout_ms, 0));

    if (!SetCommTimeouts(handle, &timeouts)) {
        fprintf(stderr, "SerialPort: SetCommTimeouts failed (Win32 error %lu)\n",
                GetLastError());
        return -1;
    }

    DWORD bytes_read = 0;
    const BOOL ok = ReadFile(handle, buf, static_cast<DWORD>(max_len), &bytes_read, nullptr);
    const DWORD read_error = GetLastError();

    SetCommTimeouts(handle, &previous);

    if (!ok) {
        fprintf(stderr, "SerialPort: ReadFile failed (Win32 error %lu)\n", read_error);
        return -1;
    }
    return static_cast<int>(bytes_read);
}

bool SerialPort::flush_input()
{
    HANDLE handle = static_cast<HANDLE>(handle_);
    if (handle == INVALID_HANDLE_VALUE)
        return false;
    if (!PurgeComm(handle, PURGE_RXCLEAR)) {
        fprintf(stderr, "SerialPort: PurgeComm RX failed (Win32 error %lu)\n",
                GetLastError());
        return false;
    }
    return true;
}

bool SerialPort::flush_io()
{
    HANDLE handle = static_cast<HANDLE>(handle_);
    if (handle == INVALID_HANDLE_VALUE)
        return false;
    if (!PurgeComm(handle, PURGE_TXCLEAR | PURGE_RXCLEAR)) {
        fprintf(stderr, "SerialPort: PurgeComm IO failed (Win32 error %lu)\n",
                GetLastError());
        return false;
    }
    return true;
}

std::string SerialPort::normalize_device_name(const std::string& device)
{
    if (device.rfind("\\\\.\\", 0) == 0)
        return device;

    std::string upper = device;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });

    if (upper.rfind("COM", 0) == 0)
        return "\\\\.\\" + device;

    return device;
}

#endif
