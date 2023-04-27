#include "AP_DDS_Client.h"

#if AP_DDS_UDP_ENABLED

#include <AP_HAL/utility/Socket.h>
#include <errno.h>

/*
  implement C functions for UDP transport
 */
extern "C" {
#include <uxr/client/profile/transport/ip/udp/udp_transport.h>
/*
  the exposed headers are missing the UDP platform function prototypes
 */
bool uxr_init_udp_platform(struct uxrUDPPlatform* platform, uxrIpProtocol ip_protocol, const char* ip, const char* port);
bool uxr_close_udp_platform(struct uxrUDPPlatform* platform);
size_t uxr_write_udp_data_platform(struct uxrUDPPlatform* platform, const uint8_t* buf, size_t len, uint8_t* errcode);
size_t uxr_read_udp_data_platform(struct uxrUDPPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode);
}

bool uxr_init_udp_platform(uxrUDPPlatform* platform, uxrIpProtocol ip_protocol, const char* ip, const char* port)
{
    auto *sock = new SocketAPM(true);
    if (sock == nullptr) {
        return false;
    }
    if (!sock->connect(ip, atoi(port))) {
        return false;
    }
    platform->data = sock;
    return true;
}

bool uxr_close_udp_platform(uxrUDPPlatform* platform)
{
    SocketAPM *sock = (SocketAPM *)platform->data;
    delete sock;
    platform->data = nullptr;
    return true;
}

size_t uxr_write_udp_data_platform(uxrUDPPlatform* platform, const uint8_t* buf, size_t len, uint8_t* errcode)
{
    SocketAPM *sock = (SocketAPM *)platform->data;
    ssize_t ret = sock->send(buf, len);
    if (ret <= 0) {
        *errcode = errno;
        return 0;
    }
    return ret;
}

size_t uxr_read_udp_data_platform(uxrUDPPlatform* platform, uint8_t* buf, size_t len, int timeout_ms, uint8_t* errcode)
{
    SocketAPM *sock = (SocketAPM *)platform->data;
    ssize_t ret = sock->recv(buf, len, timeout_ms);
    if (ret <= 0) {
        *errcode = errno;
        return 0;
    }
    return ret;
}

#endif // AP_DDS_UDP_ENABLED
