#include "AP_DDS_Client.h"

#include <AP_SerialManager/AP_SerialManager.h>

/*
  implement C functions for serial transport
 */
extern "C" {
#include <uxr/client/profile/transport/serial/serial_transport_platform.h>
}

bool uxr_init_serial_platform(void* args, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    struct uxrSerialPlatform *serial = (struct uxrSerialPlatform *)args;
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    auto *dds_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (dds_port == nullptr) {
        return false;
    }
    // ensure we own the UART
    dds_port->begin(0);
    serial->data = dds_port;
    return true;
}

bool uxr_close_serial_platform(void* args)
{
    //! @todo Add error reporting
    return true;
}

size_t uxr_write_serial_data_platform(void* args, const uint8_t* buf, size_t len, uint8_t* errcode)
{
    AP_HAL::UARTDriver *dds_port = (AP_HAL::UARTDriver *)((struct uxrSerialPlatform *)args)->data;
    if (dds_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    ssize_t bytes_written = dds_port->write(buf, len);
    if (bytes_written <= 0) {
        *errcode = 1;
        return 0;
    }
    //! @todo Add populate the error code correctly
    *errcode = 0;
    return bytes_written;
}

size_t uxr_read_serial_data_platform(void* args, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
    AP_HAL::UARTDriver *dds_port = (AP_HAL::UARTDriver *)((struct uxrSerialPlatform *)args)->data;
    if (dds_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    while (timeout > 0 && dds_port->available() < len) {
        hal.scheduler->delay(1); // TODO select or poll this is limiting speed (1mS)
        timeout--;
    }
    ssize_t bytes_read = dds_port->read(buf, len);
    if (bytes_read <= 0) {
        *errcode = 1;
        return 0;
    }
    //! @todo Add error reporting
    *errcode = 0;
    return bytes_read;
}
