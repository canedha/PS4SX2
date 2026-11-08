// SPDX-License-Identifier: LGPL-2.1-or-later
// Minimal libusb-1.0 type compatibility for OpenOrbis orbis/Usbd.h.
// Based on public libusb descriptor layouts (libusb/libusb, libusb.h).
// No libusb implementation is linked; PS4 exports are resolved from libSceUsbd.sprx.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct libusb_device libusb_device;
typedef struct libusb_device_handle libusb_device_handle;
struct libusb_transfer;
struct libusb_control_setup;
typedef void (*libusb_transfer_cb_fn)(struct libusb_transfer* transfer);

typedef struct libusb_device_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} libusb_device_descriptor;

typedef struct libusb_endpoint_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
    uint8_t bRefresh;
    uint8_t bSynchAddress;
    const unsigned char* extra;
    int extra_length;
} libusb_endpoint_descriptor;

typedef struct libusb_interface_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
    const struct libusb_endpoint_descriptor* endpoint;
    const unsigned char* extra;
    int extra_length;
} libusb_interface_descriptor;

typedef struct libusb_interface
{
    const struct libusb_interface_descriptor* altsetting;
    int num_altsetting;
} libusb_interface;

typedef struct libusb_config_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t MaxPower;
    const struct libusb_interface* interface;
    const unsigned char* extra;
    int extra_length;
} libusb_config_descriptor;

#ifdef __cplusplus
}
#endif
