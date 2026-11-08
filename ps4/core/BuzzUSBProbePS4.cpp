// SPDX-License-Identifier: GPL-3.0+
// Diagnostic-only probe for Sony Buzz USB receiver on PS4.
// Opt in by creating /data/PS4SX2/vm-probe-buzz-usb before starting PS4SX2.
// This does not claim or read the device; it only enumerates descriptors.
#include <orbis/Usbd.h>
#include <orbis/libkernel.h>
#include <unistd.h>
#include <cstdint>

extern void ProbeLog(const char* format, ...);

namespace
{
template <typename Function>
Function ResolveUSB(int module, const char* symbol)
{
    void* address = nullptr;
    const int result = sceKernelDlsym(module, symbol, &address);
    if (result != 0 || address == nullptr)
    {
        ProbeLog("Buzz USB probe: cannot resolve %s (result=%#x)\n", symbol, result);
        return nullptr;
    }
    return reinterpret_cast<Function>(address);
}
}

void PS4BuzzUSBProbe()
{
    if (access("/data/PS4SX2/vm-probe-buzz-usb", F_OK) != 0)
        return;

    ProbeLog("Buzz USB probe: starting read-only enumeration\n");
    const int module = static_cast<int>(sceKernelLoadStartModule("libSceUsbd.sprx", 0, nullptr, 0, nullptr, nullptr));
    ProbeLog("Buzz USB probe: libSceUsbd module=%#x\n", module);
    if (module < 0)
        return;

    using Init = int32_t (*)();
    using GetList = int32_t (*)(libusb_device***);
    using GetDescriptor = int32_t (*)(libusb_device*, libusb_device_descriptor*);
    // PS4 Usbd implementations historically use an additional unref argument.
    // Supplying it is compatible with the single-argument OpenOrbis declaration on x86-64.
    using FreeList = void (*)(libusb_device**, int);
    using GetConfig = int32_t (*)(libusb_device*, libusb_config_descriptor**);
    using FreeConfig = void (*)(libusb_config_descriptor*);

    const auto init = ResolveUSB<Init>(module, "sceUsbdInit");
    const auto get_list = ResolveUSB<GetList>(module, "sceUsbdGetDeviceList");
    const auto get_descriptor = ResolveUSB<GetDescriptor>(module, "sceUsbdGetDeviceDescriptor");
    const auto free_list = ResolveUSB<FreeList>(module, "sceUsbdFreeDeviceList");
    const auto get_config = ResolveUSB<GetConfig>(module, "sceUsbdGetActiveConfigDescriptor");
    const auto free_config = ResolveUSB<FreeConfig>(module, "sceUsbdFreeConfigDescriptor");
    if (!init || !get_list || !get_descriptor || !free_list)
        return;

    const int init_result = init();
    ProbeLog("Buzz USB probe: sceUsbdInit=%#x\n", init_result);

    libusb_device** list = nullptr;
    const int count = get_list(&list);
    ProbeLog("Buzz USB probe: device count=%d list=%p\n", count, static_cast<void*>(list));
    if (count < 0 || !list)
        return;

    bool found = false;
    for (int index = 0; index < count && index < 64 && list[index]; ++index)
    {
        libusb_device_descriptor descriptor{};
        const int result = get_descriptor(list[index], &descriptor);
        if (result != 0)
        {
            ProbeLog("Buzz USB probe: device %d descriptor failed=%#x\n", index, result);
            continue;
        }
        const bool match = descriptor.idVendor == 0x054c && descriptor.idProduct == 0x0002;
        ProbeLog("Buzz USB probe: device %d VID:PID=%04x:%04x%s\n",
            index, static_cast<unsigned>(descriptor.idVendor),
            static_cast<unsigned>(descriptor.idProduct), match ? " *** BUZZ FOUND ***" : "");
        if (!match)
            continue;

        found = true;
        if (get_config && free_config)
        {
            libusb_config_descriptor* configuration = nullptr;
            const int config_result = get_config(list[index], &configuration);
            ProbeLog("Buzz USB probe: active config result=%#x interfaces=%u\n", config_result,
                configuration ? static_cast<unsigned>(configuration->bNumInterfaces) : 0u);
            if (config_result == 0 && configuration)
            {
                for (int interface_index = 0; interface_index < configuration->bNumInterfaces; ++interface_index)
                {
                    const libusb_interface& interface = configuration->interface[interface_index];
                    for (int alt = 0; alt < interface.num_altsetting; ++alt)
                    {
                        const libusb_interface_descriptor& desc = interface.altsetting[alt];
                        ProbeLog("Buzz USB probe: interface=%u class=%02x endpoints=%u\n",
                            static_cast<unsigned>(desc.bInterfaceNumber),
                            static_cast<unsigned>(desc.bInterfaceClass),
                            static_cast<unsigned>(desc.bNumEndpoints));
                        for (int endpoint = 0; endpoint < desc.bNumEndpoints; ++endpoint)
                        {
                            const libusb_endpoint_descriptor& ep = desc.endpoint[endpoint];
                            ProbeLog("Buzz USB probe: endpoint=%02x attributes=%02x packet=%u\n",
                                static_cast<unsigned>(ep.bEndpointAddress),
                                static_cast<unsigned>(ep.bmAttributes),
                                static_cast<unsigned>(ep.wMaxPacketSize));
                        }
                    }
                }
                free_config(configuration);
            }
        }
    }
    ProbeLog("Buzz USB probe: finished found=%d\n", found ? 1 : 0);
    free_list(list, 1);
}
