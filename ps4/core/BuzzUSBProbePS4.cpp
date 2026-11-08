// SPDX-License-Identifier: GPL-3.0+
// PS4 host USB -> PCSX2 virtual Buzz controller bridge (Sony/Logitech 054c:0002).
// Hardware I/O lives on a dedicated thread. Only the EE thread writes guest USB state.
#include "USB/USB.h"
#include <orbis/Usbd.h>
#include <orbis/libkernel.h>
#include <pthread.h>
#include <unistd.h>
#include <atomic>
#include <cstdint>

extern void ProbeLog(const char* format, ...);

namespace
{
constexpr uint16_t BUZZ_VID = 0x054c;
constexpr uint16_t BUZZ_PID = 0x0002;
constexpr unsigned GUEST_PORT = 0;
constexpr uint32_t BUTTON_MASK = 0x000fffff;

// PCSX2 Buzz binding indices use Red/Blue/Orange/Green/Yellow per player.
// The real 054c:0002 HID report orders those bits Red/Yellow/Green/Orange/Blue.
constexpr unsigned BIT_FOR_BIND[20] = {
    0, 4, 3, 2, 1,        // Player 1
    5, 9, 8, 7, 6,        // Player 2
    10, 14, 13, 12, 11,  // Player 3
    15, 19, 18, 17, 16   // Player 4
};

template <typename Function>
Function Resolve(int module, const char* symbol)
{
    void* address = nullptr;
    const int result = sceKernelDlsym(module, symbol, &address);
    if (result != 0 || !address)
    {
        ProbeLog("BUZZ_USB: symbol %s unavailable result=%#x\n", symbol, result);
        return nullptr;
    }
    return reinterpret_cast<Function>(address);
}

using InitFn = int32_t (*)();
using ExitFn = int32_t (*)();
using GetListFn = int32_t (*)(libusb_device***);
using FreeListFn = void (*)(libusb_device**, int);
using DescriptorFn = int32_t (*)(libusb_device*, libusb_device_descriptor*);
using ConfigFn = int32_t (*)(libusb_device*, libusb_config_descriptor**);
using ConfigIndexFn = int32_t (*)(libusb_device*, uint8_t, libusb_config_descriptor**);
using FreeConfigFn = void (*)(libusb_config_descriptor*);
using OpenFn = int32_t (*)(libusb_device*, libusb_device_handle**);
using OpenByIdFn = libusb_device_handle* (*)(uint16_t, uint16_t);
using ClaimFn = int32_t (*)(libusb_device_handle*, int32_t);
using ReleaseFn = int32_t (*)(libusb_device_handle*, int32_t);
using CloseFn = void (*)(libusb_device_handle*);
using InterruptFn = int32_t (*)(libusb_device_handle*, unsigned char, unsigned char*, int32_t, int32_t*, uint32_t);

struct USBFunctions
{
    InitFn init = nullptr;
    ExitFn exit = nullptr;
    GetListFn get_list = nullptr;
    FreeListFn free_list = nullptr;
    DescriptorFn get_descriptor = nullptr;
    ConfigFn get_active_config = nullptr;
    ConfigIndexFn get_config = nullptr;
    FreeConfigFn free_config = nullptr;
    OpenFn open = nullptr;
    OpenByIdFn open_by_id = nullptr;
    ClaimFn claim = nullptr;
    ReleaseFn release = nullptr;
    CloseFn close = nullptr;
    InterruptFn interrupt = nullptr;
};

USBFunctions s_api;
libusb_device_handle* s_handle = nullptr;
pthread_t s_thread{};
bool s_thread_started = false;
bool s_claimed = false;
bool s_initialized_usbd = false;
bool s_session = false;
int s_interface = 0;
unsigned char s_endpoint = 0x81;
uint32_t s_previous_guest_mask = ~uint32_t{0};
std::atomic<bool> s_polling{false};
std::atomic<uint32_t> s_buttons{0};
std::atomic<uint64_t> s_reports{0}, s_changes{0}, s_failures{0}, s_timeouts{0};

bool ResolveFunctions(int module)
{
    s_api.init = Resolve<InitFn>(module, "sceUsbdInit");
    s_api.exit = Resolve<ExitFn>(module, "sceUsbdExit");
    s_api.get_list = Resolve<GetListFn>(module, "sceUsbdGetDeviceList");
    s_api.free_list = Resolve<FreeListFn>(module, "sceUsbdFreeDeviceList");
    s_api.get_descriptor = Resolve<DescriptorFn>(module, "sceUsbdGetDeviceDescriptor");
    s_api.get_active_config = Resolve<ConfigFn>(module, "sceUsbdGetActiveConfigDescriptor");
    s_api.get_config = Resolve<ConfigIndexFn>(module, "sceUsbdGetConfigDescriptor");
    s_api.free_config = Resolve<FreeConfigFn>(module, "sceUsbdFreeConfigDescriptor");
    s_api.open = Resolve<OpenFn>(module, "sceUsbdOpen");
    s_api.open_by_id = Resolve<OpenByIdFn>(module, "sceUsbdOpenDeviceWithVidPid");
    s_api.claim = Resolve<ClaimFn>(module, "sceUsbdClaimInterface");
    s_api.release = Resolve<ReleaseFn>(module, "sceUsbdReleaseInterface");
    s_api.close = Resolve<CloseFn>(module, "sceUsbdClose");
    s_api.interrupt = Resolve<InterruptFn>(module, "sceUsbdInterruptTransfer");
    return s_api.init && s_api.get_list && s_api.free_list && s_api.get_descriptor &&
        (s_api.open || s_api.open_by_id) && s_api.close && s_api.interrupt;
}

void InspectEndpoints(libusb_device* device)
{
    libusb_config_descriptor* config = nullptr;
    int result = -1;
    if (s_api.get_active_config)
        result = s_api.get_active_config(device, &config);
    if (result != 0 && s_api.get_config)
        result = s_api.get_config(device, 0, &config);

    ProbeLog("BUZZ_USB: config result=%#x interfaces=%u\n", result,
        config ? static_cast<unsigned>(config->bNumInterfaces) : 0u);
    if (result != 0 || !config)
        return;

    bool selected = false;
    for (unsigned i = 0; i < config->bNumInterfaces && !selected; ++i)
    {
        const libusb_interface& interface = config->interface[i];
        for (int alt = 0; alt < interface.num_altsetting && !selected; ++alt)
        {
            const libusb_interface_descriptor& desc = interface.altsetting[alt];
            ProbeLog("BUZZ_USB: interface=%u class=%02x endpoints=%u\n",
                static_cast<unsigned>(desc.bInterfaceNumber),
                static_cast<unsigned>(desc.bInterfaceClass),
                static_cast<unsigned>(desc.bNumEndpoints));
            for (unsigned e = 0; e < desc.bNumEndpoints; ++e)
            {
                const libusb_endpoint_descriptor& ep = desc.endpoint[e];
                ProbeLog("BUZZ_USB: endpoint=%02x attributes=%02x packet=%u\n",
                    static_cast<unsigned>(ep.bEndpointAddress),
                    static_cast<unsigned>(ep.bmAttributes),
                    static_cast<unsigned>(ep.wMaxPacketSize));
                if ((ep.bEndpointAddress & 0x80) && (ep.bmAttributes & 3) == 3)
                {
                    s_interface = desc.bInterfaceNumber;
                    s_endpoint = ep.bEndpointAddress;
                    selected = true;
                    break;
                }
            }
        }
    }
    if (s_api.free_config)
        s_api.free_config(config);
    ProbeLog("BUZZ_USB: selected interface=%d interrupt_in=%02x fallback=%d\n",
        s_interface, static_cast<unsigned>(s_endpoint), selected ? 0 : 1);
}

// The device returns 0x7f,0x7f, 20 button bits, and a 0xf high nibble.
// Some HID APIs prefix a zero report ID; accept both formats.
bool DecodeReport(const unsigned char* data, int size, uint32_t& buttons)
{
    int offset = 0;
    // Buzz's on-wire payload is 5 bytes (7f 7f + 20 bits + 4-bit tail).
    // Windows HID APIs may prepend a zero report-ID byte, making it 6.
    if (size >= 6 && data[0] == 0 && data[1] == 0x7f && data[2] == 0x7f)
        offset = 1;
    if (size < offset + 5 || data[offset] != 0x7f || data[offset + 1] != 0x7f)
        return false;
    buttons = (uint32_t(data[offset + 2]) |
        (uint32_t(data[offset + 3]) << 8) |
        (uint32_t(data[offset + 4] & 0x0f) << 16)) & BUTTON_MASK;
    return true;
}

void* USBThread(void*)
{
    ProbeLog("BUZZ_USB: STAGE=read-loop-start endpoint=%02x\n", static_cast<unsigned>(s_endpoint));
    uint64_t attempts = 0;
    while (s_polling.load(std::memory_order_acquire))
    {
        unsigned char report[64]{};
        int32_t received = 0;
        const int result = s_api.interrupt(s_handle, s_endpoint, report, sizeof(report), &received, 30);
        ++attempts;
        if (result == 0 && received > 0)
        {
            const uint64_t sample = s_reports.fetch_add(1, std::memory_order_relaxed) + 1;
            uint32_t mask = 0;
            const bool parsed = DecodeReport(report, received, mask);
            if (sample <= 8 || (!parsed && sample <= 20))
                ProbeLog("BUZZ_USB: HID sample=%llu len=%d raw=%02x %02x %02x %02x %02x %02x %02x %02x parsed=%d\n",
                    static_cast<unsigned long long>(sample), received,
                    report[0], report[1], report[2], report[3], report[4], report[5], report[6], report[7],
                    parsed ? 1 : 0);
            if (parsed)
            {
                const uint32_t old = s_buttons.exchange(mask, std::memory_order_release);
                if (mask != old)
                {
                    const uint64_t changed = s_changes.fetch_add(1, std::memory_order_relaxed) + 1;
                    if (changed <= 64)
                        ProbeLog("BUZZ_USB: HID buttons changed=%llu mask=%05x\n",
                            static_cast<unsigned long long>(changed), mask);
                }
            }
        }
        else if (result == -7) // libusb timeout, not a device failure
        {
            s_timeouts.fetch_add(1, std::memory_order_relaxed);
        }
        else
        {
            const uint64_t failures = s_failures.fetch_add(1, std::memory_order_relaxed) + 1;
            if (failures <= 10 || failures % 100 == 0)
                ProbeLog("BUZZ_USB: interrupt result=%#x bytes=%d failures=%llu\n",
                    result, received, static_cast<unsigned long long>(failures));
            if (result == -4) // device disconnected
                break;
            // Avoid spinning when the host rejects transfers immediately.
            usleep(10000);
        }
    }
    s_buttons.store(0, std::memory_order_release);
    s_polling.store(false, std::memory_order_release);
    ProbeLog("BUZZ_USB: STAGE=read-loop-stop calls=%llu\n", static_cast<unsigned long long>(attempts));
    return nullptr;
}
} // namespace

void PS4BuzzUSBStop()
{
    s_polling.store(false, std::memory_order_release);
    if (s_thread_started)
    {
        pthread_join(s_thread, nullptr);
        s_thread_started = false;
    }
    if (s_handle)
    {
        if (s_claimed && s_api.release)
        {
            const int result = s_api.release(s_handle, s_interface);
            ProbeLog("BUZZ_USB: release interface=%d result=%#x\n", s_interface, result);
        }
        if (s_api.close)
            s_api.close(s_handle);
        s_handle = nullptr;
    }
    if (s_initialized_usbd && s_api.exit)
    {
        const int result = s_api.exit();
        ProbeLog("BUZZ_USB: sceUsbdExit=%#x\n", result);
    }
    s_initialized_usbd = false;
    s_claimed = false;
    s_session = false;
    s_buttons.store(0, std::memory_order_release);
    ProbeLog("BUZZ_USB: STAGE=stopped reports=%llu changes=%llu failures=%llu timeouts=%llu\n",
        static_cast<unsigned long long>(s_reports.load()),
        static_cast<unsigned long long>(s_changes.load()),
        static_cast<unsigned long long>(s_failures.load()),
        static_cast<unsigned long long>(s_timeouts.load()));
}

void PS4BuzzUSBStart()
{
    s_session = true;
    s_previous_guest_mask = ~uint32_t{0};
    s_buttons.store(0);
    s_reports.store(0);
    s_changes.store(0);
    s_failures.store(0);
    s_timeouts.store(0);
    s_interface = 0;
    s_endpoint = 0x81;
    ProbeLog("BUZZ_USB: STAGE=starting target=%04x:%04x guest_port=%u\n",
        BUZZ_VID, BUZZ_PID, GUEST_PORT + 1);

    const int module = static_cast<int>(sceKernelLoadStartModule("libSceUsbd.sprx", 0, nullptr, 0, nullptr, nullptr));
    ProbeLog("BUZZ_USB: module load result=%#x\n", module);
    if (module < 0 || !ResolveFunctions(module))
    {
        ProbeLog("BUZZ_USB: STAGE=api-unavailable\n");
        return;
    }

    const int initialized = s_api.init();
    s_initialized_usbd = initialized == 0;
    ProbeLog("BUZZ_USB: sceUsbdInit=%#x\n", initialized);
    libusb_device** list = nullptr;
    const int count = s_api.get_list(&list);
    ProbeLog("BUZZ_USB: STAGE=enumerate count=%d list=%p\n", count, static_cast<void*>(list));
    if (count < 0 || !list)
    {
        ProbeLog("BUZZ_USB: STAGE=device-list-failed\n");
        return;
    }

    bool found = false;
    for (int i = 0; i < count && i < 64 && list[i]; ++i)
    {
        libusb_device_descriptor desc{};
        const int desc_result = s_api.get_descriptor(list[i], &desc);
        if (desc_result != 0)
        {
            ProbeLog("BUZZ_USB: descriptor device=%d failed=%#x\n", i, desc_result);
            continue;
        }
        ProbeLog("BUZZ_USB: device=%d vid=%04x pid=%04x\n", i,
            static_cast<unsigned>(desc.idVendor), static_cast<unsigned>(desc.idProduct));
        if (desc.idVendor != BUZZ_VID || desc.idProduct != BUZZ_PID)
            continue;

        found = true;
        ProbeLog("BUZZ_USB: STAGE=found device=%d\n", i);
        InspectEndpoints(list[i]);
        if (s_api.open)
        {
            const int result = s_api.open(list[i], &s_handle);
            ProbeLog("BUZZ_USB: sceUsbdOpen result=%#x handle=%p\n", result, static_cast<void*>(s_handle));
        }
        break;
    }
    s_api.free_list(list, 1);
    if (!found)
    {
        ProbeLog("BUZZ_USB: STAGE=not-found (plug receiver in before launching game)\n");
        return;
    }

    if (!s_handle && s_api.open_by_id)
    {
        s_handle = s_api.open_by_id(BUZZ_VID, BUZZ_PID);
        ProbeLog("BUZZ_USB: open-by-vidpid handle=%p\n", static_cast<void*>(s_handle));
    }
    if (!s_handle)
    {
        ProbeLog("BUZZ_USB: STAGE=open-failed\n");
        return;
    }
    if (s_api.claim)
    {
        const int result = s_api.claim(s_handle, s_interface);
        s_claimed = result == 0;
        ProbeLog("BUZZ_USB: claim interface=%d result=%#x\n", s_interface, result);
        // Attempt reads even when claim fails: PS4's HID driver may already own the interface.
    }
    else
    {
        ProbeLog("BUZZ_USB: claim-interface API unavailable; attempting direct read\n");
    }

    s_polling.store(true, std::memory_order_release);
    const int result = pthread_create(&s_thread, nullptr, USBThread, nullptr);
    s_thread_started = result == 0;
    ProbeLog("BUZZ_USB: STAGE=thread-start result=%d\n", result);
    if (result != 0)
        s_polling.store(false, std::memory_order_release);
}

void PS4BuzzUSBPollGuest()
{
    if (!s_session)
        return;
    const uint32_t mask = s_buttons.load(std::memory_order_acquire);
    const uint32_t diff = s_previous_guest_mask ^ mask;
    if (!diff)
        return;

    for (unsigned binding = 0; binding < 20; ++binding)
    {
        if (diff & (1u << BIT_FOR_BIND[binding]))
            USB::SetDeviceBindValue(GUEST_PORT, binding, (mask & (1u << BIT_FOR_BIND[binding])) ? 1.0f : 0.0f);
    }
    s_previous_guest_mask = mask;
    static unsigned forwarded = 0;
    if (++forwarded <= 64)
        ProbeLog("BUZZ_USB: guest update=%u raw_mask=%05x changed=%05x\n", forwarded, mask, diff);
}

void PS4BuzzUSBLogStatus()
{
    if (!s_session)
        return;
    ProbeLog("BUZZ_USB: status polling=%d reports=%llu button_changes=%llu failures=%llu timeouts=%llu guest_mask=%05x\n",
        s_polling.load() ? 1 : 0,
        static_cast<unsigned long long>(s_reports.load()),
        static_cast<unsigned long long>(s_changes.load()),
        static_cast<unsigned long long>(s_failures.load()),
        static_cast<unsigned long long>(s_timeouts.load()),
        s_previous_guest_mask & BUTTON_MASK);
}
