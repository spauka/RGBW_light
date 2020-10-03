/*
 * Copyright 2020 Sebastian Pauka
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "usb.h"
#include "printf.h"

char output_buffer[BUF_SIZE];
size_t buffer_write_pos = 0;
size_t buffer_read_pos = 0;

const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200, // USB Version 2
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

struct cdcacm_functional_descriptors_t cdcacm_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength =
            sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
     }
};

const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors)
}};

const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0xfa,

    .interface = ifaces,
};

const char * const usb_strings[] = {
    "SKAM Industries",
    "SK6812 Light Controller",
    "LIGHT",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch(req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
        char local_buf[sizeof(struct usb_cdc_notification) + 2];
        struct usb_cdc_notification *notif = (struct usb_cdc_notification*)local_buf;

        // Check if we are connecting or disconnecting
        if (req->wValue & 1)
            serial_connected = true;
        else
            serial_connected = false;

        /* We echo signals back to host as notification. */
        notif->bmRequestType = 0xA1;
        notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
        notif->wValue = 0;
        notif->wIndex = 0;
        notif->wLength = 2;
        local_buf[8] = req->wValue & 1;
        local_buf[9] = 0;
        usbd_ep_write_packet(usbd_dev, 0x83, local_buf, 10);
        return USBD_REQ_HANDLED;
        }
    case USB_CDC_REQ_SET_LINE_CODING:
        if(*len < sizeof(struct usb_cdc_line_coding))
            return USBD_REQ_NOTSUPP;

        return USBD_REQ_HANDLED;
    }
    return USBD_REQ_NOTSUPP;
}

void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    size_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
    for (size_t i = 0; i < len; i++)
        _putchar(buf[i]);
}

void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                cdcacm_control_request);
}

void usb_lp_can_rx0_isr(void)
{
    /* Poll the state of the USB */
    usbd_poll(usbd_dev);
}

void cdcacm_suspend()
{
    *USB_CNTR_REG |= USB_CNTR_FSUSP;
}

void cdcacm_wkup()
{
    *USB_CNTR_REG &= ~USB_CNTR_FSUSP;
}

size_t output_serial(usbd_device *usbd_dev) {
    if (serial_connected) {
        size_t len = 0;
        if (buffer_read_pos != buffer_write_pos) {
            if (buffer_read_pos < buffer_write_pos) {
                len = usbd_ep_write_packet(usbd_dev, 0x82, &output_buffer[buffer_read_pos], buffer_write_pos-buffer_read_pos);
                buffer_read_pos += len;
            } else {
                len = usbd_ep_write_packet(usbd_dev, 0x82, &output_buffer[buffer_read_pos], BUF_SIZE-buffer_read_pos);
                buffer_read_pos = (buffer_read_pos+len)%BUF_SIZE;
            }
        }
        return len;
    }
    return 0;
}

void _putchar(char c)
{
    if (serial_connected) {
        if ((buffer_write_pos+1)%BUF_SIZE != buffer_read_pos) {
            output_buffer[buffer_write_pos] = c;
            buffer_write_pos = (buffer_write_pos+1) % BUF_SIZE;
        } else {
            if (output_serial(usbd_dev))
                _putchar(c);
        }
    }
}
