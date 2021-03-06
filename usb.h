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

#ifndef __USB_H__
#define __USB_H__

#include <stddef.h>

#define BUF_SIZE 128

#ifndef STM32F1
#define STM32F1
#endif

#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

struct cdcacm_functional_descriptors_t {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed));

extern char output_buffer[BUF_SIZE];
extern size_t buffer_write_pos;
extern size_t buffer_read_pos;

extern usbd_device *usbd_dev;

extern const struct usb_device_descriptor dev;
extern const struct usb_endpoint_descriptor comm_endp[];
extern const struct usb_endpoint_descriptor data_endp[];
extern const struct usb_interface_descriptor comm_iface[];
extern const struct usb_interface_descriptor data_iface[];
extern const struct usb_interface ifaces[];
extern const struct usb_config_descriptor config;
extern const char * const usb_strings[];

extern uint8_t usbd_control_buffer[128];

extern bool serial_connected;

enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req));
void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);
void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);
void cdcacm_suspend(void);
void cdcacm_wkup(void);

size_t output_serial(usbd_device *usbd_dev);

#endif