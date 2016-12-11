/*
 * cdcacm.h
 *
 *  Created on: 9 нояб. 2016 г.
 *      Author: frost
 */

#ifndef INCLUDE_CDCACM_H_
#define INCLUDE_CDCACM_H_


//void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);

void usb_cdc_hw_config(void);

//int cdcacm_control_request(usbd_device *usbd_dev,
//	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
//	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req));
//
//
//void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);

#endif /* INCLUDE_CDCACM_H_ */
