#include <stdint.h>
#include "usb.h"

const struct usb_string_descriptor lang_desc = USB_ARRAY_DESC(USB_LANGID_ENG_US);
const struct usb_string_descriptor manuf_desc_en = USB_STRING_DESC("KMZ Brno I");
const struct usb_string_descriptor prod_desc_en = USB_STRING_DESC("MTB-USB v4");
const struct usb_string_descriptor cdc_iface_desc_en = USB_STRING_DESC("MTB-USB Serial Interface");
struct usb_string_descriptor serial_number_desc_en = USB_STRING_DESC("0001");
