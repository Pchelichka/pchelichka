import hid

# default is TinyUSB (0xcafe), Adafruit (0x239a), RaspberryPi (0x2e8a), Espressif (0x303a) VID
USB_VID = (0xcafe, 0x239a, 0x2886, 0x303a)
def main():
    for vid in  USB_VID:
        for dict in hid.enumerate(vid):
            dev = hid.Device(dict['vendor_id'], dict['product_id'])
            print(dev)
            if dev:
                while True:
                    # Get input from console and encode to UTF8 for array of chars.
                    # hid generic inout is single report therefore by HIDAPI requirement
                    # it must be preceded with 0x00 as dummy reportID
                    str_out = b'\x00'
                    str_out += input("Send text to HID Device : ").encode('utf-8')
                    dev.write(str_out)
                    str_in = dev.read(64)
                    print("Received from HID Device:", str_in, '\n')

if __name__ == '__main__':
    main()