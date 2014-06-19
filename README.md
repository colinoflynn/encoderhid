encoderhid
==========

Bunch of knobs connected to computer via USB. Useful for controlling software scope (e.g. PicoScope). See
http://www.colinoflynn.com/tiki-view_blog_post.php?postId=48 for details.

Built for the AT90USBKEY board using the USB1287 device, also works for the Teensy++ 2.0 using the USB1286
device. You've got to change both the MCU & F_CPU as follows:

=============  ============= ===============
Board           MCU=          F_CPU=
=============  ============= ===============
AT90USBKEY      at90usb1287   8000000
Teensy++ 2.0    at90usb1286   16000000
=============  ============= ===============

