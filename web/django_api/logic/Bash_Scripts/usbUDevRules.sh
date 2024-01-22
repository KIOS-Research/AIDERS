####ADDING UDEV RULE FOR WEATHER STATION####
var_1='SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"weatherStation\"'

sudo bash -c "echo $var_1> /etc/udev/rules.d/99-usb-serial.rules"


####ADDING UDEV RULE FOR LORA STATION####
var_1='SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", SYMLINK+=\"loraStation\"'

sudo bash -c "echo $var_1>> /etc/udev/rules.d/99-usb-serial.rules"


sudo udevadm control --reload-rules && sudo udevadm trigger

