# flight software

the core logic of our [existing flight computer software](https://github.com/CODE-Space-Program/esp32-flight-software), migrated from the Arduino ecosystem to [ESP-IDF](https://github.com/espressif/esp-idf).

## development setup

install the ESP-IDF cli:
`$ git clone --recursive https://github.com/espressif/esp-idf`
`$ cd esp-idf`
`$ ./install.sh`
`$ source ./export.sh`

## building and flashing the code

build the code:
`$ idf.py set-target esp32` - you only need to do this once
`$ idf.py build`

after physically connecting the ESP-32, find out which software corresponds to the cable that's connected to it:
see either [Check Port on Linux and macOS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/establish-serial-connection.html?utm_source=chatgpt.com#check-port-on-linux-and-macos) or [Check Port on Windows](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/establish-serial-connection.html?utm_source=chatgpt.com#check-port-on-windows)
the port will look something like `/dev/ttyUSB0`.

flash the compiled code onto the flight computer:
`$ idf.py -p PORT flash monitor` - replace PORT with the port you got from the previous step

## TODO: debugging with OpenOCD and GDB

## TODO: run unit tets

## TODO: debug vs production build
