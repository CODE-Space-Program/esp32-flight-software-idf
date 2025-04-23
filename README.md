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

## unit test setup

TODO: which prerequisites to install to be able to run unit tests locally

we use the QEMU emulator to run cross-target unit tests, meaning we can test functionality like the kalman filter in isolation without having to flash a physical ESP32 chip.
run the tests on your machine like this: `$ sh run-tests.sh`

the `/tests` directory is a fork of [ESP-IDF's built-in unit-test-app](https://github.com/espressif/esp-idf/blob/master/tools/unit-test-app/README.md)

## TODO: debug vs production build
