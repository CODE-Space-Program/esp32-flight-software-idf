# flight software

the core logic of our [existing flight computer software](https://github.com/CODE-Space-Program/esp32-flight-software), migrated from the Arduino ecosystem to [ESP-IDF](https://github.com/espressif/esp-idf).

## development setup

create a directory that will contain both the ESP-IDF command line tool and this repository:
`$ mkdir spaceprogram; cd spaceprogram`

clone this repository into `spaceprogram`:
`$ git clone https://github.com/CODE-Space-Program/esp32-flight-software-idf`

install the ESP-IDF cli into `spaceprogram` (this will take a while):
`$ git clone --recursive https://github.com/espressif/esp-idf`
`$ ./esp-idf/install.sh`

source the IDF CLI - you will have to either do this for every new terminal session, or add it to your `.bashrc`. this adds `idf.py` to your `PATH`:
`$ . ./export.sh`

### running unit tests locally

TODO: which prerequisites to install to be able to run unit tests locally

we use the QEMU emulator to run off-target unit tests, meaning we can test functionality like the kalman filter in isolation without having to flash a physical ESP32 chip.
run the tests on your machine like this: `$ sh run-tests.sh`
the tests are `.c` files that are found in a component's `test` directory.
[learn more](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/unit-tests.html)

the global `/tests` directory is a fork of [ESP-IDF's built-in unit-test-app](https://github.com/espressif/esp-idf/blob/master/tools/unit-test-app/README.md)

### linting and IntelliSense setup for VS Code

1. install the `ms-vscode.cpptools` and `espressif.esp-idf-extension` extensions
2. press command + shift + p, type `> ESP-IDF: Add VS CODE Configuration Folder`, and press enter
3. press command + shift + p, type `> C/C++ Reset IntelliSense Database`, and press enter
4. press command + shift + p, type `> Developer: Reload Window`, and press enter

### building and flashing the binary

build the code:`$ idf.py fullclean build`

after physically connecting the ESP-32, find out which software corresponds to the cable that's connected to it:
see either [Check Port on Linux and macOS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/establish-serial-connection.html?utm_source=chatgpt.com#check-port-on-linux-and-macos) or [Check Port on Windows](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/establish-serial-connection.html?utm_source=chatgpt.com#check-port-on-windows)
the port will look something like `/dev/ttyUSB0`.

flash the compiled code onto the flight computer:
`$ idf.py -p PORT flash monitor` - replace PORT with the port you got from the previous step

## TODO: debug vs production build

## TODO: debugging with OpenOCD and GDB
