# prerequisites: brew install, idf tool installs

cd tests

idf.py -T groundControl set-target esp32 build

pytest test.py \
            --target esp32 \
            --embedded-services=idf,qemu \
            --app-path . \
            --build-dir build \
            --maxfail=1 --disable-warnings -vv -s
