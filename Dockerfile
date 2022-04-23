FROM ghcr.io/jpconstantineau/docker_arduino_cli:latest

RUN arduino-cli config init
COPY arduino-cli.yaml arduino-cli.yaml
RUN arduino-cli core update-index --config-file arduino-cli.yaml
RUN arduino-cli core install esp32:esp32
RUN arduino-cli lib install "Rosserial Arduino Library"@0.9.1
RUN arduino-cli lib install "Adafruit GFX Library"
RUN arduino-cli lib install "Adafruit SSD1306"
RUN arduino-cli lib install "Adafruit BNO055"
RUN arduino-cli lib install "arduino-timer"
#RUN arduino-cli lib install "ESP32 AnalogWrite"
RUN pip3 install pyserial
