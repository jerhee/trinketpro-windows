
all:
	pushd libarduinocore && $(MAKE) && popd
	pushd libarduinowire && $(MAKE) && popd
	pushd libarduinospi && $(MAKE) && popd
	pushd blink && $(MAKE) && popd
	pushd i2ceeprom && $(MAKE) && popd

clean:
	pushd libarduinocore && $(MAKE) clean && popd
	pushd libarduinowire && $(MAKE) clean && popd
	pushd libarduinospi && $(MAKE) clean && popd
	pushd blink && $(MAKE) clean && popd
	pushd i2ceeprom && $(MAKE) clean && popd