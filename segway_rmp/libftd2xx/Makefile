all: installed

TARBALL     = build/libftd2xx1.0.4.tar.gz
TARBALL_URL = http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx1.0.4.tar.gz
UNPACK_CMD  = tar zxf
SOURCE_DIR  = build/libftd2xx1.0.4
TARBALL_PATCH = 
MD5SUM_FILE = 
BUILD_DIR = $(SOURCE_DIR)/build/$(shell uname -m)

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(CURDIR)/include $(CURDIR)/lib
	cp $(BUILD_DIR)/libftd2xx.so.1.0.4 $(CURDIR)/lib
	ln -s $(CURDIR)/lib/libftd2xx.so.1.0.4 $(CURDIR)/lib/libftd2xx.so
	cp $(SOURCE_DIR)/ftd2xx.h $(SOURCE_DIR)/WinTypes.h $(CURDIR)/include
	touch installed

clean:
	rm -rf $(CURDIR)/lib $(CURDIR)/include stage installed patched 

wipe: clean
	rm -rf $(SOURCE_DIR)
	rm -rf build
