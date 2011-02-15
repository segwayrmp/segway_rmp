all: installed

UNAME := $(shell uname -s)

SVN_DIR = build/communications-svn
SVN_REVISION = -r 54
SVN_URL = https://haydn.upc.es/labrobotica/pub/applications/communications
SVN_PATCH = iricomms_ros.patch
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=DEBUG \
			 -D CMAKE_INSTALL_PREFIX=`rospack find iricomms`/ \
			 -D iriutils_FOUND=TRUE \
			 -D iriutils_INCLUDE_DIR=`rospack find iriutils`/include/iriutils/ \
			 -D BUILD_DOXYGEN_DOCS=OFF
ifeq ($(UNAME), Darwin)
	CMAKE_ARGS += -D iriutils_LIBRARY=`rospack find iriutils`/lib/iriutils/libiriutils.dylib
else
	CMAKE_ARGS += -D iriutils_LIBRARY=`rospack find iriutils`/lib/iriutils/libiriutils.so
endif

installed: wiped $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf installed wiped

wiped: Makefile $(SVN_PATCH)
	make wipe
	touch wiped

wipe: clean
	rm -rf patched build lib include Modules

.PHONY : clean download