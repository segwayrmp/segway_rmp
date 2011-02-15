all: installed

UNAME := $(shell uname -s)

SVN_DIR = build/segway_rmp_400-svn
SVN_REVISION = -r 17
SVN_URL = https://haydn.upc.es/labrobotica/pub/drivers/segway_rmp_400
SVN_PATCH = segway_rmp400_ros.patch
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=DEBUG \
			 -D CMAKE_INSTALL_PREFIX=`rospack find iri_segway_rmp400`/ \
			 -D iriutils_FOUND=TRUE \
			 -D iriutils_INCLUDE_DIR=`rospack find iriutils`/include/iriutils/ \
			 -D comm_FOUND=TRUE \
			 -D comm_INCLUDE_DIR=`rospack find iricomms`/include/iridrivers/ \
			 -D segway_rmp_200_FOUND=TRUE \
 			 -D segway_rmp_200_INCLUDE_DIR=`rospack find iri_segway_rmp200`/include/iridrivers/ \
			 -D BUILD_DOXYGEN_DOCS=OFF
ifeq ($(UNAME), Darwin)
	CMAKE_ARGS += -D iriutils_LIBRARY=`rospack find iriutils`/lib/iriutils/libiriutils.dylib
	CMAKE_ARGS += -D comm_LIBRARY=`rospack find iricomms`/lib/iridrivers/libcomm.dylib
	CMAKE_ARGS += -D segway_rmp_200_LIBRARY=`rospack find iri_segway_rmp200`/lib/iridrivers/libsegway_rmp_200.dylib
else
	CMAKE_ARGS += -D iriutils_LIBRARY=`rospack find iriutils`/lib/iriutils/libiriutils.so
	CMAKE_ARGS += -D comm_LIBRARY=`rospack find iricomms`/lib/iridrivers/libcomm.so
	CMAKE_ARGS += -D segway_rmp_200_LIBRARY=`rospack find iri_segway_rmp200`/lib/iridrivers/libsegway_rmp_200.so
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
	rm -rf build patched lib include wiped build Modules

.PHONY : clean download