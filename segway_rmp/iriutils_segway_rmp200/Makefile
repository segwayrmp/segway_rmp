INSTALL_DIR = segway_rmp200

all: installed

SVN_DIR = build/communications-svn
SVN_URL = https://haydn.upc.es/labrobotica/pub/drivers/segway_rmp_200
SVN_PATCH = segway_rmp200_ros.patch
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=DEBUG \
			 -D CMAKE_INSTALL_PREFIX=`rospack find iriutils_segway_rmp200`/$(INSTALL_DIR) \
			 -D BUILD_DOXYGEN_DOCS=OFF

installed: wiped $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	mkdir -p `rospack find iriutils_segway_rmp200`/$(INSTALL_DIR)
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile $(SVN_PATCH)
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched lib

.PHONY : clean download