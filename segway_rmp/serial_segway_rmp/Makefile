all: installed

GIT_DIR = build/serial-git
GIT_URL = git://github.com/wjwwood/serial.git
include $(shell rospack find mk)/git_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=DEBUG \
			 -D CMAKE_INSTALL_PREFIX="$(shell rospack find serial_segway_rmp)/" \
			 -D CMAKE_FIND_INSTALL_PATH="$(shell rospack find serial_segway_rmp)/"

installed: $(GIT_DIR)
	mkdir -p lib include Modules
	mkdir -p $(GIT_DIR)/build
	cd $(GIT_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(GIT_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

clean:
	rm -rf build bin lib include installed rospack_nosubdirs Modules