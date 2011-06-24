all: installed

GIT_DIR = build/libsegwayrmp-git
GIT_URL = git://github.com/wjwwood/libsegwayrmp.git
include $(shell rospack find mk)/git_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=DEBUG \
			 -D CMAKE_INSTALL_PREFIX="$(shell rospack find libsegwayrmp)/" \
			 -D CMAKE_FIND_INSTALL_PATH="$(shell rospack find libsegwayrmp)/" \
			 -D serial_FOUND=TRUE \
			 -D serial_INCLUDE_DIRS="$(shell rospack find serial)/include/"
ifeq ($(UNAME), Darwin)
	CMAKE_ARGS += -D serial_LIBRARIES="$(shell rospack find serial)/lib/libserial.dylib"
else
	CMAKE_ARGS += -D serial_LIBRARIES="$(shell rospack find serial)/lib/libserial.a"
endif

installed: $(GIT_DIR)
	mkdir -p lib include Modules
	mkdir -p $(GIT_DIR)/build
	cd $(GIT_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(GIT_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

clean:
	rm -rf build bin lib include installed rospack_nosubdirs Modules