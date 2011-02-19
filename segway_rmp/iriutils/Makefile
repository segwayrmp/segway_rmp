all: installed

# TARBALL     = build/utilities-svn-r73.tar.gz
# TARBALL_URL = http://williamjwoodall.com/files/utilities-svn-r73.tar.gz
# UNPACK_CMD  = tar xfj
# SOURCE_DIR  = build/iriutils-svn
# TARBALL_PATCH = iriutils_ros.patch
# # MD5SUM_FILE = eigen2-2.0.15.tar.bz2.md5sum
# 
# include $(shell rospack find mk)/download_unpack_build.mk

SVN_DIR = build/utilities-svn
SVN_REVISION = -r 73 --trust-server-cert
# SVN_URL = https://haydn.upc.es/labrobotica/pub/applications/utilities
SVN_URL = https://haydn.upc.es/labrobotica/pub/drivers/iriutils
SVN_PATCH =    iriutils_ros.patch
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=DEBUG \
			 -D CMAKE_INSTALL_PREFIX=`rospack find iriutils`/ \
			 -D BUILD_DOXYGEN_DOCS=OFF

installed: wiped $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf wiped installed

wiped: Makefile $(SVN_PATCH)
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched lib include Modules

.PHONY : clean wipe
