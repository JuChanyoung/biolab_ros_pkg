all: installed

TARBALL = build/ARIA-2.7.2.tgz
TARBALL_URL = http://robots.mobilerobots.com/ARIA/download/archives/ARIA-2.7.2.tgz
SOURCE_DIR = build/Aria-2.7.2
MD5SUM_FILE = ARIA-2.7.2.tgz.md5sum
UNPACK_CMD = tar xzf
TARBALL_PATCH = install_location.patch

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && make clean
	cd $(SOURCE_DIR) && make && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-rm -rf build/Aria-2.7.2 Aria
	rm -f *~ installed

.PHONY : clean wipe

wipe: clean
	rm -f build/ARIA-2.7.2.tgz
