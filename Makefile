.PHONY: all
.DEFAULT: all

all:
	mkdir build
	cmake -Bbuild -H.
	@echo
	@echo "-----------------------------------------------"
	@echo "Flowlessly build system generated, run:"
	@echo " $$ cd build"
	@echo " $$ make"
	@echo "-----------------------------------------------"
