PYTHON ?= python3
CYTHON ?= cython

PYVERSION := $(shell $(PYTHON) -c "import sys; print(sys.version)")
PYPREFIX := $(shell $(PYTHON) -c "import sys; print(sys.prefix)")
INCDIR := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_python_inc())")
PLATINCDIR := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_python_inc(plat_specific=True))")
LIBDIR1 := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
LIBDIR2 := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_config_var('LIBPL'))")
PYLIB := $(shell $(PYTHON) -c "from distutils import sysconfig; print(sysconfig.get_config_var('LIBRARY')[3:-2])")
CC := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('CC'))")
LINKCC := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('LINKCC'))")
LINKFORSHARED := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('LINKFORSHARED'))")
LIBS := $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('LIBS'))")
SYSLIBS :=  $(shell $(PYTHON) -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('SYSLIBS'))")

CFLAGS := -fPIC -O2 -shared

BUILD_DIR := build
BUILD_LIB := $(BUILD_DIR)/libpyloader.so
BUILD_PYLOADER_O := $(BUILD_DIR)/pyloader.o
BUILD_MYMODULE_O := $(BUILD_DIR)/mymodule.o
BUILD_MYMODULE_C := $(BUILD_DIR)/mymodule.c
BUILD_PYTHON_PATHS_H := $(BUILD_DIR)/python_paths.h

.PHONY: all clean info python_paths.h

all: $(BUILD_DIR) $(BUILD_LIB)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_LIB): $(BUILD_PYLOADER_O) $(BUILD_MYMODULE_O)
	$(LINKCC) -o $@ $^ -L$(LIBDIR1) -L$(LIBDIR2) -l$(PYLIB) -Wl,-rpath,$(LIBDIR1) -Wl,-rpath,$(LIBDIR2) $(LIBS) $(SYSLIBS) $(CFLAGS) $(LINKFORSHARED)

$(BUILD_PYLOADER_O): pyloader.c $(BUILD_PYTHON_PATHS_H)
	$(CC) $(CFLAGS) -I$(INCDIR) -I$(PLATINCDIR) -c -o $@ pyloader.c

$(BUILD_MYMODULE_C): mymodule.pyx
	$(CYTHON) -o $@ $<

$(BUILD_MYMODULE_O): $(BUILD_MYMODULE_C)
	$(CC) $(CFLAGS) -I$(INCDIR) -I$(PLATINCDIR) -c -o $@ $<

$(BUILD_PYTHON_PATHS_H):
	@echo "// Auto-generated Python path config" > $@
	@echo "#define PYTHON_HOME L\"$(PYPREFIX)\"" >> $@
	@echo "static const wchar_t *PYTHON_PATHS[] = {" >> $@
	@$(PYTHON) -c 'import sys; print("\n".join([f"    L\"{p}\"," for p in sys.path]))' >> $@
	@echo "    NULL" >> $@
	@echo "};" >> $@
	@echo "Generated $@"

info:
	@echo "PYTHON=$(PYTHON)"
	@echo "PYVERSION=$(PYVERSION)"
	@echo "PYPREFIX=$(PYPREFIX)"
	@echo "INCDIR=$(INCDIR)"
	@echo "PLATINCDIR=$(PLATINCDIR)"
	@echo "LIBDIR1=$(LIBDIR1)"
	@echo "LIBDIR2=$(LIBDIR2)"
	@echo "PYLIB=$(PYLIB)"
	@echo "CC=$(CC)"
	@echo "LINKCC=$(LINKCC)"
	@echo "LINKFORSHARED=$(LINKFORSHARED)"
	@echo "LIBS=$(LIBS)"
	@echo "SYSLIBS=$(SYSLIBS)"

clean:
	rm -f $(BUILD_DIR)
	rm -f $(BUILD_LIB)
	rm -f $(BUILD_PYLOADER_O)
	rm -f $(BUILD_MYMODULE_O)
	rm -f $(BUILD_MYMODULE_C)
	rm -f $(BUILD_PYTHON_PATHS_H)
