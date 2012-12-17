#=============================================================================#
# Application Name:
#=============================================================================#
APP=Corblivar

#=============================================================================#
# Define Compiler Executable:
#=============================================================================#
# !!! O3 w/ g++ 4.5.x produces wrong code !!!
# http://gcc.gnu.org/bugzilla/show_bug.cgi?id=48172
#COMPILER	= g++-4.4
COMPILER	= clang++

#=============================================================================#
# Compiler Options:
#=============================================================================#
# warnings
OPT := $(OPT) -Wall -Wextra -Wpointer-arith -Wuninitialized #-Winline -Wshadow 
#
# machine code
# gcc, Intel Pentium 4
#OPT := $(OPT) -mtune=pentium4m -march=pentium4m -mfpmath=sse 
# icc, Core 2 Duo
#OPT = $(OPT) -fast -axT -xT
# gcc, AMD Athlon64
#OPT := $(OPT) -march=athlon64 -mfpmath=sse
# gcc, AMD Athlon64 plus experimental usage of SSE and 387 unit in parallel
#OPT := $(OPT) -march=athlon64 -mfpmath=sse,387
# gcc, Improved version of Intel Pentium4 CPU with 64-bit extensions, MMX, SSE, SSE2 and SSE3 instruction set support.
#OPT := $(OPT) -march=nocona -mfpmath=sse
#
# gprof profiler code
#OPT := $(OPT) -pg
# 32bit binary
#OPT := $(OPT) -m32
## debug symbols
#OPT := $(OPT) -g
## gdb
#OPT := $(OPT) -ggdb3
#
# Runtime Optimization
# !!! O3 w/ g++ 4.5.x produces wrong code !!!
# http://gcc.gnu.org/bugzilla/show_bug.cgi?id=48172
OPT := $(OPT) -O2
# native tuning, since gcc 4.2
OPT := $(OPT) -march=native

#=============================================================================#
# Variables:
#=============================================================================#
BUILD_DIR := build
SRC_DIR := src
LIBS_DIR := libs
SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ := $(SRC:%.cpp=%.o)
OBJ := $(notdir $(OBJ))
OBJ := $(addprefix $(BUILD_DIR)/,$(OBJ))
DEP := $(OBJ:%.o=%.d)

#=============================================================================#
# Library Options:
#=============================================================================#
# GC library
#LIB_GC_DIR := $(LIBS_DIR)/packages/gc6.8
#LIB_GC_SO := $(LIBS_DIR)/lib/libgc.so
# library includes
#OPT := $(OPT) -I$(LIBS_DIR)/include

#=============================================================================#
# Linker Options:
#=============================================================================#
LIBS :=
#LIBS := $(LIB_GC_SO)

#=============================================================================#
# Link Main Executable
#=============================================================================#
all: libs $(APP)

$(APP): $(BUILD_DIR) $(OBJ)
	@echo
	@echo Linking binary
	$(COMPILER) $(OBJ) $(LIBS) $(OPT) -o $@

$(BUILD_DIR):
	@echo
	@echo create build dir
	mkdir -p $(BUILD_DIR)

#=============================================================================#
# Compile Source Code to Object Files
#=============================================================================#
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(SRC_DIR)/$(APP).hpp
	@echo
	@echo General target for $<
	$(COMPILER) -c $(OPT) $< -o $@

#=============================================================================#
# Librarys build
#=============================================================================#
# build GC lib
#libs: $(LIB_GC_SO)
libs:

#$(LIB_GC_SO):
#	@echo
#	@echo Building GC lib
#	cd ./$(LIB_GC_DIR) && ./configure --prefix=$(LIBS_DIR) --disable-threads && make && make check && make install

#=============================================================================#
# Librarys cleanup
#=============================================================================#
cleanlibs:
#	@echo
#	@echo Cleanup GC lib
#	cd ./$(LIB_GC_DIR) && make clean && make uninstall
	
##=============================================================================#
## Doxygen documentation:
##=============================================================================#
#doc: $(SRC) Doxyfile
#	doxygen
#
##=============================================================================#
## Doxygen documentation:
##=============================================================================#
#doclatex: doc
#	cd ./doc/latex && $(MAKE) 
#
##=============================================================================#
## Remove documentation files
##=============================================================================#
#docclean:
#	@echo "Deleting Documentation: rm -rf doc"
#	@rm -rf doc

#=============================================================================#
# Cleanup build
#=============================================================================#
clean:
	@echo "Removing: "
	@echo "		 " $(BUILD_DIR)/* $(APP)
	@rm -f	$(BUILD_DIR)/* $(APP)

#=============================================================================#
# Purge build
#=============================================================================#
purge: clean cleanlibs

#=============================================================================#
# Pack source
#=============================================================================#
release: all
	@echo "Packing Archive"
	@echo "Save as ../$(APP).tar.gz"
	tar -czf ../$(APP).tar.gz . --exclude=libs/lib --exclude=libs/share --exclude=libs/packages --exclude=libs/bin --exclude=build/* --exclude=$(APP)* --exclude=*.out --exclude=*.swp

#=============================================================================#
# Some debugging output
#=============================================================================#
debug:
	@echo -----------------------------------------------------------------------
	@echo - COMPILER: $(COMPILER)
	@echo -----------------------------------------------------------------------
	@echo - SRC  : $(SRC)
	@echo -----------------------------------------------------------------------
	@echo - OBJ    : $(OBJ)
	@echo -----------------------------------------------------------------------
	@echo - DEP    : $(DEP)
	@echo -----------------------------------------------------------------------
