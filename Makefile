#=============================================================================#
# Application Name:
#=============================================================================#
APP := Corblivar
AUX := ThermalAnalyzerFitting 3DFP_Parser 3DSTAF_Parser
ALL := Corblivar ThermalAnalyzerFitting

#=============================================================================#
# Define Compiler Executable:
#=============================================================================#
COMPILER	= clang++
#COMPILER	= /opt/clang+llvm-3.2-x86-linux-ubuntu-12.04/bin/clang++
#COMPILER	= g++
#COMPILER	= /opt/intel/bin/icpc

#=============================================================================#
# Compiler Options:
#=============================================================================#
# warnings
OPT := $(OPT) -Wall -Wextra
# C++11
OPT := $(OPT) -std=c++11 -I/usr/include/i386-linux-gnu/c++/4.8
# threading support, requires clang > 3.0
#OPT := $(OPT) -pthread
# OpenMP, requires gcc
#OPT := $(OPT) -fopenmp
# gprof profiler code
#OPT := $(OPT) -pg
# 32bit binary
#OPT := $(OPT) -m32
# debug symbols
OPT := $(OPT) -g
# Runtime Optimization
OPT := $(OPT) -O2
# native tuning, since gcc 4.2
OPT := $(OPT) -march=native

#=============================================================================#
# Variables:
#=============================================================================#
BUILD_DIR := build
SRC_DIR := src
LIBS_DIR := libs
SRC_AUX := src_aux

# derive related variables
SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ := $(SRC:%.cpp=%.o)
OBJ := $(notdir $(OBJ))
OBJ := $(addprefix $(BUILD_DIR)/,$(OBJ))
DEP := $(OBJ:%.o=%.d)
# assume all objects to be required for aux binaries, expect main object
#OBJ_AUX := $(filter-out $(BUILD_DIR)/$(APP).o, $(OBJ))
OBJ_AUX := $(filter-out build/Corblivar.o, $(OBJ))
OBJ_AUX := $(filter-out build/Corblivar.o, $(OBJ))
# variable to monitor changes in aux src
SRC_AUX_ALL := $(wildcard $(SRC_AUX)/*.cpp)

#=============================================================================#
# Library Options:
#=============================================================================#
#OPT := $(OPT) -I$(LIBS_DIR)/include

#=============================================================================#
# Linker Options:
#=============================================================================#
#LIBS := -fopenmp

#=============================================================================#
# Link Main Executable
#=============================================================================#
all: libs $(ALL)

$(APP): $(BUILD_DIR) $(OBJ)
	@echo
	@echo linking main binary
	$(COMPILER) $(OBJ) $(LIBS) -o $@

$(BUILD_DIR):
	@echo
	@echo create build dir
	mkdir -p $(BUILD_DIR)

#=============================================================================#
# Target for auxiliary binaries: each target represents one binary
#=============================================================================#
$(AUX): $(BUILD_DIR) $(SRC_AUX_ALL) $(OBJ_AUX)
	@echo
	@echo compile and link aux binary $@
	$(COMPILER) $(OPT) $(SRC_AUX)/$@.cpp $(OBJ_AUX) -o $@

#=============================================================================#
# Compile Source Code to Object Files
#=============================================================================#

# pull in dependency for compiled objects
# -include performs include w/o warning, aborts for non-existent files
-include $(DEP)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo
	@echo compile and determine dependencies for $<
	$(COMPILER) -c $(OPT) $< -o $@
	$(COMPILER) -MM $(OPT) $< | sed 's/$(basename $(notdir $@))\.o/$(BUILD_DIR)\/$(notdir $@)/g' > $(@:%.o=%.d)

#=============================================================================#
# Librarys build
#=============================================================================#
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
	@echo "removing: $(BUILD_DIR)/* $(APP) $(AUX)"
	@rm -f $(BUILD_DIR)/* $(APP) $(AUX)

#=============================================================================#
# Purge build
#=============================================================================#
purge: clean cleanlibs

#=============================================================================#
# Pack source
#=============================================================================#
release: all
	@echo "packing source archive"
	@echo "save as $(APP).tar.gz"
	tar -czf $(APP).tar.gz . --exclude=libs/lib --exclude=libs/share --exclude=libs/packages --exclude=libs/bin --exclude=build/* --exclude=$(APP)* --exclude=*.out --exclude=*.swp

#=============================================================================#
# Some debugging output
#=============================================================================#
debug:
	@echo -----------------------------------------------------------------------
	@echo - COMPILER: $(COMPILER)
	@echo -----------------------------------------------------------------------
	@echo - APP  : $(APP)
	@echo -----------------------------------------------------------------------
	@echo - SRC  : $(SRC)
	@echo -----------------------------------------------------------------------
	@echo - OBJ    : $(OBJ)
	@echo -----------------------------------------------------------------------
	@echo - DEP    : $(DEP)
	@echo -----------------------------------------------------------------------
	@echo - SRC_AUX    : $(SRC_AUX_ALL)
	@echo -----------------------------------------------------------------------
	@echo - OBJ_AUX    : $(OBJ_AUX)
	@echo -----------------------------------------------------------------------
