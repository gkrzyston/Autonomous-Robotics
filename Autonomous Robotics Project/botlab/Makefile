include ../common.mk

# flags for building the gtk library
CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_LCM) \
	$(CFLAGS_GTK) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_GSL) \
	-O3

LDFLAGS = $(LDFLAGS_STD) \
      $(LDFLAGS_MATH) \
      $(LDFLAGS_COMMON) \
      $(LDFLAGS_LCMTYPES) \
      $(LDFLAGS_LCM) \
      $(LDFLAGS_GSL) \
      $(LDFLAGS_MAPPING)

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_MAPPING = $(LIB_PATH)/libmapping.a
#LIBMAPPING_OBJS = occupancy_grid.o

BIN_MAIN= $(BIN_PATH)/driver 



ALL =  $(BIN_MAIN)

all: $(ALL)

$(BIN_MAIN): occupancy_grid.o driver.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS) -lcjson -lserialport -lfftw3 $(CXXFLAGS)


 

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
