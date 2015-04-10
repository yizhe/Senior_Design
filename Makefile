include ./build_rule.mk

TARGET = lidar
URG_LIB = /usr/local/lib/liburg_c.a
CFLAGS = -O2 $(INCLUDES) -I/usr/local/include/urg_c
LDLIBS = -lm

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET) *.exe

$(TARGET) : lidar.o $(URG_LIB)
            
$(URG_LIB) :
	cd $(@D)/ && $(MAKE) $(@F)
