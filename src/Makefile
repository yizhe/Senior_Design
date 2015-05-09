include ./build_rule.mk

TARGET = demo
INCLUDES = -I/usr/local/include/urg_c
URG_LIB = /usr/local/lib/liburg_c.a
CFLAGS = -O2 -w $(INCLUDES) 
LDLIBS = -lm

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET) 

$(TARGET) : demo.o $(URG_LIB)
            
$(URG_LIB) :
	cd $(@D)/ && $(MAKE) $(@F)
