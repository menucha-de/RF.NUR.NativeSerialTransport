CC ?= gcc
CFLAGS=-std=c99 -Itarget/include -I$(JDK_INCLUDE) -I$(JDK_INCLUDE)/linux -O3 -Wall -fmessage-length=0 -fPIC -MMD -MP
LDFLAGS=-shared
SOURCES=src/com_nordicid_nativeserial_NativeSerialTransport.c
TARGET=target/libNativeSerialTransport.so
OBJS=$(SOURCES:.c=.o)

ifeq ($(shell uname -m), armv7l)
	ARCH ?= armhf
else
	ARCH ?= amd64
endif

JDK_INCLUDE=$(JAVA_HOME)/include

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) $(TARGET) src/*.d
