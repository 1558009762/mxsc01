#CROSS_COMPILE=arm-xilinx-linux-gnueabi-
CROSS_COMPILE=/opt/arm/usr/bin/arm-linux-

CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar
LDFLAGS :=

OUTPUT := hdlctest
all: ${OUTPUT}

SRCS := $(shell find ./ -name "*.c")
SRCS := ${SRCS}
OBJS := $(addsuffix .o,$(basename ${SRCS}))
#OBJS := $(subst ../,./,${OBJS})

${OUTPUT}: ${OBJS}
	${LD} -o $@ ${LDFLAGS} ${OBJS}
	${CROSS_COMPILE}strip $@

%.o: %.c
	${CC} -std=gnu99 -o $@ -c $^

clean:
	-rm -f ${OBJS} ${OUTPUT}

c:
	-rm -f ${OBJS}
