OS:=$(shell uname -s)

CC=gcc
CFLAGS=-c 
LFLAGS  = -L"../iniparser3.0b"
common_libs=-lm -liniparser
linux_libs=-lm -lpthread -lrt
qnx_libs=-lsocket
INCLUDES=-I"../include/" -I"include/" -I"../tsg/include" -I"../iniparser3.0b/src/"
SOURCES=main.c client_handler.c log_handler.c timeout_handler.c status_handler.c settings_handler.c dio_handler.c timing_handler.c coordination_handler.c dummy_handler.c reciever_handler.c dds_handler.c ../utils.c ../tsg/maketsg.c ../_tcpsocket.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=arby_server
INI_TEST=ini_test
INI_SOURCES=ini_test.c 
INI_OBJECTS=$(INI_SOURCES:.c=.o)

all: $(SOURCES) $(EXECUTABLE)

ini: $(INI_SOURCES) $(INI_TEST)

$(EXECUTABLE): $(OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(OBJECTS) $(LFAGS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(OBJECTS) $(LFLAGS) $(common_libs) $(qnx_libs) 
endif

$(INI_TEST): $(INI_OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(INI_OBJECTS) $(LFAGS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(INI_OBJECTS) $(LFLAGS) $(common_libs) $(qnx_libs) 
endif

.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(LFAGS) $(INCLUDES)

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE) $(INI_OBJECTS) $(INI_TEST) 



