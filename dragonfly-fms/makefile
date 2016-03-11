CC=arm-linux-gnueabihf-g++
#CC=g++
CFLAGS=-c -g -Wall -pthread -std=c++11 
LDFLAGS=
SOURCES=main.cpp charger.cpp flightcontrol.cpp com.cpp#flightmanager.cpp 
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=flightmanager

all: $(SOURCES) $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
