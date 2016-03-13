CC=g++-4.9
CFLAGS=-c -Wall -Wextra -Werror -std=c++14
LDFLAGS= -letk
SOURCES=$(wildcard *.cpp)
OBJECTS=$(patsubst %.cpp,.obj/%.o,$(SOURCES)) 
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) -lc $(OBJECTS) -o $@ $(LDFLAGS)
	./main

.obj/%.o:%.cpp
	$(CC) $(CFLAGS) $< -o $@



clean:
	find . -name \*.o -execdir rm {} \;
	rm -f $(EXECUTABLE)


