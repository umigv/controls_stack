CC=g++ -g -Wall -std=c++17

# List of source files for your thread library
SOURCES=dstarlite.cpp node.cpp

# Generate the library's object files
OBJS=${SOURCES:.cpp=.o}


all: lib.o app

# Compile the thread library 
lib.o: ${OBJS}
	ld -r -o $@ ${OBJS}

# Compile an application program
app: /test/test_compilation.cpp lib.o $(OBJS)
	${CC} -o $@ $^ 

# Generic rules for compiling a source file to an object file
%.o: %.cpp
	${CC} -c $<
%.o: %.cc
	${CC} -c $<

clean:
	rm -f ${OBJS} app

