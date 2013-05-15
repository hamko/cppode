PRODUCT = a.out 
OBJS = sample01.o

all: $(PRODUCT)

SUFIXES: .cpp .o
.cpp.o:
	g++ -O2 -Wall -g -c -I. -I/usr/local/include -I/usr/X11R6/include -DdTRIMESH_ENABLED -DdDOUBLE $< -L/usr/local/lib -L/usr/X11R6/lib -L/usr/

$(PRODUCT): $(OBJS)
	g++ -I -O2 -Wall -g -o $@ $< -L/usr/local/lib -L/usr/X11R6/lib -L/usr/X11R6/lib -I. -I/usr/local/include -I/usr/X11R6/include -lm -lode -ldrawstuff -lX11 -lglut -lGLU -lGL

.PHONY: clean
clean:
	rm $(PRODUCT)
	rm $(OBJS)
