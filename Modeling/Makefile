CC = c++
FLAGS = -Wall

#-----------------------------------------
#Optimization ----------------------------
OPT = -O2 -g

#-----------------------------------------
#-----------------------------------------

TARGETS = smoothing
OBJECTS = smoothing.o io.o

#-----------------------------------------
# 
INCS = -I/usr/include/eigen3 -I/usr/local/include/eigen3/
LIBS =

CCOPTS = $(OPT) $(FLAGS) $(INCS)  
LDOPTS = $(OPT) $(LIBS)

#-----------------------------------------
#-----------------------------------------

default: $(TARGETS) 

clean:
	/bin/rm -fv *.o $(TARGETS) 

#-----------------------------------------
#-----------------------------------------

smoothing: $(OBJECTS)
	$(CC) $(LDOPTS) -o smoothing $(OBJECTS) -lblas


#-----------------------------------------

.cpp.o:
	$(CC) $(CCOPTS) -c $< -o $@

#-----------------------------------------
#-----------------------------------------
