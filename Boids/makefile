CC          = c++ 

#-----------------------------------------
#Optimization ----------------------------
OPT   = -O3 -Wno-deprecated

GL_LIB = -framework OpenGL -framework GLUT -framework foundation
#GL_LIB = -lGL -lglut -lGLU

#-----------------------------------------

TARGETS = fishtank viewer

OBJECTS = kdTree.o

#-----------------------------------------

LIBS = $(GL_LIB) 
INCS = -I/usr/local/include/eigen3 -I/usr/include/eigen3

CCOPTS = $(OPT) $(INCS) 
LDOPTS = $(OPT) $(INCS) $(LIBS) 

#-----------------------------------------
#-----------------------------------------

default: $(TARGETS)


clean:
	/bin/rm -f *.o $(TARGETS)

#-----------------------------------------
#-----------------------------------------

viewer: $(OBJECTS) viewer.o
	$(CC) $(OBJECTS) viewer.o $(LDOPTS) $(GL_LIB) -o viewer

fishtank: $(OBJECTS) fishtank.o
	$(CC) $(OBJECTS) fishtank.o $(LDOPTS) -o fishtank

#-----------------------------------------
#-----------------------------------------

.cpp.o: 
	$(CC) $(CCOPTS) -c $< 

#-----------------------------------------
#-----------------------------------------















