WRKDIR := ${workspacePath}
QPDIR := $(WRKDIR)/ThirdParty/qpOASES-3.2.1
EIGENDIR := $(WRKDIR)/ThirdParty/eigen
RBDLDIR := $(WRKDIR)/ThirdParty/rbdl
MJDIR   := $(WRKDIR)/ThirdParty/mjpro150
HEADDIR := $(WRKDIR)/RLProject
SRCDIR := $(HEADDIR)/src
CASSIEDIR := $(SRCDIR)/Cassie2d
OUTDIR := $(HEADDIR)/bin

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
OBJS = $(CASSIEDIR)/Cassie2d.o \
	$(SRCDIR)/DynamicModel.o \
	$(SRCDIR)/DynamicState.o \
        $(SRCDIR)/CassieVis.o \
	$(SRCDIR)/OSC_RBDL.o

# libraries
LIBS = -lm -L$(MJDIR)/bin -lmujoco150 -lglew -lGL -l:libglfw.so.3 -L$(RBDLDIR)/build -lrbdl -L$(QPDIR)/bin -lqpOASES $(QPDIR)/src/LAPACKReplacement.o $(QPDIR)/src/BLASReplacement.o

# Additional flags for compilation (e.g., include flags)
INCL = -I$(CASSIEDIR) -I$(SRCDIR) -I$(EIGENDIR) -I$(QPDIR)/include -I$(MJDIR)/include -I$(RBDLDIR)/src -I$(RBDLDIR)/include -I$(RBDLDIR)/build/include

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -std=c++11 -O3 -pipe -DNDEBUG -Wl,-rpath,$(QPDIR)/bin,-rpath,$(RBDLDIR)/build,-rpath,$(MJDIR)/bin -Wparentheses -Wreturn-type -Wpointer-arith -Wwrite-strings -Wno-unknown-pragmas -Wno-long-long -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_NONE -D__NO_COPYRIGHT__ 

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: test

.SUFFIXES: .cpp .c .o .obj

test: $(OBJS)
	bla=;\
	for file in $(OBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) -shared $(CXXFLAGS) $(INCL) -o $(OUTDIR)/libcassie2d.so $$bla $(LIBS)

main: $(OBJS)
	bla=;\
	for file in $(OBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) -shared $(CXXFLAGS) $(INCL) -o $(OUTDIR)/libcassie2d.so $$bla $(LIBS)
	$(CXX) $(CXXFLAGS) -Wl,-rpath,../bin main.cpp $(INCL) -o $(OUTDIR)/test $$bla $(LIBS) -L../bin -lcassie2d

clean:
	rm -rf $(OUTDIR)/libcassie2d.so $(OBJS) $(FOBJS)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
