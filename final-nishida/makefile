CFLAGS = -g -I.
COMPILE = g++ $(CFLAGS) -c
LINK = g++ $(CFLAGS)
LIBS = -lGL -lGLU -lglut -lqd -lmpfr

all:	ps4-nishida

ps4-nishida	: ps4-nishida.o kdtree.o point.o acp.o permute.o 
	$(LINK) ps4-nishida.o kdtree.o point.o acp.o permute.o $(LIBS) -o ps4-nishida

acp.o:	acp.cc acp.h
	$(COMPILE) acp.cc

point.o: point.C point.h object.h pv.h acp.h
	$(COMPILE) point.C

permute.o: permute.C permute.h
	$(COMPILE) permute.C

kdtree.o: kdtree.C kdtree.h object.h pv.h acp.h permute.h 
	$(COMPILE) kdtree.C

ps4-nishida.o: ps4-nishida.C
	$(COMPILE) ps4-nishida.C

clean : 
	rm -f *.o *~ ps4-nishida
