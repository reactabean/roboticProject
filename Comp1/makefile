
all: testjig

testjig: testjig.o Comp1.o 
	g++ -o testjig testjig.o Comp1.o 

testjig.o: testjig.cpp Comp1.o Comp1.h
	g++ -c testjig.cpp

Comp1.o: Comp1.cpp Comp1.h
	g++ -c Comp1.cpp

clean:
	rm -f testjig

clear:
	clear
	clear
	clear 

