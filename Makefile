all: main
	cd packet; make
	cd node; make
	cd host; make
	cd switch; make
	cd event; make
	cd IP; make
	cd entry; make
	cd mysort; make
	cd mylist; make
	cd fattree; make
	make out
	@echo "************[All make done]************";

out:
	g++ -o main main.o packet/*.o node/*.o host/*.o switch/*.o fattree/*.o event/*.o IP/*.o entry/*.o mysort/*.o mylist/*.o

main:
	g++ -c main.cpp

clean:
	rm -f */*.o *.o */*~ *~ */.*~ .*~ main
