INPUT1=busy_day.in
INPUT2=redundancy.in
INPUT3=mother_of_all_warehouses.in

INPUT=$(INPUT3)

a.out: main.cpp
	g++ main.cpp -O3 -Wall -std=c++0x -Wextra -Wno-sign-compare

run: a.out
	./a.out $(INPUT)

run-all: a.out
	./a.out $(INPUT1)
	./a.out $(INPUT3)
	./a.out $(INPUT2)

clean:
	rm -f a.out*

clean-all: clean
	rm -f sol-*
