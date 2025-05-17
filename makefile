simulation: main.o drone.o utils.o report.o collision.o
	gcc main.o drone.o utils.o report.o collision.o -lm -o simulation

main.o: main.c drone.h utils.h report.h collision.h
	gcc -g -Wall -Wextra -fanalyzer -c main.c -o main.o

drone.o: drone.c drone.h
	gcc -g -Wall -Wextra -fanalyzer -c drone.c -o drone.o

utils.o: utils.c utils.h
	gcc -g -Wall -Wextra -fanalyzer -c utils.c -o utils.o

report.o: report.c report.h
	gcc -g -Wall -Wextra -fanalyzer -c report.c -o report.o

collision.o: collision.c collision.h
	gcc -g -Wall -Wextra -fanalyzer -c collision.c -o collision.o

clean:
	rm -f *.o simulation

run: simulation
	./simulation $(ARGS)
