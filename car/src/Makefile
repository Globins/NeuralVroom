LDIR = ../include
CC = gcc
CFLAGS = -I$(LDIR) -Wall -pthread -lpigpio -lcurl -lrt 

LIBS = -lm
ODIR = obj

_DEPS = wheelPair.h stepperMotor.h obstacleGrid.h tof.h vehicle.h
DEPS = $(patsubst %,$(LDIR)/%,$(_DEPS))

_OBJ = stepperMotor.o wheelPair.o tof.o obstacleGrid.o vehicle.o main.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 