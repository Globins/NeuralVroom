LDIR = ../lib
CC = gcc
CFLAGS = -I$(LDIR)

ODIR = obj

_DEPS = gpioAPI.h
DEPS = $(patsubst %,$(LDIR)/%,$(_DEPS))

_OBJ = stepperMotor.o gpioAPI.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

stepper: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)