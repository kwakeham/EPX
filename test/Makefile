# Host (PC) build of the EPX control logic for software-in-the-loop testing.
# No nRF SDK involved: PID_controller, motor_sm and gears are SDK-free and are
# compiled straight from ../libraries.

CC      ?= cc
CFLAGS  ?= -O2 -g -Wall -Wextra -std=c11
LIBS    := ../libraries
SRC     := test_loop.c sim_plant.c \
           $(LIBS)/PID_controller.c $(LIBS)/motor_sm.c $(LIBS)/gears.c \
           $(LIBS)/derailleur.c $(LIBS)/shift_seq.c
OUT     := sil

.PHONY: all run clean
all: $(OUT)

$(OUT): $(SRC)
	$(CC) $(CFLAGS) -I. -I$(LIBS) $(SRC) -lm -o $(OUT)

# Run unit asserts (stderr) and write the CSV time-series to run.csv (stdout).
run: $(OUT)
	./$(OUT) > run.csv

clean:
	rm -f $(OUT) run.csv
