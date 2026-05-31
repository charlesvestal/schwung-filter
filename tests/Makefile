CC ?= cc
CFLAGS = -O2 -std=c11 -I../src/dsp -Wall -Wextra
SRCS = measure.c ../src/dsp/svf_core.c ../src/dsp/smoother.c ../src/dsp/modulation.c
measure: $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o measure -lm
clean:
	rm -f measure
.PHONY: clean
