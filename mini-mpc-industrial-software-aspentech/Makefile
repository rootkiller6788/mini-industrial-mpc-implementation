CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -O2 -g -Iinclude
AR       = ar
ARFLAGS  = rcs
BUILDDIR = build
SRCS     = $(wildcard src/*.c)
OBJS     = $(SRCS:src/%.c=$(BUILDDIR)/%.o)
DEPS     = $(wildcard include/*.h)
LIBRARY  = $(BUILDDIR)/libmpc_aspen.a

.PHONY: all clean test lib examples demos benches

all: lib test

lib: $(LIBRARY)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(LIBRARY): $(OBJS) | $(BUILDDIR)
	$(AR) $(ARFLAGS) $@ $^

$(BUILDDIR)/%.o: src/%.c $(DEPS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

test: $(LIBRARY)
	$(CC) $(CFLAGS) tests/test_dmc.c src/*.c -lm -o $(BUILDDIR)/test_dmc
	$(BUILDDIR)/test_dmc

examples: $(LIBRARY)
	$(CC) $(CFLAGS) examples/example_dmc_siso.c src/*.c -lm -o $(BUILDDIR)/example_dmc_siso
	$(CC) $(CFLAGS) examples/example_distillation.c src/*.c -lm -o $(BUILDDIR)/example_distillation
	$(CC) $(CFLAGS) examples/example_dmc3_sim.c src/*.c -lm -o $(BUILDDIR)/example_dmc3_sim
	@echo "=== Examples Built ==="

demos: $(LIBRARY)
	$(CC) $(CFLAGS) demos/demo_dmc3_console.c src/*.c -lm -o $(BUILDDIR)/demo_dmc3_console
	@echo "=== Demos Built ==="

benches: $(LIBRARY)
	$(CC) $(CFLAGS) benches/bench_mpc_core.c src/*.c -lm -o $(BUILDDIR)/bench_mpc_core
	@echo "=== Benches Built ==="

clean:
	rm -rf $(BUILDDIR)
