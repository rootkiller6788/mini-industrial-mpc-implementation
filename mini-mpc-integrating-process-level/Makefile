CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -I./include
LDFLAGS = -lm
SRCDIR = src
INCDIR = include
TESTDIR = tests
EXAMDIR = examples
BUILDDIR = build
SRCS = $(wildcard $(SRCDIR)/*.c)
OBJS = $(patsubst $(SRCDIR)/%.c,$(BUILDDIR)/%.o,$(SRCS))

.PHONY: default
default: $(OBJS)
	@echo All source files compiled successfully.
	@echo Run make test to run the test suite.

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/%.o: $(SRCDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: test
test: $(BUILDDIR)/test_mpc_level
	./$(BUILDDIR)/test_mpc_level

$(BUILDDIR)/test_mpc_level: $(TESTDIR)/test_mpc_level.c $(SRCS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -o $@ $(TESTDIR)/test_mpc_level.c $(SRCS) $(LDFLAGS)

.PHONY: examples
examples: $(BUILDDIR)/dmc_demo $(BUILDDIR)/gpc_vs_dmc $(BUILDDIR)/surge_averaging

$(BUILDDIR)/dmc_demo: $(EXAMDIR)/dmc_level_control_demo.c $(SRCS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -o $@ $< $(SRCS) $(LDFLAGS)

$(BUILDDIR)/gpc_vs_dmc: $(EXAMDIR)/gpc_vs_dmc_comparison.c $(SRCS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -o $@ $< $(SRCS) $(LDFLAGS)

$(BUILDDIR)/surge_averaging: $(EXAMDIR)/surge_tank_averaging_demo.c $(SRCS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -o $@ $< $(SRCS) $(LDFLAGS)

.PHONY: demos
demos: $(BUILDDIR)/surge_visual

$(BUILDDIR)/surge_visual: demos/mpc_surge_visual.c $(SRCS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -o $@ $< $(SRCS) $(LDFLAGS)

.PHONY: bench
bench: $(BUILDDIR)/bench_qp

$(BUILDDIR)/bench_qp: benches/bench_mpc_qp.c $(SRCS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -o $@ $< $(SRCS) $(LDFLAGS)

.PHONY: all
all: test examples

.PHONY: lines
lines:
	@echo include/ lines:
	@find $(INCDIR) -name '*.h' -exec cat {} + | wc -l
	@echo src/ lines:
	@find $(SRCDIR) -name '*.c' -exec cat {} + | wc -l
	@echo Total include+src:
	@find $(INCDIR) $(SRCDIR) \( -name '*.h' -o -name '*.c' \) -exec cat {} + | wc -l

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)

.PHONY: help
help:
	@echo Targets: default test examples demos bench all lines clean
