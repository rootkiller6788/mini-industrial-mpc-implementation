# Makefile for mini-mpc-performance-monitoring-kpi
# MPC Performance Monitoring & KPI Library

CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -Wpedantic -O2 -g
LDFLAGS  = -lm
AR       = ar
ARFLAGS  = rcs

INCDIR   = include
SRCDIR   = src
TESTDIR  = tests
EXAMPLEDIR = examples
DEMODIR  = demos
BENCHDIR = benches
BUILDDIR = build

SRCS     = $(SRCDIR)/mpc_kpi_defs.c            $(SRCDIR)/mpc_kpi_metrics.c            $(SRCDIR)/mpc_kpi_monitoring.c            $(SRCDIR)/mpc_kpi_diagnosis.c            $(SRCDIR)/mpc_kpi_industrial.c

OBJS     = $(SRCS:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
LIB      = $(BUILDDIR)/libmpckpi.a
TEST_SRC = $(TESTDIR)/test_mpc_kpi.c
TEST_BIN = $(BUILDDIR)/test_mpc_kpi

DEMO_SRC = $(DEMODIR)/kpi_dashboard_demo.c
DEMO_BIN = $(BUILDDIR)/kpi_dashboard_demo

BENCH_SRC = $(BENCHDIR)/bench_kpi.c
BENCH_BIN = $(BUILDDIR)/bench_kpi

.PHONY: all clean test examples demo bench count audit

all: $(LIB) test examples

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/%.o: $(SRCDIR)/%.c | $(BUILDDIR)
	$(CC) $(CFLAGS) -I$(INCDIR) -c $< -o $@

$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $^

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(TEST_SRC) $(LIB) | $(BUILDDIR)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpckpi $(LDFLAGS) -o $@

examples: $(EXAMPLEDIR)/example_harris_index           $(EXAMPLEDIR)/example_kpi_baselining           $(EXAMPLEDIR)/example_kpi_dashboard

$(EXAMPLEDIR)/example_harris_index: $(EXAMPLEDIR)/example_harris_index.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpckpi $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_kpi_baselining: $(EXAMPLEDIR)/example_kpi_baselining.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpckpi $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_kpi_dashboard: $(EXAMPLEDIR)/example_kpi_dashboard.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpckpi $(LDFLAGS) -o $@

demo: $(DEMO_BIN)
	./$(DEMO_BIN)

$(DEMO_BIN): $(DEMO_SRC) $(LIB) | $(BUILDDIR)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpckpi $(LDFLAGS) -o $@

bench: $(BENCH_BIN)
	./$(BENCH_BIN)

$(BENCH_BIN): $(BENCH_SRC) $(LIB) | $(BUILDDIR)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpckpi $(LDFLAGS) -o $@

count:
	@echo "include/ (.h) lines:"
	@cat $(INCDIR)/*.h | wc -l
	@echo "src/ (.c) lines:"
	@cat $(SRCDIR)/*.c | wc -l
	@echo "Total (.h + .c):"
	@cat $(INCDIR)/*.h $(SRCDIR)/*.c | wc -l

audit:
	@echo "=== Filler Detection ==="
	@grep -rn "_fn[0-9]" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no _fnN"
	@grep -rn "_aux[0-9]" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no _auxN"
	@grep -rn "_ext[0-9][^a-z]" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no _extN"
	@grep -rn "TODO\|FIXME\|stub\|placeholder" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no stubs"
	@grep -rn "algorithm variant" $(SRCDIR) 2>/dev/null || echo "  OK: no filler"
	@grep -rn "auxiliary function" $(SRCDIR) 2>/dev/null || echo "  OK: no filler"

clean:
	rm -rf $(BUILDDIR)
	rm -f $(EXAMPLEDIR)/example_harris_index
	rm -f $(EXAMPLEDIR)/example_kpi_baselining
	rm -f $(EXAMPLEDIR)/example_kpi_dashboard
	rm -f $(EXAMPLEDIR)/example_harris_index.exe
	rm -f $(EXAMPLEDIR)/example_kpi_baselining.exe
	rm -f $(EXAMPLEDIR)/example_kpi_dashboard.exe

help:
	@echo "Targets: all, test, examples, demo, bench, count, audit, clean"
