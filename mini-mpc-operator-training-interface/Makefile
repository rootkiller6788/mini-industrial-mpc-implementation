# Makefile for mini-mpc-operator-training-interface
# MPC Operator Training Simulator Library

CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -Wpedantic -O2 -g
LDFLAGS  = -lm
AR       = ar
ARFLAGS  = rcs

INCDIR   = include
SRCDIR   = src
TESTDIR  = tests
EXAMPLEDIR = examples
BUILDDIR = build

SRCS     = $(SRCDIR)/mpc_ots_defs.c \
           $(SRCDIR)/mpc_ots_scenario.c \
           $(SRCDIR)/mpc_ots_assessment.c \
           $(SRCDIR)/mpc_ots_interface.c \
           $(SRCDIR)/mpc_ots_guidance.c \
           $(SRCDIR)/mpc_ots_adaptive.c \
           $(SRCDIR)/mpc_ots_industrial.c

OBJS     = $(SRCS:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
LIB      = $(BUILDDIR)/libmpcots.a
TEST_SRC = $(TESTDIR)/test_mpc_ots.c
TEST_BIN = $(BUILDDIR)/test_mpc_ots

.PHONY: all clean test examples count audit

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
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcots $(LDFLAGS) -o $@

examples: $(EXAMPLEDIR)/example_reactor_training \
          $(EXAMPLEDIR)/example_column_training \
          $(EXAMPLEDIR)/example_whatif_analysis

$(EXAMPLEDIR)/example_reactor_training: $(EXAMPLEDIR)/example_reactor_training.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcots $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_column_training: $(EXAMPLEDIR)/example_column_training.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcots $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_whatif_analysis: $(EXAMPLEDIR)/example_whatif_analysis.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcots $(LDFLAGS) -o $@

count:
	@echo "include/ (.h) lines:"
	@cat $(INCDIR)/*.h 2>/dev/null | wc -l
	@echo "src/ (.c) lines:"
	@cat $(SRCDIR)/*.c 2>/dev/null | wc -l
	@echo "Total (.h + .c):"
	@cat $(INCDIR)/*.h $(SRCDIR)/*.c 2>/dev/null | wc -l
	@echo "Lean 4 (.lean) lines (not counted):"
	@cat $(SRCDIR)/*.lean 2>/dev/null | wc -l

audit:
	@echo "=== Filler Detection ==="
	@grep -rn "_fn[0-9]" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no _fnN"
	@grep -rn "_aux[0-9]" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no _auxN"
	@grep -rn "TODO\|FIXME\|stub\|placeholder" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no stubs"
	@grep -rn "algorithm variant" $(SRCDIR) 2>/dev/null || echo "  OK: no algorithm variant filler"
	@grep -rn "extension point" $(SRCDIR) 2>/dev/null || echo "  OK: no extension point filler"
	@echo "=== Lean 4 Filler Detection ==="
	@grep -rn "SystemMetric" $(SRCDIR)/*.lean 2>/dev/null || echo "  OK: no SystemMetric cross-file copy"
	@grep -rn "traceability_matrix" $(SRCDIR)/*.lean 2>/dev/null || echo "  OK: no traceability_matrix stubs"
	@grep -rn "\bsorry\b" $(SRCDIR)/*.lean 2>/dev/null || echo "  OK: no sorry"
	@grep -rn "by trivial" $(SRCDIR)/*.lean 2>/dev/null || echo "  OK: trivial usage present"
	@echo "=== Empty File Detection ==="
	@find . -name "*.c" -size -200c -exec echo "SUSPECT: {}" \; 2>/dev/null || echo "  OK: no suspect small files"

clean:
	rm -rf $(BUILDDIR)
	rm -f $(EXAMPLEDIR)/example_reactor_training
	rm -f $(EXAMPLEDIR)/example_column_training
	rm -f $(EXAMPLEDIR)/example_whatif_analysis

help:
	@echo "Targets: all, test, examples, count, audit, clean"
