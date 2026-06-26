# Makefile for mini-mpc-constraint-handling-priority
# MPC Constraint Handling & Priority Management Library

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

SRCS     = $(SRCDIR)/mpc_constraint_defs.c \
           $(SRCDIR)/mpc_constraint_priority.c \
           $(SRCDIR)/mpc_constraint_relaxation.c \
           $(SRCDIR)/mpc_constraint_optimization.c \
           $(SRCDIR)/mpc_constraint_feasibility.c \
           $(SRCDIR)/mpc_constraint_industrial.c

OBJS     = $(SRCS:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
LIB      = $(BUILDDIR)/libmpcconstraint.a
TEST_SRC = $(TESTDIR)/test_mpc_constraints.c
TEST_BIN = $(BUILDDIR)/test_mpc_constraints

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
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcconstraint $(LDFLAGS) -o $@

examples: $(EXAMPLEDIR)/example_input_saturation \
          $(EXAMPLEDIR)/example_output_prioritization \
          $(EXAMPLEDIR)/example_constraint_relaxation

$(EXAMPLEDIR)/example_input_saturation: $(EXAMPLEDIR)/example_input_saturation.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcconstraint $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_output_prioritization: $(EXAMPLEDIR)/example_output_prioritization.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcconstraint $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_constraint_relaxation: $(EXAMPLEDIR)/example_constraint_relaxation.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lmpcconstraint $(LDFLAGS) -o $@

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
	@grep -rn "TODO\|FIXME\|stub\|placeholder" $(SRCDIR) $(INCDIR) $(TESTDIR) $(EXAMPLEDIR) 2>/dev/null || echo "  OK: no stubs"

clean:
	rm -rf $(BUILDDIR)
	rm -f $(EXAMPLEDIR)/example_input_saturation
	rm -f $(EXAMPLEDIR)/example_output_prioritization
	rm -f $(EXAMPLEDIR)/example_constraint_relaxation

help:
	@echo "Targets: all, test, examples, count, audit, clean"
