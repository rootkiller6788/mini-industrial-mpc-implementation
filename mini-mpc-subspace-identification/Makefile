# Makefile for mini-mpc-subspace-identification
# Subspace State-Space System Identification (4SID) Library
# N4SID, MOESP, CVA algorithms with industrial MPC model generation

CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -Wpedantic -O2 -g -D__USE_MINGW_ANSI_STDIO=1
LDFLAGS  = -lm
AR       = ar
ARFLAGS  = rcs

INCDIR   = include
SRCDIR   = src
TESTDIR  = tests
EXAMPLEDIR = examples
BUILDDIR = build

SRCS     = $(SRCDIR)/ssid_defs.c \
           $(SRCDIR)/ssid_hankel.c \
           $(SRCDIR)/ssid_projection.c \
           $(SRCDIR)/ssid_svd.c \
           $(SRCDIR)/ssid_n4sid.c \
           $(SRCDIR)/ssid_validation.c

OBJS     = $(SRCS:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
LIB      = $(BUILDDIR)/libssid.a
TEST_SRC = $(TESTDIR)/test_ssid_core.c
TEST_BIN = $(BUILDDIR)/test_ssid_core

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
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lssid $(LDFLAGS) -o $@

examples: $(EXAMPLEDIR)/example_2nd_order \
          $(EXAMPLEDIR)/example_cstr_reactor \
          $(EXAMPLEDIR)/example_order_selection

$(EXAMPLEDIR)/example_2nd_order: $(EXAMPLEDIR)/example_2nd_order.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lssid $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_cstr_reactor: $(EXAMPLEDIR)/example_cstr_reactor.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lssid $(LDFLAGS) -o $@

$(EXAMPLEDIR)/example_order_selection: $(EXAMPLEDIR)/example_order_selection.c $(LIB)
	$(CC) $(CFLAGS) -I$(INCDIR) $< -L$(BUILDDIR) -lssid $(LDFLAGS) -o $@

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
	rm -f $(EXAMPLEDIR)/example_2nd_order
	rm -f $(EXAMPLEDIR)/example_cstr_reactor
	rm -f $(EXAMPLEDIR)/example_order_selection

help:
	@echo "Targets: all, test, examples, count, audit, clean"
