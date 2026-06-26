# Makefile for mini-mpc-disturbance-feedforward
# MPC with Disturbance Feedforward Control

CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
INCLUDES = -Iinclude
LDFLAGS = -lm

SRC_DIR = src
TEST_DIR = tests
EXAMPLE_DIR = examples

# Source files
SRCS = $(SRC_DIR)/mpc_common.c \
       $(SRC_DIR)/mpc_model.c \
       $(SRC_DIR)/mpc_qp.c \
       $(SRC_DIR)/mpc_observer.c \
       $(SRC_DIR)/mpc_feedforward.c \
       $(SRC_DIR)/mpc_controller.c \
       $(SRC_DIR)/mpc_constraints.c

OBJS = $(SRCS:.c=.o)

# Test files
TEST_SRCS = $(wildcard $(TEST_DIR)/test_*.c)
TEST_BINS = $(TEST_SRCS:.c=)

# Example files
EXAMPLE_SRCS = $(wildcard $(EXAMPLE_DIR)/example_*.c)
EXAMPLE_BINS = $(EXAMPLE_SRCS:.c=)

.PHONY: all clean test examples

all: libmpc.a

libmpc.a: $(OBJS)
	ar rcs $@ $^

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

test: libmpc.a $(TEST_BINS)
	@for t in $(TEST_BINS); do \
		echo "Running $$t..."; \
		./$$t || exit 1; \
	done
	@echo "All tests passed."

examples: libmpc.a $(EXAMPLE_BINS)

$(TEST_DIR)/%: $(TEST_DIR)/%.c libmpc.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lmpc $(LDFLAGS) -o $@

$(EXAMPLE_DIR)/%: $(EXAMPLE_DIR)/%.c libmpc.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lmpc $(LDFLAGS) -o $@

clean:
	rm -f $(SRC_DIR)/*.o
	rm -f libmpc.a
	rm -f $(TEST_BINS)
	rm -f $(EXAMPLE_BINS)

# Quick compile check (no linking)
check:
	@for src in $(SRCS); do \
		echo "Checking $$src..."; \
		$(CC) $(CFLAGS) $(INCLUDES) -fsyntax-only $$src || exit 1; \
	done
	@echo "All sources compile OK."
