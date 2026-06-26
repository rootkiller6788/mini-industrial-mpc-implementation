CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -I include
LDFLAGS = -lm
AR = ar
ARFLAGS = rcs

SRC_DIR = src
TEST_DIR = tests
EXAMPLE_DIR = examples
BENCH_DIR = benches

SRCS = src/mpc_illcond_condition.c src/mpc_illcond_matrix.c \
       src/mpc_illcond_regularization.c src/mpc_illcond_preconditioner.c \
       src/mpc_illcond_sensitivity.c src/mpc_illcond_svd.c \
       src/mpc_illcond_qp.c
OBJS = $(SRCS:.c=.o)
LIB = libmpc_illcond.a

TEST_SRCS = tests/test_illcond_core.c tests/test_illcond_solver.c
TEST_EXES = $(TEST_SRCS:.c=.exe)

EXAMPLE_SRCS = examples/example_highpurity_column.c \
               examples/example_stiff_reactor.c \
               examples/example_collinear_sensors.c
EXAMPLE_EXES = $(EXAMPLE_SRCS:.c=.exe)

.PHONY: all clean test examples

all: $(LIB) test

$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $^

src/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

test: $(TEST_EXES)
	@for t in $(TEST_EXES); do echo "Running $$t..."; ./$$t || exit 1; done

tests/%.exe: tests/%.c $(LIB)
	$(CC) $(CFLAGS) $< -L. -lmpc_illcond -o $@ $(LDFLAGS)

examples: $(EXAMPLE_EXES)
	@for e in $(EXAMPLE_EXES); do echo "Running $$e..."; ./$$e || exit 1; done

examples/%.exe: examples/%.c $(LIB)
	$(CC) $(CFLAGS) $< -L. -lmpc_illcond -o $@ $(LDFLAGS)

clean:
	rm -f $(OBJS) $(LIB) tests/*.exe examples/*.exe benches/*.exe
