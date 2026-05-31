CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
export CC
export AR

all:
		$(CC) -g -o stmloader -I./ \
				loader.c \
				serial.c \
				stmbootloader.c \
				-Wall

test:
		$(CC) -g -o test_invariant_stmbootloader \
				../../tests/test_invariant_stmbootloader.c \
				-Wall \
				$(shell pkg-config --cflags --libs check) \
				-lm
		./test_invariant_stmbootloader

clean:
		rm -f stmloader test_invariant_stmbootloader; rm -rf stmloader.dSYM
