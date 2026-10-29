# 独立 C 运行器：链接 codegen/ 下生成的代码，不依赖 MATLAB。
# Windows 用 mingw32-make，Linux/macOS 用 make。

ifeq ($(origin CC),default)
CC := gcc
endif
CFLAGS  ?= -O2 -std=c89 -Wall -I. -I../codegen
LDFLAGS ?= -lm

SRC_CG  := $(wildcard ../codegen/*.c)
OBJ     := $(patsubst ../codegen/%.c,build/%.o,$(SRC_CG)) build/main.o
BIN     := build/uav_quad_runner

.PHONY: all clean run

all: $(BIN)

build:
	@mkdir -p build

build/%.o: ../codegen/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/main.o: main.c | build
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

run: $(BIN)
	@mkdir -p out
	./$(BIN)

clean:
	rm -rf build out
