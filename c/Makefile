# 三个构型的独立运行器：链接 codegen/<构型> 下生成的代码，不依赖 MATLAB。
# Windows 用 mingw32-make，Linux/macOS 用 make。

ifeq ($(origin CC),default)
CC := gcc
endif
CFLAGS  ?= -O2 -std=c89 -Wall -I. -I../codegen/quad -I../codegen/fw -I../codegen/tilt
LDFLAGS ?= -lm

MISSIONS := quad fw tilt

all: $(addprefix build/uav_,$(addsuffix _runner,$(MISSIONS)))

define mission_rules
build/$(1)_main.o: main.c | build
	$(CC) $(CFLAGS) -DMISSION_$(1) -c $$< -o $$@

build/$(1)_%.o: ../codegen/$(1)/%.c | build
	$(CC) $(CFLAGS) -c $$< -o $$@

CG_$(1) := $$(patsubst ../codegen/$(1)/%.c,build/$(1)_%.o,$$(wildcard ../codegen/$(1)/*.c))

build/uav_$(1)_runner: $$(CG_$(1)) build/$(1)_main.o
	$(CC) $$^ -o $$@ $(LDFLAGS)
endef

$(foreach m,$(MISSIONS),$(eval $(call mission_rules,$(m))))

build:
	@mkdir -p build

run: all
	@mkdir -p out
	./build/uav_quad_runner out/quad.csv
	./build/uav_fw_runner out/fw.csv
	./build/uav_tilt_runner out/tilt.csv

clean:
	rm -rf build out

.PHONY: all run clean
