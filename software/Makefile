# the name of the program we're going to build
PROJECT = program
# build directory
BUILD = build

# name of the Quartus program
FPGA_PROJECT_PATH = ../quartus
FPGA_PROJECT = clarvi

# source files
C_FILES := $(wildcard src/*.c)
AS_FILES := $(wildcard src/*.s)
ASP_FILES := $(wildcard src/*.S)

# object files
C_OBJ_FILES := $(patsubst src/%.c,build/%.o,$(C_FILES))
AS_OBJ_FILES := $(patsubst src/%.s,build/%.o,$(AS_FILES))
ASP_OBJ_FILES := $(patsubst src/%.S,build/%.o,$(ASP_FILES))


# settings for the compilers
RISCV_PREFIX = riscv32-unknown-elf-
CC = $(RISCV_PREFIX)gcc
LD = $(RISCV_PREFIX)ld
OBJCOPY = $(RISCV_PREFIX)objcopy
OBJDUMP = $(RISCV_PREFIX)objdump

TARGET_ARCH = -m32 -march=RV32I
CCFLAGS = -O0 $(TARGET_ARCH)

# Lists of rules: the name of a file, followed by the files it depends on.
# When typing 'make' not followed by a rule name, the first rule is followed.  This rule depends on
# everything, causing all the parts to be built

all: dirs $(BUILD)/mem.hex $(BUILD)/$(PROJECT).dump

# how to build a .o file from a .c file
$(BUILD)/%.o: src/%.c
	$(CC) -c -o $@ $< $(CCFLAGS)

# how to build a .o file from a .s file
$(BUILD)/%.o: src/%.s
	$(CC) -c -o $@ $< $(CCFLAGS)

# how to build a .o file from a .S file
$(BUILD)/%.o: src/%.S
	$(CC) -c -o $@ $< $(CCFLAGS)

# link with gcc
$(BUILD)/$(PROJECT).elf: ${AS_OBJ_FILES} ${C_OBJ_FILES} ${ASP_OBJ_FILES}
	$(CC) -o $@ $^ -T link.ld -lgcc -nostdlib

# build an elf executable
$(BUILD)/$(PROJECT).dump: $(BUILD)/$(PROJECT).elf
	$(OBJDUMP) -S -s $< > $@

# extract the binary data from data and text sections to get a binary image of memory
$(BUILD)/mem.bin: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O binary --only-section=.data* --only-section=.text* $< $@

# convert to an ASCII hex file for simulation
$(BUILD)/mem.txt: $(BUILD)/mem.bin
	hexdump -v -e '"%08x\n"' $< > $@

# convert to an Intel HEX file for downloading to the FPGA
$(BUILD)/mem.hex: $(BUILD)/mem.txt
	python txt2hex.py $< $@ 4


# make software project folder
dirs:
	if [ ! -d $(BUILD) ]; then \
		mkdir $(BUILD); \
	fi


# update the memory files inside the FPGA bitfile
update-mem:	all
	cd ${FPGA_PROJECT_PATH} && quartus_cdb ${FPGA_PROJECT} -c ${FPGA_PROJECT} --update_mif
	cd ${FPGA_PROJECT_PATH} && quartus_asm --read_settings_files=on --write_settings_files=off ${FPGA_PROJECT} -c ${FPGA_PROJECT}

# download the bitfile to your board
download:
	cd ${FPGA_PROJECT_PATH} && quartus_pgm -m jtag -o P\;output_files/${FPGA_PROJECT}.sof@2

# build the whole FPGA from the command line
# not that it's harder to spot warnings in this output compared with the GUI
build_fpga:
	cd ${FPGA_PROJECT_PATH} && quartus_sh --flow compile ${FPGA_PROJECT}

# 'clean' rule: delete all the files so we can start afresh
clean:
	rm -rf $(BUILD)
