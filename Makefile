#compiling settings
PROCESSOR	= 16F688
CC		= xc8
PROJ_DIR	= .
BUILD_DIR	= ${PROJ_DIR}/build
BIN_DIR		= ${PROJ_DIR}/bin
OBJ_DIR		= ${PROJ_DIR}/obj
CC_INCLUDE	= -I/Applications/microchip/xc8/v1.34/include/
CC_FLAGS	= -P -N255 --warn=0 --addrqual=ignore --mode=pro# --echo
OPTIMISATION= --opt=default,+asm,-asmfile,+speed,-space,-debug,9 --runtime=+plib
OUTPUT		= --output=intel --outdir=${BUILD_DIR} --objdir=${OBJ_DIR}
OUTFILE		= ut61e-mod.hex

CHIP		= --chip=${PROCESSOR}
ALL_C		= ${PROJ_DIR}/ut61e-mod.c
#programming settings
#PROGRAMMER	= pickit2
PROGRAM		= /Applications/pk2cmd/pk2cmd
DEVICE		= -pPIC${PROCESSOR}
#datasheet stuff
READER		= okular
PREFIX		= PIC
SUFFIX		= .pdf
DIR		= ~/datasheets/IC\'s/Microchip/PIC/

.PHONY : all clean clean_all prog program erase on off info help
.DEFAULT : all
${ALL_C} all:
	${CC} ${CC_FLAGS} ${CHIP} ${OPTIMISATION} ${ALL_C} ${OUTPUT}
	cp ${BUILD_DIR}/${OUTFILE} ${BIN_DIR}/${OUTFILE}
clean:
	rm -f ${BUILD_DIR}/* ${OBJ_DIR}/* ${BIN_DIR}/*
prog program:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -f${BIN_DIR}/${OUTFILE} -m -j
erase:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -e -j
on:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -t
off:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -w
info:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -I
datasheet:
	${READER} ${DIR}/${PREFIX}${PROCESSOR}${SUFFIX}
help:
	@echo "Type 'make' or 'make all' to make the program"
	@echo "Use the 'prog' or 'program' targets to program the device"
	@echo "Use 'erase' to erase the device"
	@echo "Use 'on' and 'off' to control the PicKit's power output"
	@echo "Use 'info' to get info on the device"
	@echo "'clean' cleans temp files"
	@echo "'clean_all' cleans temp and release files"
	@echo "Use 'datasheet' to open datasheet of part"

