#!/bin/bash

# exec > /dev/null 2>&1

if ! command -v srec_cat &> /dev/null
then
    echo "<srec_cat> could not be found, installing..."
    echo "use command: ( sudo apt-get install -y srecord ) to install it"
    exit
else
	echo "srecord installed already..."
fi
pwd
echo "Computing CRC"
echo "-------------------------------------"
# "Batch script for generating CRC in SW4STM32 project"
# "Must be placed at SW4STM32/<TARGET> folder"

# "Path configuration"
TARGET_NAME='nucleol053_classB_cubeide'
BYTE_SWAP=1
COMPARE_HEX=1
CRC_ADDR_FROM_MAP=1
# "Not used when CRC_ADDR_FROM_MAP=1"
CRC_ADDR=0x08009000

echo "Derived configuration"
MAP_FILE="${TARGET_NAME}.map"
INPUT_BIN="${TARGET_NAME}.bin -binary"
INPUT_HEX="${TARGET_NAME}.hex -intel"
OUTPUT_HEX="${TARGET_NAME}_CRC.hex -intel"
TMP_FILE='crc_tmp_file.txt'

if [[ "${CRC_ADDR_FROM_MAP}" == "1" ]]; then
	echo "Extract CRC address from ${MAP_FILE} file"
	echo "-----------------------------------------------------------"
	echo "Load line with checksum location to crc_search variable"
	echo "Extracting CRC address from MAP file"
	grep "(_Check_Sum" $MAP_FILE>$TMP_FILE
	CRC_SEARCH=`cat $TMP_FILE`
	rm $TMP_FILE
	echo "print value of CRC_SEARCH variable: ${CRC_SEARCH}"
	echo "get first word at line, which should be CRC address in HEX format"
	grep -oP '(?<=0x).{16}' <<< "$s"
	[[ $CRC_SEARCH =~ 0x([[:xdigit:]]+) ]]
	echo ${BASH_REMATCH[1]}
	CRC_ADDR=0x"${BASH_REMATCH[1]}"
	echo "-----------------------------------------------------------"
	echo "End of CRC address extraction"
fi
	echo "print value of CRC_ADDR variable: ${CRC_ADDR}"

echo "Converting BIN to HEX with offset"
echo "INPUT_BIN = ${INPUT_BIN}"

du -h  ${TARGET_NAME}.bin
srec_cat \
	${INPUT_BIN} \
	-offset 0x08000000 \
	-o ${INPUT_HEX}
du -h ${TARGET_NAME}.hex
echo "Converted BIN to HEX with offset" 

# Compute CRC and store it to new HEX file
if [[ "${BYTE_SWAP}"!="1" ]]; then
# ECHO to see what is going on
echo srec_cat \
	${INPUT_HEX} \
	-crop 0x08000000 ${CRC_ADDR} \
	-byte_swap 4 \
	-stm32-b-e ${CRC_ADDR} \
	-byte_swap 4 \
	-o $TMP_FILE -intel	
srec_cat \
	${INPUT_HEX} \
	-crop 0x08000000 ${CRC_ADDR} \
	-byte_swap 4 \
	-stm32-b-e ${CRC_ADDR} \
	-byte_swap 4 \
	-o $TMP_FILE -intel	
else 
# ECHO to see what is going on
echo srec_cat \
	${INPUT_HEX} \
	-crop 0x08000000 ${CRC_ADDR} \
	-stm32-l-e ${CRC_ADDR} \
	-o $TMP_FILE -intel
srec_cat \
	${INPUT_HEX} \
	-crop 0x08000000 ${CRC_ADDR} \
	-stm32-l-e ${CRC_ADDR} \
	-o $TMP_FILE -intel
fi

du -h ${TMP_FILE}

echo srec_cat \
	${INPUT_HEX} -exclude -within $TMP_FILE -intel \
	$TMP_FILE -intel \
	-o $OUTPUT_HEX

srec_cat \
	${INPUT_HEX} -exclude -within $TMP_FILE -intel \
	$TMP_FILE -intel \
	-o $OUTPUT_HEX
du -h ${TARGET_NAME}_CRC.hex
echo "Delete temporary file"
rm $TMP_FILE
echo "Modified HEX file with CRC stored at ${OUTPUT_HEX}"

echo "Compare input HEX file with output HEX file"
if [[ "${COMPARE_HEX}"=="1" ]]; then  
echo "Comparing ${INPUT_HEX} with ${OUTPUT_HEX}"
srec_cmp \
	 ${INPUT_HEX} ${OUTPUT_HEX} -v
fi

rm ${TARGET_NAME}.hex
mv ${TARGET_NAME}_CRC.hex ${TARGET_NAME}.hex

echo "-------------------------------------"
