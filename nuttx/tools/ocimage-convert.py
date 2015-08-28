#!/usr/bin/python
# -*- coding: latin-1 -*-
#
#  @file ocimage-convert.py
#  @brief Offcode "ocImage converter"
#

#############################################################################
#
# Copyright (C) 2015, Offcode Ltd. All rights reserved.
# Author: Juho Grundström <juho@offcode.fi>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice,   this list of conditions and the following disclaimer.
#    * Redistributions in  binary form must  reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * The names of the contributors may not be used to endorse or promote
#      products derived from this  software without specific prior written
#      permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND  ANY  EXPRESS  OR  IMPLIED WARRANTIES,  INCLUDING,  BUT NOT LIMITED TO,
# THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY  AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COPYRIGHT OWNER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT,  INDIRECT,  INCIDENTAL,  SPECIAL,  EXEMPLARY, OR
# CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT  LIMITED  TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES;  LOSS OF USE, DATA, OR PROFITS;  OR BUSINESS
# INTERRUPTION)  HOWEVER CAUSED AND  ON ANY THEORY OF LIABILITY,  WHETHER IN
# CONTRACT,  STRICT LIABILITY,  OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#############################################################################

from optparse import OptionParser
import os
import struct
import sys

file_format_identifier = 3737190144
file_format_version = 1

def crc32_32bits(crc, data):
    crc &= 0xffffffff
    data &= 0xffffffff
    crc ^= data
    crc &= 0xffffffff
    for i in range(0,32):
	if crc & 0x80000000:
	    crc = (crc << 1) ^ 0x04C11DB7
	else:
	    crc = (crc << 1)
    crc &= 0xffffffff
    return crc

def calculate_crc_from_binary_data(bindata):
    crc = 0xffffffff
    bindata_length = len(bindata)
    i = 0
    while i < bindata_length:
	num_remaining_bindata_bytes = bindata_length - i;
	num_bytes_to_read = num_remaining_bindata_bytes
	if num_remaining_bindata_bytes > 4:
	    num_bytes_to_read = 4
	temp_bindata = bindata[i:i+num_bytes_to_read]
	if num_bytes_to_read < 4:
	    j = 4 - num_bytes_to_read
	    while j < 4:
		temp_bindata += struct.pack('B', 0)
		j += 1
	value_32bits = struct.unpack('I', temp_bindata[0:4]);
	crc = crc32_32bits(crc, value_32bits[0])
	i = i + 4;
    return crc

def create_ocimage_from_file(output,input):
    try:
	bindata = open(input,'rb').read()
    except:
	print 'Could not read file "%s"' % input
	return -1;
    if len(bindata) % 4 != 0:
        print "File size (%s) not divisible by four." % len(bindata)
        return -1
    bindata_len = len(bindata)
    inverted_bindata_len = (~bindata_len) & 0xFFFFFFFF
    binary_crc = calculate_crc_from_binary_data(bindata)
    version = file_format_version
    bindata += struct.pack('4I', file_format_identifier, version, inverted_bindata_len, binary_crc)
    suffix = bindata[-(4*4):]
    suffix_crc = calculate_crc_from_binary_data(suffix)
    bindata += struct.pack('I', suffix_crc)
    try:
	open(output,'wb').write(bindata)
    except:
	print 'Could not write file "%s"' % output
	return -1;
    return 0

def verify_ocimage_file(file):
    print 'File:              "%s"' % file
    bindata = open(file,'rb').read()
    binary_crc_from_computation = calculate_crc_from_binary_data(bindata[:-(5*4)])
    suffix = bindata[-(5*4):]
    suffix_crc_from_computation = calculate_crc_from_binary_data(suffix[-5*4:-4])
    identifier, version, inverted_length, binary_crc_from_file, suffix_crc_from_file = struct.unpack('5I', suffix)
    length = (~inverted_length) & 0xFFFFFFFF
    if identifier != file_format_identifier:
	print 'Invalid file format identifier!'
	return -1
    print 'ocImage version:   "%d"' % version
    print 'Binary length:     "%d"' % length
    print 'Binary CRC32:      "0x%08x"' % binary_crc_from_file
    print 'Suffix CRC32:      "0x%08x"' % suffix_crc_from_file
    if binary_crc_from_file != binary_crc_from_computation:
	print 'Binary CRC32 mismatch!'
	return -1
    if suffix_crc_from_file != suffix_crc_from_computation:
	print 'Suffix CRC32 mismatch!'
	return -1
    return 0

if __name__=="__main__":
    usage = """
    %prog input.img
    %prog input.bin output.img"""

    option_parser = OptionParser(usage=usage)
    args = sys.argv
    if len(args)==3:
	input = args[1]
	output = args[2]
	if not os.path.isfile(input):
	    print 'Invalid input file "%s".' % input
	    sys.exit(1)
	if create_ocimage_from_file(output,input) < 0:
	    print 'Could not create ocImage'
	    sys.exit(1)
    elif len(args)==2:
	input = args[1]
	if not os.path.isfile(input):
	    print 'Invalid input file "%s".' % input
	    sys.exit(1)
	if verify_ocimage_file(input) < 0:
	    print 'Not a valid ocImage.'
	    sys.exit(1)
    else:
	option_parser.print_help()
	sys.exit(1)
