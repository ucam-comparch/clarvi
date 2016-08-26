# Copyright (c) 2016, Robert Eady
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# Convert the Yarvi .txt format to Intel .hex

import sys

argc = len(sys.argv)
if argc != 3 and argc != 4:
    sys.exit("Syntax: %s <infile> <outfile> [<address resolution>]" % sys.argv[0])

addr_resolution = int(sys.argv[3]) if argc == 4 else 1

def txt2hex(txtfile, hexfile):
    address = 0
    EOF = ':00000001FF'
    recordtype = '00'
    for line in txtfile:
        line_stripped = line.rstrip('\r\n')
        numbytes = len(line_stripped)/2
        data = line_stripped.upper()
        # the inner part of the line to be checksummed
        hexline = format(numbytes, '02X') + format(address, '04X') + recordtype + data
        # do the checksum and add colon at the front
        hexline = ':' + hexline + checksum(hexline)
        hexfile.write(hexline+'\n')
        # advance the address
        address += numbytes / addr_resolution
    hexfile.write(EOF+'\n')

def checksum(line):
    sum = 0
    # for each pair of letters (1 byte)
    for i in range(0, len(line), 2):
        byte = line[i:i+2]
        # accumulate the numeric value of the hex byte
        sum += int(byte, 16)
    # take the two's complement of the LSB of the sum and convert back to hex
    return format(((sum ^ 0xff) + 1) % 256, '02X')


# allow "-" as the first argument meaning read from stdin
with (sys.stdin if sys.argv[1] == '-' else open(sys.argv[1], 'r')) as txtfile:
    with open(sys.argv[2], 'w') as hexfile:
        txt2hex(txtfile, hexfile)
