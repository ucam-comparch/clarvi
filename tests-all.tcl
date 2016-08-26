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


source test-compile.tcl

set colour_green "\033\[0;32m"
set colour_red   "\033\[0;31m"
set colour_cyan  "\033\[0;36m"
set colour_reset "\033\[0m"

# allow the test environment to be provided as an argument to vsim
set test_env [lsearch -inline $argv "-Gtest_env=*" ]
if { $test_env ne "" } {
    set e [ string range $test_env [ expr [ string first "=" $test_env ]+1 ] end ]
    set search_string "*-$e-*.mem.txt"
} else {
    set search_string "*.mem.txt"
}


onbreak {resume}

foreach testMem [ lsort [ glob -dir tests/build $search_string ] ] {

    set testBase [ lindex [ split $testMem "." ] 0 ]

    echo "$colour_cyan @ Test: ${testBase} $colour_reset"

    vsim work.clarvi_sim -GINIT_FILE=$testMem -quiet -t ns -voptargs=+acc=npr
    run 1ms

    if { [ runStatus ] eq "break"} {
        set reg28 [ examine -decimal {registers[28]} ]
        set cycles [ examine -decimal cycles ]
        set test [ expr $reg28 >> 1 ]
        if { $reg28 eq "1" } {
            echo "$colour_green @ PASS in $cycles cycles $colour_reset"
        } else {
            echo "$colour_red @ FAIL on test $test $colour_reset"
        }
    } elseif { [ runStatus ] eq "ready" } {
        echo "$colour_red @ FAIL tests did not terminate $colour_reset"
    }
}

quit
