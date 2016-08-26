# RISC-V baremetal init.s
# This code is executed first.

.section .text.init
entry:

    la    sp, __sp-32   # set up the stack pointer, using a constant defined in the linker script.
    call  main          # call the main function
    ecall               # halt the simluation when it returns
end:
	j end               # loop when finished, since there is no environment to return to.
