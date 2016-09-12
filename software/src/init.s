# RISC-V baremetal init.s
# This code is executed first.

.section .text.init
entry:

    la    sp, __sp-32   # set up the stack pointer, using a constant defined in the linker script.

    la    t0, end       # on hardware, ECALL doesn't stop the CPU, so define
                        # a handler to catch the ECALL and spin
    csrrw zero,0x305,t0 # set the address of the handler (CSR 0x305 is the trap handler base register)

    call  main          # call the main function
    ecall               # halt the simluation when it returns

end:
    j end               # loop when finished if there is no environment to return to.
