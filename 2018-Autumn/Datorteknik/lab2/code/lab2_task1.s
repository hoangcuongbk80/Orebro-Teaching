.data
string: .asciz "\n%d + %d = %d \n"
.text
.global main
.extern printf
main:
push {ip, lr}
mov r1, #2
mov r2, #3
add r3, r1, r2
ldr r0, = string
bl printf
pop {ip, pc}
