section .text
    global _start

_start:
    mov rax, 0x5
    int 0x80
    mov rbx, rax
    mov rax, 0x3
    int 0x80
    mov rax, 0x1
    int 0x80