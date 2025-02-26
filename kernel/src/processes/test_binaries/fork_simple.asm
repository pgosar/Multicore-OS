section .text
    global _start

_start:
    mov rax, 0x5
    int 0x80
    mov rdi, rax
    mov rax, 0x1
    int 0x80