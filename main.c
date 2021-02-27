#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>

// unix only
#include <unistd.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>

// struct to hold memory and registers
typedef struct LC3 {
    uint16_t memory[UINT16_MAX];
    uint16_t reg[8];
    uint16_t PC;
    enum {
        FL_POS = 1 << 0,
        FL_ZRO = 1 << 1,
        FL_NEG = 1 << 2
    } COND;
} LC3;

typedef enum {
    OP_BR = 0, // branch
    OP_ADD,    // add
    OP_LD,     // load
    OP_ST,     // store
    OP_JSR,    // jump register
    OP_AND,    // bitwise and
    OP_LDR,    // load register
    OP_STR,    // store register
    OP_RTI,    // unused
    OP_NOT,    // bitwise not
    OP_LDI,    // load indirect
    OP_STI,    // store indirect
    OP_JMP,    // jump
    OP_RES,    // reserve (unused)
    OP_LEA,    // load effective address
    OP_TRAP,   // execute trap
} Opcode;

typedef enum {
    TRAP_GETC = 0x20,  // get char from keyboard, not echoed to term
    TRAP_OUT = 0X21,   // output a char
    TRAP_PUTS = 0X22,  // output a str
    TRAP_IN = 0X23,    // get char from keyboard, echoed to term
    TRAP_PUTSP = 0X24, // output a byte str
    TRAP_HALT = 0X25   // halt prog
} Trapcode;

// platform specific code
uint16_t check_key();
void disable_input_buffering();
void restore_input_buffering();
void handle_interrupt();

// helper functions
void readfile(LC3 *comp, char *file_path);
uint16_t swap16(uint16_t x);
uint16_t sign_extend(int16_t x, int bit_count);
void update_flag(LC3 *comp, uint16_t r);
void mem_write(LC3 *comp, uint16_t address, uint16_t val);
uint16_t mem_read(LC3 *comp, uint16_t address);

// instructions
void op_add(LC3 *comp, uint16_t instr);
void op_and(LC3 *comp, uint16_t instr);
void op_not(LC3 *comp, uint16_t instr);
void op_br(LC3 *comp, uint16_t instr);
void op_jmp(LC3 *comp, uint16_t instr);
void op_jsr(LC3 *comp, uint16_t instr);
void op_ld(LC3 *comp, uint16_t instr);
void op_ldi(LC3 *comp, uint16_t instr);
void op_ldr(LC3 *comp, uint16_t instr);
void op_lea(LC3 *comp, uint16_t instr);
void op_st(LC3 *comp, uint16_t instr);
void op_sti(LC3 *comp, uint16_t instr);
void op_str(LC3 *comp, uint16_t instr);
void op_trap(LC3 *comp, uint16_t instr, int *running);

// for debugging
void print_opcode(uint16_t op);
void print_registers(LC3 *comp);

// emulator functions
LC3 *init_lc3();
void emulate(LC3 *comp);

struct termios original_tio;

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s <filename>\n", argv[0]);
        exit(1);
    }
    
    LC3 *comp = init_lc3();
    readfile(comp, argv[1]);

    // setup
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    emulate(comp);
    restore_input_buffering();

    return 0;
}


uint16_t check_key() {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}
void disable_input_buffering() {
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

void handle_interrupt() {
    restore_input_buffering();
    puts("");
    exit(-2);
}

void readfile(LC3 *comp, char *file_path) {
    FILE *fp = fopen(file_path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "error, could not open %s", file_path);
        exit(1);
    }
    
    uint16_t origin;
    fread(&origin, sizeof(uint16_t), 1, fp);
    origin = swap16(origin);

    uint16_t *addr = comp->memory + origin;
    size_t size = fread(addr, sizeof(uint16_t), UINT16_MAX - origin, fp);

    while (size-- > 0) {
        *addr = swap16(*addr);
        addr++;
    }
    fclose(fp);
}

uint16_t swap16(uint16_t x) {
    return (x << 8) | (x >> 8);
}

uint16_t sign_extend(int16_t x, int bit_count) {
    uint16_t dup = x;
    if ((dup >> bit_count - 1) & 1) dup |= 0xffff << bit_count;
    return dup;
}

void update_flag(LC3 *comp, uint16_t r) {
    if (comp->reg[r] == 0) {
        comp->COND = FL_ZRO;
    } else if (comp->reg[r] >> 15) {
        comp->COND = FL_NEG;
    } else {
        comp->COND = FL_POS;
    }
}

void mem_write(LC3 *comp, uint16_t address, uint16_t val) {
    comp->memory[address] = val; 
}

uint16_t mem_read(LC3 *comp, uint16_t address) {
    // memory mapped registers
    enum {MR_KBSR = 0xfe00, MR_KBDR = 0xfe02};

    if (address == MR_KBSR) {
        if (check_key()) {
            comp->memory[MR_KBSR] = 1 << 15;
            comp->memory[MR_KBDR] = getchar();
        } else {
            comp->memory[MR_KBSR] = 0;
        }
    }
    return comp->memory[address];
}

void op_add(LC3 *comp, uint16_t instr) {
    uint16_t DR = (instr >> 0x9) & 0x7;
    uint16_t SR1 = (instr >> 0x6) & 0x7;
    uint16_t mode = (instr >> 0x5) & 0x1;

    if (mode) {
        uint16_t imm5 = sign_extend(instr & 0x1f, 5);
        // printf("imm5 = %d\n", imm5);
        comp->reg[DR] = comp->reg[SR1] + imm5;
    } else {
        uint16_t SR2 = instr & 0x7; 
        // printf("sr2 = %d\n", SR2);
        comp->reg[DR] = comp->reg[SR1] + comp->reg[SR2];
    }
    update_flag(comp, DR); 
}

void op_and(LC3 *comp, uint16_t instr) {
    uint16_t DR = (instr >> 0x9) & 0x7;
    uint16_t SR1 = (instr >> 0x6) & 0x7;
    uint16_t mode = (instr >> 0x5) & 0x1;

    if (mode == 0x0) {
        uint16_t SR2 = instr & 0x7; 
        comp->reg[DR] = comp->reg[SR1] & comp->reg[SR2];
    } else {
        uint16_t imm5 = sign_extend(instr & 0x1f, 5);
        comp->reg[DR] = comp->reg[SR1] & imm5;
    }
    update_flag(comp, DR); 
}

void op_not(LC3 *comp, uint16_t instr) {
    uint16_t DR = (instr >> 0x9) & 0x7;
    uint16_t SR = (instr >> 0x6) & 0x7;
    comp->reg[DR] = ~(comp->reg[SR]);
    update_flag(comp, DR);
}

void op_br(LC3 *comp, uint16_t instr) {
    uint16_t nzp = (instr >> 0x9) & 0x7;
    uint16_t pc_offset9 = sign_extend(instr & 0x1ff, 9);
    if (nzp & comp->COND) comp->PC += pc_offset9;
}

void op_jmp(LC3 *comp, uint16_t instr) {
    uint16_t baser = (instr >> 0x6) & 0x7;
    comp->PC = comp->reg[baser];
}

void op_jsr(LC3 *comp, uint16_t instr) {
    uint16_t mode = (instr >> 0xb) & 0x1;
    comp->reg[7] = comp->PC;
    if (mode == 0x1) {
        uint16_t pc_offset11 = sign_extend(instr & 0x7ff, 11);
        comp->PC += pc_offset11;
    } else {
        uint16_t baser = (instr >> 0x6) & 0x7;
        comp->PC = comp->reg[baser];
    }
}

void op_ld(LC3 *comp, uint16_t instr) {
    uint16_t DR = (instr >> 0x9) & 0x7;
    uint16_t pc_offset9 = sign_extend(instr & 0x1ff, 9);
    comp->reg[DR] = mem_read(comp, comp->PC + pc_offset9);
    update_flag(comp, DR);
}

void op_ldi(LC3 *comp, uint16_t instr) {
    uint16_t DR = (instr >> 0x9) & 0x7;
    uint16_t pc_offset9 = sign_extend(instr & 0x1ff, 9);
    comp->reg[DR] = mem_read(comp, mem_read(comp, comp->PC + pc_offset9));
    update_flag(comp, DR);
}

void op_ldr(LC3 *comp, uint16_t instr) {
    uint16_t DR = (instr >> 0x9) & 0x7;
    uint16_t baser = (instr >> 0x6) & 0x7;
    uint16_t pc_offset6 = sign_extend(instr & 0x3f, 6);
    comp->reg[DR] = mem_read(comp, comp->reg[baser] + pc_offset6);
    update_flag(comp, DR);
}

void op_lea(LC3 *comp, uint16_t instr) {
    uint16_t DR = (instr >> 0x9) & 0x7;
    uint16_t pc_offset9 = sign_extend(instr & 0x1ff, 9);
    comp->reg[DR] = comp->PC + pc_offset9;
    update_flag(comp, DR);
}

void op_st(LC3 *comp, uint16_t instr) {
    uint16_t SR = (instr >> 0x9) & 0x7;
    uint16_t pc_offset9 = sign_extend(instr & 0x1ff, 9);
    mem_write(comp, comp->PC + pc_offset9, comp->reg[SR]);
}

void op_sti(LC3 *comp, uint16_t instr) {
    uint16_t SR = (instr >> 0x9) & 0x7;
    uint16_t pc_offset9 = sign_extend(instr & 0x1ff, 9);
    mem_write(comp, mem_read(comp, comp->PC + pc_offset9), comp->reg[SR]);
}

void op_str(LC3 *comp, uint16_t instr) {
    uint16_t SR = (instr >> 0x9) & 0x7;
    uint16_t baser = (instr >> 0x6) & 0x7;
    uint16_t pc_offset6 = sign_extend(instr & 0x3f, 6);
    mem_write(comp, comp->reg[baser] + pc_offset6, comp->reg[SR]);
}

void op_trap(LC3 *comp, uint16_t instr, int *running) {
    switch (instr & 0xff) {
        case TRAP_GETC:
            comp->reg[0] = (uint16_t)getchar();
            break;
        case TRAP_OUT:
            putc((char)comp->reg[0], stdout);
            fflush(stdout);
            break;
        case TRAP_PUTS:
            {
                uint16_t* c = comp->memory + comp->reg[0];
                while (*c) {
                    putc((char)*c, stdout);
                    ++c;
                }
                fflush(stdout);
            }
            break;
        case TRAP_IN:
            printf("enter a character: ");
            char c = getchar();
            putc(c, stdout);
            comp->reg[0] = (uint16_t)c;
            break;
        case TRAP_PUTSP:
            {
                uint16_t *c = comp->memory + comp->reg[0];
                while (*c) {
                    char char1 = *c & 0xff, char2 = *c >> 0x8;
                    putc(char1, stdout);
                    if (char2) putc(char2, stdout);
                    c++;
                }
                fflush(stdout);
            }
            break;
        case TRAP_HALT:
            puts("HALT");
            fflush(stdout);
            *running = 0; 
            break;
    }
}

void print_opcode(uint16_t op) {
    switch (op) {
        case OP_ADD: puts("ADD"); break;
        case OP_AND: puts("AND"); break;
        case OP_NOT: puts("NOT"); break;
        case OP_BR: puts("BR"); break;
        case OP_JMP: puts("JMP"); break;
        case OP_JSR: puts("JSR"); break;
        case OP_LD: puts("LD"); break;
        case OP_LDI: puts("LDI"); break;
        case OP_LDR: puts("LDR"); break;
        case OP_LEA: puts("LEA"); break;
        case OP_ST: puts("ST"); break;
        case OP_STI: puts("STI"); break;
        case OP_STR: puts("STR"); break;
        case OP_TRAP: puts("TRAP"); break;
        case OP_RES: puts("RES"); break;
        case OP_RTI: puts("RTI"); break;
        default: puts("FUCK");
    }
} 

void print_registers(LC3 *comp) {
    for (int i = 0; i < 8; i++) {
        printf("REG[%d] %d\n", i, comp->reg[i]);
    }
    printf("PC = %d COND = %d\n", comp->PC, comp->COND);
}

LC3 *init_lc3() {
    LC3 *comp = malloc(sizeof(LC3));
    comp->PC = 0x3000;
    return comp;
}

void emulate(LC3 *comp) {
    int running = 1;
    while (running) {
        uint16_t instr = mem_read(comp, comp->PC);
        uint16_t op = instr >> 12; 
        comp->PC++;

        switch (op) {
            case OP_ADD: op_add(comp, instr); break;
            case OP_AND: op_and(comp, instr); break;
            case OP_NOT: op_not(comp, instr); break;
            case OP_BR: op_br(comp, instr); break;
            case OP_JMP: op_jmp(comp, instr); break;
            case OP_JSR: op_jsr(comp, instr); break;
            case OP_LD: op_ld(comp, instr); break;
            case OP_LDI: op_ldi(comp, instr); break;
            case OP_LDR: op_ldr(comp, instr); break;
            case OP_LEA: op_lea(comp, instr); break;
            case OP_ST: op_st(comp, instr); break;
            case OP_STI: op_sti(comp, instr); break;
            case OP_STR: op_str(comp, instr); break;
            case OP_TRAP: op_trap(comp, instr, &running); break;
            case OP_RES:
            case OP_RTI:
            default:
                abort();
        }
    }
}

