# 1 "program/motor_FOC.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 393 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "program/motor_FOC.c" 2

# 1 "program\\motor_FOC.h" 1




# 1 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 1 3
# 53 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
    typedef unsigned int size_t;
# 68 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
    typedef __builtin_va_list __va_list;
# 87 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
typedef struct __fpos_t_struct {
    unsigned long long int __pos;





    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
# 108 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
typedef struct __FILE FILE;
# 119 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
struct __FILE {
    union {
        long __FILE_alignment;



        char __FILE_size[84];

    } __FILE_opaque;
};
# 138 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;
# 224 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int remove(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int rename(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 243 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *tmpfile(void);






extern __attribute__((__nothrow__)) char *tmpnam(char * );
# 265 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fclose(FILE * ) __attribute__((__nonnull__(1)));
# 275 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fflush(FILE * );
# 285 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *fopen(const char * __restrict ,
                           const char * __restrict ) __attribute__((__nonnull__(1,2)));
# 329 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *freopen(const char * __restrict ,
                    const char * __restrict ,
                    FILE * __restrict ) __attribute__((__nonnull__(2,3)));
# 342 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void setbuf(FILE * __restrict ,
                    char * __restrict ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) int setvbuf(FILE * __restrict ,
                   char * __restrict ,
                   int , size_t ) __attribute__((__nonnull__(1)));
# 370 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int fprintf(FILE * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 393 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int _fprintf(FILE * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));





#pragma __printf_args
extern __attribute__((__nothrow__)) int printf(const char * __restrict , ...) __attribute__((__nonnull__(1)));






#pragma __printf_args
extern __attribute__((__nothrow__)) int _printf(const char * __restrict , ...) __attribute__((__nonnull__(1)));





#pragma __printf_args
extern __attribute__((__nothrow__)) int sprintf(char * __restrict , const char * __restrict , ...) __attribute__((__nonnull__(1,2)));








#pragma __printf_args
extern __attribute__((__nothrow__)) int _sprintf(char * __restrict , const char * __restrict , ...) __attribute__((__nonnull__(1,2)));






#pragma __printf_args
extern __attribute__((__nothrow__)) int __ARM_snprintf(char * __restrict , size_t ,
                     const char * __restrict , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __attribute__((__nothrow__)) int snprintf(char * __restrict , size_t ,
                     const char * __restrict , ...) __attribute__((__nonnull__(3)));
# 460 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int _snprintf(char * __restrict , size_t ,
                      const char * __restrict , ...) __attribute__((__nonnull__(3)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int fscanf(FILE * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 503 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
#pragma __scanf_args
extern __attribute__((__nothrow__)) int _fscanf(FILE * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int scanf(const char * __restrict , ...) __attribute__((__nonnull__(1)));








#pragma __scanf_args
extern __attribute__((__nothrow__)) int _scanf(const char * __restrict , ...) __attribute__((__nonnull__(1)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int sscanf(const char * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 541 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
#pragma __scanf_args
extern __attribute__((__nothrow__)) int _sscanf(const char * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int vfscanf(FILE * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int vscanf(const char * __restrict , __va_list) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) int vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));

extern __attribute__((__nothrow__)) int _vfscanf(FILE * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int _vscanf(const char * __restrict , __va_list) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) int _vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int __ARM_vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));

extern __attribute__((__nothrow__)) int vprintf(const char * __restrict , __va_list ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int _vprintf(const char * __restrict , __va_list ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) int vfprintf(FILE * __restrict ,
                    const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));
# 584 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int vsprintf(char * __restrict ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));
# 594 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int __ARM_vsnprintf(char * __restrict , size_t ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));

extern __attribute__((__nothrow__)) int vsnprintf(char * __restrict , size_t ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));
# 609 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int _vsprintf(char * __restrict ,
                      const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));





extern __attribute__((__nothrow__)) int _vfprintf(FILE * __restrict ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));





extern __attribute__((__nothrow__)) int _vsnprintf(char * __restrict , size_t ,
                      const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));
# 635 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int __ARM_asprintf(char ** , const char * __restrict , ...) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) int __ARM_vasprintf(char ** , const char * __restrict , __va_list ) __attribute__((__nonnull__(2)));
# 649 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fgetc(FILE * ) __attribute__((__nonnull__(1)));
# 659 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) char *fgets(char * __restrict , int ,
                    FILE * __restrict ) __attribute__((__nonnull__(1,3)));
# 673 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fputc(int , FILE * ) __attribute__((__nonnull__(2)));
# 683 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fputs(const char * __restrict , FILE * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) int getc(FILE * ) __attribute__((__nonnull__(1)));
# 704 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
    extern __attribute__((__nothrow__)) int (getchar)(void);
# 713 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) char *gets(char * ) __attribute__((__nonnull__(1)));
# 725 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int putc(int , FILE * ) __attribute__((__nonnull__(2)));
# 737 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
    extern __attribute__((__nothrow__)) int (putchar)(int );






extern __attribute__((__nothrow__)) int puts(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int ungetc(int , FILE * ) __attribute__((__nonnull__(2)));
# 778 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t fread(void * __restrict ,
                    size_t , size_t , FILE * __restrict ) __attribute__((__nonnull__(1,4)));
# 794 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t __fread_bytes_avail(void * __restrict ,
                    size_t , FILE * __restrict ) __attribute__((__nonnull__(1,3)));
# 810 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t fwrite(const void * __restrict ,
                    size_t , size_t , FILE * __restrict ) __attribute__((__nonnull__(1,4)));
# 822 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fgetpos(FILE * __restrict , fpos_t * __restrict ) __attribute__((__nonnull__(1,2)));
# 833 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fseek(FILE * , long int , int ) __attribute__((__nonnull__(1)));
# 850 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fsetpos(FILE * __restrict , const fpos_t * __restrict ) __attribute__((__nonnull__(1,2)));
# 863 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) long int ftell(FILE * ) __attribute__((__nonnull__(1)));
# 877 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void rewind(FILE * ) __attribute__((__nonnull__(1)));
# 886 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void clearerr(FILE * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int feof(FILE * ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) int ferror(FILE * ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) void perror(const char * );
# 917 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int _fisatty(FILE * ) __attribute__((__nonnull__(1)));



extern __attribute__((__nothrow__)) void __use_no_semihosting_swi(void);
extern __attribute__((__nothrow__)) void __use_no_semihosting(void);
# 6 "program\\motor_FOC.h" 2
# 1 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 1
# 74 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef enum IRQn
{

    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    SVCall_IRQn = -5,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,


    BOD_IRQn = 0,
    WDT_IRQn = 1,
    USCI1_IRQn = 3,
    GP_IRQn = 4,
    EPWM_IRQn = 5,
    BRAKE0_IRQn = 6,
    BRAKE1_IRQn = 7,
    BPWM0_IRQn = 8,
    BPWM1_IRQn = 9,
    USCI2_IRQn = 11,
    ECAP_IRQn = 15,
    CCAP_IRQn = 16,
    GDMA0_IRQn = 19,
    GDMA1_IRQn = 20,
    HIRCTRIM_IRQn = 21,
    TMR0_IRQn = 22,
    TMR1_IRQn = 23,
    TMR2_IRQn = 24,
    ACMP_IRQn = 26,
    PWRWU_IRQn = 28,
    ADC0_IRQn = 29,
    ADC1_IRQn = 30,
} IRQn_Type;
# 124 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
# 1 "../../Library/CMSIS/Include\\core_cm0.h" 1
# 39 "../../Library/CMSIS/Include\\core_cm0.h" 3





# 1 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdint.h" 1 3
# 56 "C:\\Users\\JPChang0\\AppData\\Local\\Keil_v6\\ARM\\ARMCLANG\\bin\\..\\include\\stdint.h" 3
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int int64_t;


typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef signed short int int_least16_t;
typedef signed int int_least32_t;
typedef signed long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;
typedef unsigned long long int uint_least64_t;




typedef signed int int_fast8_t;
typedef signed int int_fast16_t;
typedef signed int int_fast32_t;
typedef signed long long int int_fast64_t;


typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
typedef unsigned long long int uint_fast64_t;






typedef signed int intptr_t;
typedef unsigned int uintptr_t;



typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
# 45 "../../Library/CMSIS/Include\\core_cm0.h" 2 3
# 163 "../../Library/CMSIS/Include\\core_cm0.h" 3
# 1 "../../Library/CMSIS/Include\\core_cmInstr.h" 1 3
# 39 "../../Library/CMSIS/Include\\core_cmInstr.h" 3
# 57 "../../Library/CMSIS/Include\\core_cmInstr.h" 3
# 1 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 1 3
# 50 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__((always_inline)) static __inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}







__attribute__((always_inline)) static __inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 102 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 126 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
# 156 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}
# 186 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}
# 216 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_PSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, psp" : "=r" (result) );
  return(result);
}
# 246 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : "sp");
}
# 270 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_MSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return(result);
}
# 300 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : "sp");
}
# 324 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 354 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 836 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline int32_t __REVSH(int32_t value)
{
  int32_t result;

  __asm volatile ("revsh %0, %1" : "=l" (result) : "l" (value) );
  return(result);
}
# 852 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  return (op1 >> op2) | (op1 << (32U - op2));
}
# 875 "../../Library/CMSIS/Include\\cmsis_armcc_V6.h" 3
__attribute__((always_inline)) static __inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;




  int32_t s = 4 * 8 - 1;

  result = value;
  for (value >>= 1U; value; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;

  return(result);
}
# 58 "../../Library/CMSIS/Include\\core_cmInstr.h" 2 3
# 164 "../../Library/CMSIS/Include\\core_cm0.h" 2 3
# 1 "../../Library/CMSIS/Include\\core_cmFunc.h" 1 3
# 39 "../../Library/CMSIS/Include\\core_cmFunc.h" 3
# 165 "../../Library/CMSIS/Include\\core_cm0.h" 2 3
# 247 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t _reserved0:28;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;
# 277 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;
# 295 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:15;
    uint32_t T:1;
    uint32_t _reserved1:3;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;
# 334 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef union
{
  struct
  {
    uint32_t _reserved0:1;
    uint32_t SPSEL:1;
    uint32_t _reserved1:30;
  } b;
  uint32_t w;
} CONTROL_Type;
# 362 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef struct
{
  volatile uint32_t ISER[1U];
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];
} NVIC_Type;
# 389 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
        uint32_t RESERVED0;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];
  volatile uint32_t SHCSR;
} SCB_Type;
# 496 "../../Library/CMSIS/Include\\core_cm0.h" 3
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 629 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}







static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}
# 653 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}







static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}







static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}
# 688 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}
# 712 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
  else
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
}






static __inline void NVIC_SystemReset(void)
{
  __builtin_arm_dsb(0xF);;

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  __builtin_arm_dsb(0xF);;

  for(;;)
  {
    __builtin_arm_nop();
  }
}
# 769 "../../Library/CMSIS/Include\\core_cm0.h" 3
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (uint32_t)(ticks - 1UL);
  NVIC_SetPriority (SysTick_IRQn, (1UL << 2) - 1UL);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0UL;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) |
                   (1UL << 1U) |
                   (1UL );
  return (0UL);
}
# 125 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/Device/Nuvoton/NM1240/Include\\system_NM1240.h" 1
# 32 "../../Library/Device/Nuvoton/NM1240/Include\\system_NM1240.h"
extern uint32_t __HIRC;
extern uint32_t SystemCoreClock;
extern uint32_t CyclesPerUs;
# 45 "../../Library/Device/Nuvoton/NM1240/Include\\system_NM1240.h"
extern void SystemInit(void);
# 57 "../../Library/Device/Nuvoton/NM1240/Include\\system_NM1240.h"
extern void SystemCoreClockUpdate(void);
# 126 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 148 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 177 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t INT_NMICTL;
    volatile uint32_t INT_IRQSTS;

} INT_T;
# 206 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 525 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile const uint32_t PDID;
    volatile uint32_t RSTSTS;
    volatile uint32_t IPRST0;
    volatile uint32_t IPRST1;
    volatile uint32_t WAIT;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t BODCTL;
    volatile uint32_t IVSCTL;
    volatile const uint32_t RESERVE1[1];
    volatile uint32_t PORCTL;
    volatile const uint32_t RESERVE2[2];
    volatile uint32_t GPA_MFP;
    volatile uint32_t GPB_MFP;
    volatile uint32_t GPC_MFP;
    volatile uint32_t GPD_MFP;
    volatile uint32_t GPE_MFP;
    volatile uint32_t GPF_MFP;
    volatile const uint32_t RESERVE3[46];
    volatile uint32_t REGLCTL;
    volatile const uint32_t RESERVE5[4];
    volatile const uint32_t TSOFFSET;

} SYS_T;
# 816 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 1035 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t PWRCTL;
    volatile uint32_t AHBCLK;
    volatile uint32_t APBCLK;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t CLKSEL0;
    volatile uint32_t CLKSEL1;
    volatile const uint32_t RESERVE1[2];
    volatile uint32_t CLKDIV;
    volatile const uint32_t RESERVE2[11];
    volatile const uint32_t STATUS;
    volatile const uint32_t RESERVE3[3];
    volatile uint32_t CLKOCTL;

} CLK_T;
# 1185 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 2272 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t MODE;
    volatile uint32_t DINOFF;
    volatile uint32_t DOUT;
    volatile uint32_t DATMSK;
    volatile const uint32_t PIN;
    volatile uint32_t DBEN;
    volatile uint32_t INTTYPE;
    volatile uint32_t INTEN;
    volatile uint32_t INTSRC;
    volatile uint32_t SMTEN;
    volatile uint32_t SLEWCTL;
    volatile uint32_t PLEN;
    volatile uint32_t PHEN;
} GPIO_T;


typedef struct
{
# 2322 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t GPIO_DBCTL;
} GPIO_DB_T;
# 2687 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 2838 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CTL;
    volatile uint32_t CMP;
    volatile uint32_t INTSTS;
    volatile const uint32_t CNT;
    volatile const uint32_t CAP;
    volatile uint32_t EXTCTL;
    volatile uint32_t EINTSTS;
} TIMER_T;

typedef struct {
# 2921 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CCAPCTL;
    volatile const uint32_t CCAP[4];

} TIMER_AC_T;
# 3050 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 3287 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CNT;
    volatile uint32_t HLD[3];
    volatile uint32_t CNTCMP;
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t STS;
    volatile uint32_t CTL2;
    volatile const uint32_t RESERVE[1];
    volatile uint32_t TST;

} ECAP_T;
# 3461 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 3527 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CTL;

} WDT_T;
# 3580 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 3961 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CTL;
    volatile uint32_t INTEN;
    volatile uint32_t BRGEN;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t DATIN0;
    volatile const uint32_t RESERVE1[3];
    volatile uint32_t CTLIN0;
    volatile uint32_t RESERVE2[1];
    volatile uint32_t CLKIN;
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile uint32_t BUFCTL;
    volatile uint32_t BUFSTS;
   volatile uint32_t DMACTL;
   volatile const uint32_t RESERVE3[4];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;

} UUART_T;
# 4205 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 4596 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CTL;
    volatile uint32_t INTEN;
    volatile uint32_t BRGEN;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t DATIN0;
    volatile const uint32_t RESERVE1[3];
    volatile uint32_t CTLIN0;
    volatile const uint32_t RESERVE2[1];
    volatile uint32_t CLKIN;
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile uint32_t BUFCTL;
    volatile uint32_t BUFSTS;
   volatile uint32_t DMACTL;
    volatile const uint32_t RESERVE3[4];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;

} USPI_T;
# 4836 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 5169 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CTL;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t BRGEN;
    volatile const uint32_t RESERVE1[8];
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile const uint32_t RESERVE2[1];
  volatile const uint32_t BUFSTS;
    volatile uint32_t DMACTL;
   volatile uint32_t DEVADDR0;
    volatile const uint32_t RESERVE3[1];
    volatile uint32_t ADDRMSK0;
    volatile const uint32_t RESERVE4[1];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;
    volatile const uint32_t RESERVE5[9];
    volatile uint32_t TMCTL;

} UI2C_T;
# 5390 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 5425 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t DIVIDEND;
    volatile uint32_t DIVISOR;
    volatile uint32_t QUOTIENT;
    volatile uint32_t REM;

} HDIV_T;

typedef struct
{
# 5458 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile const uint32_t STATUS;

} HDIV_STS_T;
# 5499 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 6090 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t NPCTL;
    volatile uint32_t CLKDIV;
    volatile uint32_t CTL;
    volatile uint32_t PERIOD;
    volatile const uint32_t RESERVE0[5];
    volatile uint32_t CMPDAT[6];
    volatile const uint32_t CNT;
    volatile const uint32_t RESERVE1[5];
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;
    volatile uint32_t RESDLY;
    volatile uint32_t BRKCTL;
    volatile uint32_t DTCTL;
    volatile const uint32_t RESERVE2[4];
    volatile uint32_t PHCHG;
    volatile uint32_t PHCHGNXT;
    volatile uint32_t PHCHGALT;
    volatile uint32_t IFA;

} EPWM_T;
# 6561 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 6784 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CLKPSC;
    volatile uint32_t CLKDIV;
    volatile uint32_t CTL;
    volatile uint32_t PERIOD0;
    volatile uint32_t CMPDAT0;
    volatile const uint32_t CNT0;
    volatile uint32_t PERIOD1;
    volatile uint32_t CMPDAT1;
    volatile const uint32_t CNT1;
    volatile const uint32_t RESERVE0[7];
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;

} BPWM_T;
# 6913 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 7041 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CTL[1];
    volatile const uint32_t RESERVE0;
    volatile uint32_t STATUS;
    volatile const uint32_t RESERVE1;
    volatile uint32_t TRGDLY;
    volatile uint32_t DACVAL;
    volatile uint32_t DACCTL;

} ACMP_T;
# 7141 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 7356 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
 volatile const uint32_t RESERVE0[4];
 volatile uint32_t CS_CTL;
 volatile const uint32_t RESERVE1[3];
  volatile uint32_t CTL;
  volatile uint32_t TRGSOR;
  volatile uint32_t TRGDLY;
  volatile uint32_t SMPCNT;
  volatile uint32_t STATUS;
 volatile const uint32_t RESERVE2[3];
  volatile uint32_t VALSTS;
 volatile const uint32_t RESERVE3[15];
 volatile const uint32_t ADC0_DAT0;
 volatile const uint32_t ADC0_DAT1;
 volatile const uint32_t ADC0_DAT2;
 volatile const uint32_t ADC0_DAT3;
 volatile const uint32_t ADC0_DAT4;
 volatile const uint32_t ADC0_DAT5;
 volatile const uint32_t ADC0_DAT6;
 volatile const uint32_t ADC0_DAT7;
 volatile const uint32_t ADC0_DAT8;
 volatile const uint32_t ADC0_DAT9;
 volatile const uint32_t ADC0_DAT10;
 volatile const uint32_t ADC0_DAT11;
 volatile const uint32_t ADC0_DAT12;
 volatile const uint32_t ADC0_DAT13;
 volatile const uint32_t ADC0_DAT14;
 volatile const uint32_t ADC0_DAT15;
 volatile const uint32_t ADC1_DAT0;
 volatile const uint32_t ADC1_DAT1;
 volatile const uint32_t ADC1_DAT2;
 volatile const uint32_t ADC1_DAT3;
 volatile const uint32_t ADC1_DAT4;
 volatile const uint32_t ADC1_DAT5;
 volatile const uint32_t ADC1_DAT6;
 volatile const uint32_t ADC1_DAT7;
 volatile const uint32_t ADC1_DAT8;
 volatile const uint32_t ADC1_DAT9;
 volatile const uint32_t ADC1_DAT10;
 volatile const uint32_t ADC1_DAT11;
 volatile const uint32_t ADC1_DAT12;
 volatile const uint32_t ADC1_DAT13;
 volatile const uint32_t ADC1_DAT14;
 volatile const uint32_t ADC1_DAT15;

} ADC_T;
# 7605 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 7743 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t ISPCTL;
    volatile uint32_t ISPADDR;
    volatile uint32_t ISPDAT;
    volatile uint32_t ISPCMD;
    volatile uint32_t ISPTRG;
    volatile const uint32_t DFBA;
    volatile const uint32_t RESERVE0[10];
    volatile uint32_t ISPSTS;
    volatile const uint32_t RESERVE1[3];
    volatile uint32_t CRCSEED;
    volatile const uint32_t CRCCV;

} FMC_T;
# 7830 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 7843 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t CTL;

} OP_T;
# 7869 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef struct
{
# 7984 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
    volatile uint32_t CTL;
    volatile uint32_t SRCB;
    volatile uint32_t DSTB;
    volatile uint32_t TCNT;
    volatile const uint32_t CSRC;
    volatile const uint32_t CDST;
    volatile const uint32_t CTCNT;

} GDMA_T;
# 8160 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
typedef volatile unsigned char vu8;
typedef volatile unsigned short vu16;
typedef volatile unsigned long vu32;
# 8368 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h"
# 1 "../../Library/StdDriver/inc\\sys.h" 1
# 641 "../../Library/StdDriver/inc\\sys.h"
static __inline void SYS_UnlockReg(void)
{
    do{*((volatile uint32_t *)(((( uint32_t)0x50000000) + 0x00000) + 0x100)) = 0x59;*((volatile uint32_t *)(((( uint32_t)0x50000000) + 0x00000) + 0x100)) = 0x16;*((volatile uint32_t *)(((( uint32_t)0x50000000) + 0x00000) + 0x100)) = 0x88;}while(*((volatile uint32_t *)(((( uint32_t)0x50000000) + 0x00000) + 0x100))==0);
}
# 653 "../../Library/StdDriver/inc\\sys.h"
static __inline void SYS_LockReg(void)
{
    *((volatile uint32_t *)(((( uint32_t)0x50000000) + 0x00000) + 0x100)) = 0x00;
}

void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_UnlockReg(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetGDMA(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);
# 8369 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\acmp.h" 1
# 474 "../../Library/StdDriver/inc\\acmp.h"
void ACMP_Open(ACMP_T *, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *, uint32_t u32ChNum);
# 8370 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\adc.h" 1
# 432 "../../Library/StdDriver/inc\\adc.h"
void ADC_Open(ADC_T *adc, uint32_t u32InputMode);
void ADC_Close(ADC_T *adc);
void ADC_ConfigChannel(ADC_T *adc, uint32_t u32Channel, uint32_t u32TriggerSrc);
void ADC_SetTriggerDelayTime(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32TriggerDelayTime);
void ADC_SetSampleCnt(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32SampleCnt);
uint32_t ADC_Get_Data_Valid_Flag(ADC_T *adc, uint32_t u32Channel);
uint32_t ADC_GET_INT_FLAG(ADC_T *adc, uint32_t u32Msk);
# 8371 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\fmc.h" 1
# 92 "../../Library/StdDriver/inc\\fmc.h"
extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_Erase_SPROM(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);

extern void FMC_Open(void);
extern uint32_t FMC_Read (uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern uint32_t FMC_GetVectorPageAddr(void);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_GetCRC32Sum(uint32_t addr, uint32_t count, uint32_t *chksum);
# 8372 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\gpio.h" 1
# 486 "../../Library/StdDriver/inc\\gpio.h"
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);
# 8373 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\epwm.h" 1
# 532 "../../Library/StdDriver/inc\\epwm.h"
uint32_t EPWM_ConfigOutputChannel(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void EPWM_Start(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_Stop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_ForceStop(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableFaultBrake(EPWM_T *epwm, uint32_t u32ChannelMask, uint32_t u32LevelMask, uint32_t u32BrakeSource);
void EPWM_EnableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_DisableOutput(EPWM_T *epwm, uint32_t u32ChannelMask);
void EPWM_EnableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void EPWM_DisableDeadZone(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void EPWM_DisableDutyInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetDutyIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_DisableFaultBrakeInt(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_ClearFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
uint32_t EPWM_GetFaultBrakeIntFlag(EPWM_T *epwm, uint32_t u32BrakeSource);
void EPWM_EnablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType);
void EPWM_DisablePeriodInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetPeriodIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_EnableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_DisableZeroInt(EPWM_T *epwm, uint32_t u32ChannelNum);
void EPWM_ClearZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
uint32_t EPWM_GetZeroIntFlag(EPWM_T *epwm, uint32_t u32ChannelNum);
# 8374 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\bpwm.h" 1
# 165 "../../Library/StdDriver/inc\\bpwm.h"
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequncy,
                                  uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableFaultBrake(BPWM_T *bpwm, uint32_t u32ChannelMask, uint32_t u32LevelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void BPWM_DisableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
# 8375 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\timer.h" 1
# 111 "../../Library/StdDriver/inc\\timer.h"
static __inline void TIMER_ICEDEBUG_HOLD_ENABLE(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (31));
}






static __inline void TIMER_ICEDEBUG_HOLD_DISABLE(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (31));
}






static __inline void TIMER_Reset(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (26));
}






static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (30));
}






static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (30));
}







static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (23));
}






static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (23));
}






static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (7));
}






static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (7));
}






static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (29));
}






static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (29));
}






static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (5));
}






static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (5));
}
# 244 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return(timer->INTSTS & (0x1ul << (0)) ? 1 : 0);
}






static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (0));
}
# 266 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->EINTSTS;
}






static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->EINTSTS = (0x1ul << (0));
}
# 288 "../../Library/StdDriver/inc\\timer.h"
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->INTSTS & (0x1ul << (1)) ? 1 : 0);
}






static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (1));
}






static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}






static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->CNT;
}
# 460 "../../Library/StdDriver/inc\\timer.h"
uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
# 8376 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\wdt.h" 1
# 132 "../../Library/StdDriver/inc\\wdt.h"
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x04000))->CTL = 0;
    return;
}






static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x04000))->CTL |= (0x1ul << (6));
    return;
}






static __inline void WDT_DisableInt(void)
{

    ((WDT_T *) ((( uint32_t)0x40000000) + 0x04000))->CTL &= ~((0x1ul << (6)) | (0x1ul << (2)) | (0x1ul << (3)) | (0x1ul << (5)) | (0x1ul << (12))) ;
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval,
               uint32_t u32ResetDelay,
               uint32_t u32EnableReset,
               uint32_t u32EnableWakeup);
# 8377 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\clk.h" 1
# 140 "../../Library/StdDriver/inc\\clk.h"
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetEXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetPCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_EnableEXTCLK(void);
void CLK_DisableEXTCLK(void);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SysTickDelay(uint32_t us);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_HIRC_Calibration(void);
# 8378 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\op.h" 1
# 8379 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\ecap.h" 1
# 482 "../../Library/StdDriver/inc\\ecap.h"
void ECAP_Open(ECAP_T* ecap, uint32_t u32FuncMask);
void ECAP_Close(ECAP_T *ecap);
void ECAP_EnableINT(ECAP_T* ecap, uint32_t u32Mask);
void ECAP_DisableINT(ECAP_T* ecap, uint32_t u32Mask);
# 8380 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\usci_uart.h" 1
# 419 "../../Library/StdDriver/inc\\usci_uart.h"
void UUART_ClearIntFlag(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_GetIntFlag(UUART_T* uuart, uint32_t u32Mask);
void UUART_Close(UUART_T* uuart);
void UUART_DisableInt(UUART_T* uuart, uint32_t u32Mask);
void UUART_EnableInt(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_Open(UUART_T* uuart, uint32_t u32baudrate);
uint32_t UUART_Read(UUART_T* uuart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
uint32_t UUART_SetLine_Config(UUART_T* uuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
void UUART_SelectLINMode(UUART_T* uuart, uint32_t u32Mode);
uint32_t UUART_Write(UUART_T* uuart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void UUART_EnableWakeup(UUART_T* uuart, uint32_t u32WakeupMode);
void UUART_DisableWakeup(UUART_T* uuart);
# 8381 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\hdiv.h" 1
# 40 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int32_t HDIV0_Div(int32_t x, int16_t y)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVIDEND = x;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVISOR = y;
    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->QUOTIENT;
}
# 55 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int16_t HDIV0_Mod(int32_t x, int16_t y)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVIDEND = x;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVISOR = y;
    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->REM;
}
# 70 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int32_t HDIV1_Div(int32_t x, int16_t y)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14020))->DIVIDEND = x;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14020))->DIVISOR = y;
    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14020))->QUOTIENT;
}
# 85 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int16_t HDIV1_Mod(int32_t x, int16_t y)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14020))->DIVIDEND = x;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14020))->DIVISOR = y;
    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14020))->REM;
}
# 100 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int32_t HDIV2_Div(int32_t x, int16_t y)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14040))->DIVIDEND = x;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14040))->DIVISOR = y;
    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14040))->QUOTIENT;
}
# 115 "../../Library/StdDriver/inc\\hdiv.h"
static __inline int16_t HDIV2_Mod(int32_t x, int16_t y)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14040))->DIVIDEND = x;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14040))->DIVISOR = y;
    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14040))->REM;
}
# 8382 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\usci_i2c.h" 1
# 36 "../../Library/StdDriver/inc\\usci_i2c.h"
enum UI2C_MASTER_EVENT {
                             MASTER_SEND_ADDRESS = 10,
                             MASTER_SEND_H_WR_ADDRESS,
                             MASTER_SEND_H_RD_ADDRESS,
                             MASTER_SEND_L_ADDRESS,
                             MASTER_SEND_DATA,
                             MASTER_SEND_REPEAT_START,
                             MASTER_READ_DATA,
                             MASTER_STOP,
                             MASTER_SEND_START
                           };




enum UI2C_SLAVE_EVENT {
                            SLAVE_ADDRESS_ACK = 100,
                            SLAVE_H_WR_ADDRESS_ACK,
                            SLAVE_L_WR_ADDRESS_ACK,
                            SLAVE_GET_DATA,
                            SLAVE_SEND_DATA,
                            SLAVE_H_RD_ADDRESS_ACK,
                            SLAVE_L_RD_ADDRESS_ACK
                          };
# 447 "../../Library/StdDriver/inc\\usci_i2c.h"
uint32_t UI2C_Open(UI2C_T *ui2c, uint32_t u32BusClock);
void UI2C_Close(UI2C_T *ui2c);
void UI2C_ClearTimeoutFlag(UI2C_T *ui2c);
void UI2C_Trigger(UI2C_T *ui2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Ptrg, uint8_t u8Ack);
void UI2C_DisableInt(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_EnableInt(UI2C_T *ui2c, uint32_t u32Mask);
uint32_t UI2C_GetBusClockFreq(UI2C_T *ui2c);
uint32_t UI2C_SetBusClockFreq(UI2C_T *ui2c, uint32_t u32BusClock);
uint32_t UI2C_GetIntFlag(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_ClearIntFlag(UI2C_T* ui2c , uint32_t u32Mask);
uint32_t UI2C_GetData(UI2C_T *ui2c);
void UI2C_SetData(UI2C_T *ui2c, uint8_t u8Data);
void UI2C_SetSlaveAddr(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddr, uint8_t u8GCMode);
void UI2C_SetSlaveAddrMask(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddrMask);
void UI2C_EnableTimeout(UI2C_T *ui2c, uint32_t u32TimeoutCnt);
void UI2C_DisableTimeout(UI2C_T *ui2c);
void UI2C_EnableWakeup(UI2C_T *ui2c, uint8_t u8WakeupMode);
void UI2C_DisableWakeup(UI2C_T *ui2c);
# 8383 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\usci_spi.h" 1
# 192 "../../Library/StdDriver/inc\\usci_spi.h"
static __inline void USPI_SET_DATA_WIDTH(USPI_T *uspi, uint32_t u32Width)
{
    if(u32Width == 16)
        u32Width = 0;

    uspi->LINECTL = (uspi->LINECTL & ~(0xful << (8))) | (u32Width << (8));
}
# 370 "../../Library/StdDriver/inc\\usci_spi.h"
uint32_t USPI_Open(USPI_T *uspi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void USPI_Close(USPI_T *uspi);
void USPI_ClearRxBuf(USPI_T *uspi);
void USPI_ClearTxBuf(USPI_T *uspi);
void USPI_DisableAutoSS(USPI_T *uspi);
void USPI_EnableAutoSS(USPI_T *uspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t USPI_SetBusClock(USPI_T *uspi, uint32_t u32BusClock);
uint32_t USPI_GetBusClock(USPI_T *uspi);
void USPI_EnableInt(USPI_T *uspi, uint32_t u32Mask);
void USPI_DisableInt(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetIntFlag(USPI_T *uspi, uint32_t u32Mask);
void USPI_ClearIntFlag(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetStatus(USPI_T *uspi, uint32_t u32Mask);
void USPI_EnableWakeup(USPI_T *uspi);
void USPI_DisableWakeup(USPI_T *uspi);
void USPI_DMA_CTL(USPI_T *uspi, uint32_t u32CtrlMask);
# 8384 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 1 "../../Library/StdDriver/inc\\gdma.h" 1
# 187 "../../Library/StdDriver/inc\\gdma.h"
void GDMA_OPEN(GDMA_T* gdma, uint32_t u32ModeMask, uint32_t u32TransferWidth, uint32_t u32BurstMask, uint32_t u32AddressMask);
# 8385 "../../Library/Device/Nuvoton/NM1240/Include\\NM1240.h" 2
# 7 "program\\motor_FOC.h" 2
# 1 "program\\system_initialize.h" 1


# 1 "program\\variable_typedefine.h" 1








# 1 "program\\system_initialize.h" 1
# 10 "program\\variable_typedefine.h" 2
# 1 "program\\system_parameter.h" 1
# 355 "program\\system_parameter.h"
static __inline int32_t HDIV0_DIV(int32_t dividend, int16_t divisor)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVIDEND = dividend;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVISOR = divisor;

    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->QUOTIENT;
}

static __inline int16_t HDIV0_MOD(int32_t dividend, int16_t divisor)
{
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVIDEND = dividend;
    ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->DIVISOR = divisor;

    return ((HDIV_T *) ((( uint32_t)0x50000000) + 0x14000))->REM;
}
# 11 "program\\variable_typedefine.h" 2
# 27 "program\\variable_typedefine.h"
typedef struct tag_MotorSpec
{
int16_t i16_freq_pwm;
int16_t i16_speed_loop_time;
int16_t i16_dead_time;

int16_t i16_speed_slope_cnt_target;
int16_t i16_speed_slope_delta_rpm;

int16_t i16_current_gain;
int16_t i16_Idc_over_current;
int16_t i16_Idc_OC_Q15_max;
int16_t i16_Iphase_OC_Q15_max;

int16_t i16_Vq_maximum;
int16_t i16_Iq_current_limit_in_A;
int16_t i16_Iq_current_limit;
int16_t i16_speed_maximum;





int16_t i16_motor_pole;
}AMotorSpec;

typedef struct tag_MotorInfo
{
int16_t i16_Iu;
int16_t i16_Iv;
int16_t i16_Iw;
int16_t i16_Ialfa;
int16_t i16_Ibeta;
int16_t i16_Id;
int16_t i16_Iq;
int16_t i16_Id_error;
int16_t i16_Iq_error;

int16_t i16_Iu_ref_ADC;
int16_t i16_Iv_ref_ADC;
int16_t i16_Iw_ref_ADC;


int16_t i16_rotor_speed;
int16_t i16_speed_error;
int16_t i16_hall_angle;

int32_t i32_sin_Q15;
int32_t i32_cos_Q15;



uint8_t u8_Hall_Position;
uint8_t u8_Last_Hall_Position;
uint8_t u8_rotor_direction;

int16_t i16_IDC_Bus;
int16_t i16_IDC_Bus_avg;
int16_t i16_VDC_Bus;
int16_t i16_VDC_Bus_avg;
}AMotorInfo;

typedef struct tag_MotorCommand
{

int16_t i16_Id;
int16_t i16_Iq;
int16_t i16_Vd;
int16_t i16_Vq;
int16_t i16_Valfa;
int16_t i16_Vbeta;
int16_t i16_Va;
int16_t i16_Vb;
int16_t i16_Vc;
int16_t i16_Duty0;
int16_t i16_Duty2;
int16_t i16_Duty4;

int16_t i16_rotor_speed;
int16_t i16_rotor_speed_target;
int16_t i16_angle;

int16_t i16_Iq_target;
int16_t i16_Vq_target;

uint8_t u8_start;

uint8_t u8_direction;
uint8_t u8_zone;
}AMotorCommand;


typedef struct tag_MotorController
{
int32_t i32_Iq_control_limit_dig;
int32_t i32_Iq_control_limit_dig_ui;

int32_t i32_Id_control_limit_dig;
int32_t i32_Id_control_limit_dig_ui;

int32_t i32_Speed_control_limit_dig;
int32_t i32_Speed_control_limit_dig_ui;

int16_t i16_maximum_speed_limit;
}AMotorController;


typedef struct tag_MotorSym
{
uint8_t u8_zero_current_error;
uint8_t u8_Idc_OC_error;
uint8_t u8_Iphase_OC_error;

}AMotorSym;


typedef struct tag_MotorOther
{

int32_t i32_Capture_Value_Q15;
int32_t i32_Last_Capture_Value_Q15;
int32_t i32_Hall_Step_Theda_Q26;
int32_t i32_Hall_Accumulated_Theda_Q26;

int32_t i32_Hall_Step_Theda_EST_Constatnt;
int32_t i32_Hall_Wr_EST_Constant;

uint8_t u8_flag_ECAP_overflow;

uint16_t ui16_initial_hall_change_cnt;
uint8_t u8_operation_state;
uint8_t u8_Motor_Running_State;
uint8_t u8_CMD_Motor_Action;
uint8_t u8_flag_error_record;

uint8_t u8_flag_do_speed_average;

uint16_t ui16_speed_slope_counter;
uint16_t ui16_VSP_12bit_ADC;

uint8_t u8_flag_need_read_VSP;

}AMotorOther;



typedef struct tag_Motor
{
    AMotorSpec spec;
    AMotorInfo info;
    AMotorCommand cmd;
    AMotorController ctrl;
    AMotorSym sym;
    AMotorOther other;
}AMotor;


extern uint8_t u8_flag_VSP_control, u8_flag_UART_control;
extern uint16_t u16_Duty_UART,u16_Duty_VSP;

extern uint8_t u8_flag_1ms, u8_flag_10ms, u8_flag_10ms, u8_flag_100ms, u8_flag_500ms;
extern uint16_t u16_timer0_int_counter, u16_1ms_timer_counter, u16_10ms_time_counter, u16_100ms_time_counter, u16_500ms_time_counter;



extern int16_t i16_bak_Iu_ref, i16_bak_Iv_ref, i16_bak_Iw_ref;


extern AMotor MotorA;


typedef struct
{
    int32_t e1;
    int32_t ui_31;

    int32_t kp_15,ki_15;
}PICs;

extern PICs PI_Iq, PI_Id, PI_Speed;


extern volatile int32_t t1, t2, t3, t4;
# 4 "program\\system_initialize.h" 2





extern void SYS_Init_Clock(void);

extern void Initialize_EPWM(void);
extern void Initialize_EPWM_1R(void);
extern void Set_EPWM_MASK_Enable(uint8_t MASK_DATA);
extern void Set_EPWM_MASK_Disable(void);
extern void Set_GPA_As_EPWM_Output(void);
extern void Set_EPWM_Output_As_GPA_Output(uint8_t OUT_DATA);
extern void Stop_Motor_PWMOUT_OFF(void);
extern void Start_Motor_PWMOUT_ON(int16_t duty0, int16_t duty2, int16_t duty4);
extern void Initialize_Variabls(void);
extern void Initialize_Motor_Parameters(void);
extern void Initialize_PI_Parameters(void);

extern void Initialize_ECAP(void);

extern void Initialize_ADC(void);
extern uint16_t ADC0_SW_Read(uint32_t u32ChannelNum);
extern uint16_t ADC1_SW_Read(uint32_t u32ChannelNum);
extern void Set_ADC1_P1_ADC0_P3_Simu_HW_Trg_by_PWM0Period(void);
extern void Set_ADC0_OP1_O_ADC1_OP1_O_Independent_2SH_Trg_by_Hardware(void);
extern uint16_t SW_Trg_ADC1_P7_Return_Reslut(void);
extern uint16_t SW_Trg_ADC1_P1_Return_Reslut(void);
extern uint16_t SW_Trg_ADC0_P3_Return_Reslut(void);
extern uint16_t SW_Trg_ADC0_P4_Return_Reslut(void);
extern uint16_t SW_Trg_ADC1_TempSensor_Return_Reslut(void);
extern uint16_t SW_Trg_ADC0_BandGap_Return_Reslut(void);
extern uint16_t SW_Trg_ADC0_OP1_O_Return_Reslut(void);
extern uint16_t SW_Trg_ADC1_OP1_O_Return_Reslut(void);
extern void Set_GPIO_as_ADC_Input(void);

extern void Initialize_OP1(void);

extern void Initialize_ACMP0(uint16_t DAC0_value);

extern void Initialize_Timer0(void);

extern void Initialize_USCI1_SPI1_for_DAC(void);

extern void Initialize_GPIO(void);

extern void Initialize_HDIV(void);

extern void Initialize_Temperature_Sensor(void);

extern void Initialize_GDMA(void);

extern void Initialize_NVIC_Priority(void);
extern void Enable_NVIC_For_System_Requirement(void);

extern void Initialize_Motor_System(void);
extern void Iu_Iv_Zero_Current_Calibration(void);

extern void Initialize_varaibles_for_TMR0_int(void);

extern void Initialize_Motor_System_1R(void);
extern void OP1_O_Zero_Current_Calibration(void);

extern void Initialize_NVIC_Priority_1R(void);
extern void Enable_NVIC_For_System_Requirement_1R(void);





extern void Check_control_mode(void);
# 103 "program\\system_initialize.h"
static __inline void CLEAR_EPWM_BK_RESTART_CNT(void)
{

    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->INTSTS = (0x1ul << (16));
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->INTSTS = (0x1ul << (17));


    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL |= (0x1ul << (27));
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL |= (0x1UL) | (0x4UL) | (0x10UL);
}
# 8 "program\\motor_FOC.h" 2




extern const int16_t sinTab[];


extern int16_t Cos(int16_t angle);
extern int16_t Sin(int16_t angle);
extern int16_t under_over_16_cp(int16_t angle);
extern void _Iuvw_to_Idq(AMotor* Motor, int32_t sin, int32_t cos);
extern void Clark_Tranform(int32_t Iu, int32_t Iv, int32_t Iw, int32_t sin, int32_t cos);
extern void Vdq_to_SVPWM_2R(AMotor* Motor, EPWM_T* epwm, int32_t pwm_full_scale, int32_t pwm_max_duty, int32_t sin, int32_t cos);
extern void Vdq_to_SVPWM_1R(AMotor* Motor, EPWM_T* epwm, int32_t pwm_full_scale, int32_t pwm_max_duty, int32_t sin, int32_t cos);


static __inline int16_t COS(int16_t angle)
{
    angle += 256;
    if(angle >= 1024)
        angle -= 1024;
    return sinTab[angle];
}

static __inline int16_t SIN(int16_t angle)
{
    if(angle >= 1024)
    angle -= 1024;
    return sinTab[angle];
}





static __inline void Update_Hall_Angle(int32_t Hall_Accumulated_Theda_Q26, int32_t Hall_Step_Theda_Q26)
{
    if (MotorA.other.u8_operation_state == 2)
    {
        Hall_Accumulated_Theda_Q26 += Hall_Step_Theda_Q26;

        if(Hall_Accumulated_Theda_Q26 > ((1<<26)-1))
            Hall_Accumulated_Theda_Q26 = Hall_Accumulated_Theda_Q26 - (1<<26);
        else if (Hall_Accumulated_Theda_Q26 < 0)
            Hall_Accumulated_Theda_Q26 = Hall_Accumulated_Theda_Q26 + (1<<26);




        MotorA.info.i16_hall_angle = (signed short int) ((Hall_Accumulated_Theda_Q26 >> 16));
    }

    MotorA.other.i32_Hall_Accumulated_Theda_Q26 = Hall_Accumulated_Theda_Q26;
    MotorA.other.i32_Hall_Step_Theda_Q26 = Hall_Step_Theda_Q26;
}






static __inline void For_TEST_ONLY_Update_Hall_Angle(int32_t Hall_Accumulated_Theda_Q26, int32_t Hall_Step_Theda_Q26)
{
    Hall_Accumulated_Theda_Q26 += Hall_Step_Theda_Q26;

    if(Hall_Accumulated_Theda_Q26 > ((1<<26)-1))
        Hall_Accumulated_Theda_Q26 = Hall_Accumulated_Theda_Q26 - (1<<26);
    else if (Hall_Accumulated_Theda_Q26 < 0)
        Hall_Accumulated_Theda_Q26 = Hall_Accumulated_Theda_Q26 + (1<<26);

    MotorA.info.i16_hall_angle = (signed short int) ((Hall_Accumulated_Theda_Q26 >> 16));

    MotorA.other.i32_Hall_Accumulated_Theda_Q26 = Hall_Accumulated_Theda_Q26;
    MotorA.other.i32_Hall_Step_Theda_Q26 = Hall_Step_Theda_Q26;
}





static __inline void Re_Constuct_3_Phase_Current_2R(void)
{
    int32_t Iu, Iv, Iw;

    Iu = ((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADC1_DAT1;
    Iu = -( (Iu - MotorA.info.i16_Iu_ref_ADC) << 4);
    Iv = ((ADC_T *) ((( uint32_t)0x40000000) + 0xE0000))->ADC0_DAT3;
    Iv = -( (Iv - MotorA.info.i16_Iv_ref_ADC) << 4);
    Iw = -Iu - Iv;

    MotorA.info.i16_Iu = Iu;
    MotorA.info.i16_Iv = Iv;
    MotorA.info.i16_Iw = Iw;
}
# 112 "program\\motor_FOC.h"
static __inline void Re_Constuct_3_Phase_Current_1R(int32_t I_1, int32_t I_2)
{
    int32_t Iu, Iv, Iw;
    int32_t I1_ref_ADC, I2_ref_ADC;
    I1_ref_ADC = MotorA.info.i16_Iu_ref_ADC;
    I2_ref_ADC = MotorA.info.i16_Iv_ref_ADC;

    switch(MotorA.cmd.u8_zone)
    {
        case 5:
            Iu = I_1;
            Iu = -( (Iu - I1_ref_ADC) << 4);
            Iv = I_2;
            Iv = ( (Iv - I2_ref_ADC) << 4);
            Iw = -Iu - Iv;
        break;

        case 4:
            Iu = I_1;
            Iu = -( (Iu - I1_ref_ADC) << 4);
            Iw = I_2;
            Iw = ( (Iw - I2_ref_ADC) << 4);
            Iv = -Iu - Iw;
        break;

        case 6:
            Iv = I_1;
            Iv = -( (Iv - I1_ref_ADC) << 4);
            Iw = I_2;
            Iw = ( (Iw - I2_ref_ADC) << 4);
            Iu = -Iv - Iw;
        break;

        case 2:
            Iv = I_1;
            Iv = -( (Iv - I1_ref_ADC) << 4);
            Iu = I_2;
            Iu = ( (Iu - I2_ref_ADC) << 4);
            Iw = -Iv - Iu;
        break;

        case 3:
            Iw = I_1;
            Iw = -( (Iw - I1_ref_ADC) << 4);
            Iu = I_2;
            Iu = ( (Iu - I2_ref_ADC) << 4);
            Iv = -Iw - Iu;
        break;

        case 1:
            Iw = I_1;
            Iw = -( (Iw - I1_ref_ADC) << 4);
            Iv = I_2;
            Iv = ( (Iv - I2_ref_ADC) << 4);
            Iu = -Iw - Iv;
        break;

        default:
            Iu = I_1;
            Iu = -( (Iu - I1_ref_ADC) << 4);
            Iw = I_2;
            Iw = ( (Iw - I2_ref_ADC) << 4);
            Iv = -Iu - Iw;
        break;
    }





    MotorA.info.i16_Iu = (Iu + MotorA.info.i16_Iu) >> 1;
    MotorA.info.i16_Iv = (Iv + MotorA.info.i16_Iv) >> 1;
    MotorA.info.i16_Iw = (Iw + MotorA.info.i16_Iw) >> 1;
}
# 3 "program/motor_FOC.c" 2
# 1 "program\\motor_functions.h" 1
# 10 "program\\motor_functions.h"
# 1 "program\\PI_control.h" 1








extern int16_t PI_Iq_current(AMotor* Motor, PICs* C, int16_t FW_Item);
extern int16_t PI_Id_current(AMotor* Motor, PICs* C, int16_t FW_Item);
extern int16_t PI_speed(AMotor* Motor, PICs* C, int16_t FW_Item);
extern int16_t PI_speed_six_step(AMotor* Motor, PICs* C, int16_t FW_Item);
extern int32_t PI_controller(int16_t out_limit, int32_t ui_limit, int32_t Error, PICs* C, int16_t FW_Item);
# 11 "program\\motor_functions.h" 2
# 25 "program\\motor_functions.h"
extern uint16_t temp_data, tempA, tempB, tempC, tempD;
extern void DAC_Output_M0(void);
extern void DAC_Module_Check(void);
extern void DAC_Output_M0_1R(void);
# 38 "program\\motor_functions.h"
extern void PhaseCurrent_OC_Check(AMotor* Motor);

extern void Detect_Initial_Hall_Position_M0(void);

extern uint8_t Check_Rotor_FW_RW_M0(uint8_t u8_Last_Hall_Pos, uint8_t u8_Hall_Pos, uint8_t u8_Last_Direction);
extern void Detect_Initial_Hall_Position_M0(void);
extern void HallSpeedEstimator(AMotor* Motor);
extern void Fix_Phase_Angle_M0(int16_t i16_Shift_Angle);

extern void cmd_speed_slope_with_hall(AMotor* Motor);

extern void read_VSP_Check_ON_OFF_Motor(AMotor* Motor);
extern void read_UART_Check_ON_OFF_Motor(AMotor* Motor);
extern void read_VSP_to_speed_cmd(AMotor* Motor);
extern void read_VSP_to_Iq_cmd(AMotor* Motor);
extern void read_VSP_to_Vq_cmd(AMotor* Motor);
extern void read_VSP_to_duty_cmd(AMotor* Motor);


extern void delay_Xmsec(unsigned int z);

extern void Enable_CLKO_to_PA0(void);
# 4 "program/motor_FOC.c" 2
# 1 "program\\svpwm.h" 1
# 27 "program\\svpwm.h"
extern void _Iuvw_to_Idq(AMotor* Motor, int32_t sin, int32_t cos);
# 37 "program\\svpwm.h"
extern void SVPWM_2R(int32_t Vd, int32_t Vq, int32_t sin, int32_t cos, int32_t pwm_full_scale, AMotor* Motor, uint16_t* zone, uint16_t* Taoff, uint16_t* Tboff, uint16_t* Tcoff);
# 47 "program\\svpwm.h"
extern void SVPWM_1R(int32_t Vd, int32_t Vq, int32_t sin, int32_t cos, int32_t pwm_full_scale, uint16_t pwm_shift_CMP0, uint16_t pwm_shift_CMP1, uint16_t Max_PWM_Duty,
      uint16_t PWM_Margine, AMotor* Motor, uint16_t* zone, uint16_t* Taoff_Up, uint16_t* Tboff_Up, uint16_t* Tcoff_Up, uint16_t* Taoff_Down, uint16_t* Tboff_Down, uint16_t* Tcoff_Down);
# 5 "program/motor_FOC.c" 2



unsigned char u8_Flag_DoPWMShift, u8_Flag_LeftShift, u8_Flag_RightShift;
uint32_t ui32_duty0, ui32_duty2, ui32_duty4;




const int16_t sinTab[] =
{
     0 , 201 , 402 , 603 , 804 , 1006 , 1207 , 1408 , 1609 , 1810 ,
  2011 , 2212 , 2412 , 2613 , 2814 , 3014 , 3214 , 3415 , 3615 , 3815 ,
  4015 , 4214 , 4414 , 4613 , 4812 , 5011 , 5210 , 5409 , 5607 , 5805 ,
  6003 , 6201 , 6398 , 6596 , 6793 , 6989 , 7186 , 7382 , 7578 , 7774 ,
  7969 , 8164 , 8359 , 8553 , 8747 , 8941 , 9135 , 9328 , 9521 , 9713 ,
  9905 , 10097 , 10288 , 10479 , 10669 , 10859 , 11049 , 11238 , 11427 , 11616 ,
 11804 , 11991 , 12178 , 12365 , 12551 , 12737 , 12922 , 13106 , 13291 , 13474 ,
 13658 , 13840 , 14022 , 14204 , 14385 , 14566 , 14746 , 14925 , 15104 , 15282 ,
 15460 , 15637 , 15814 , 15990 , 16165 , 16340 , 16514 , 16688 , 16860 , 17033 ,
 17204 , 17375 , 17546 , 17715 , 17884 , 18052 , 18220 , 18387 , 18553 , 18719 ,
 18884 , 19048 , 19211 , 19374 , 19536 , 19697 , 19857 , 20017 , 20176 , 20334 ,
 20492 , 20648 , 20804 , 20959 , 21114 , 21267 , 21420 , 21572 , 21723 , 21873 ,
 22023 , 22171 , 22319 , 22466 , 22612 , 22757 , 22902 , 23045 , 23188 , 23330 ,
 23470 , 23610 , 23750 , 23888 , 24025 , 24161 , 24297 , 24432 , 24565 , 24698 ,
 24830 , 24961 , 25090 , 25219 , 25347 , 25475 , 25601 , 25726 , 25850 , 25973 ,
 26095 , 26217 , 26337 , 26456 , 26574 , 26692 , 26808 , 26923 , 27037 , 27150 ,
 27263 , 27374 , 27484 , 27593 , 27701 , 27808 , 27914 , 28019 , 28123 , 28225 ,
 28327 , 28428 , 28527 , 28626 , 28723 , 28819 , 28915 , 29009 , 29102 , 29194 ,
 29285 , 29374 , 29463 , 29550 , 29637 , 29722 , 29806 , 29889 , 29971 , 30052 ,
 30132 , 30210 , 30288 , 30364 , 30439 , 30513 , 30586 , 30657 , 30728 , 30797 ,
 30865 , 30932 , 30998 , 31063 , 31126 , 31189 , 31250 , 31310 , 31368 , 31426 ,
 31482 , 31538 , 31592 , 31644 , 31696 , 31747 , 31796 , 31844 , 31891 , 31936 ,
 31981 , 32024 , 32066 , 32107 , 32146 , 32185 , 32222 , 32258 , 32293 , 32326 ,
 32359 , 32390 , 32419 , 32448 , 32476 , 32502 , 32527 , 32550 , 32573 , 32594 ,
 32614 , 32633 , 32651 , 32667 , 32682 , 32696 , 32709 , 32720 , 32730 , 32739 ,
 32747 , 32754 , 32759 , 32763 , 32766 , 32767 , 32767 , 32767 , 32764 , 32761 ,
 32756 , 32750 , 32743 , 32735 , 32725 , 32715 , 32703 , 32689 , 32675 , 32659 ,
 32642 , 32624 , 32604 , 32584 , 32562 , 32539 , 32514 , 32489 , 32462 , 32434 ,
 32405 , 32374 , 32343 , 32310 , 32275 , 32240 , 32204 , 32166 , 32127 , 32087 ,
 32045 , 32003 , 31959 , 31914 , 31867 , 31820 , 31771 , 31721 , 31670 , 31618 ,
 31565 , 31510 , 31454 , 31397 , 31339 , 31280 , 31219 , 31157 , 31095 , 31031 ,
 30965 , 30899 , 30831 , 30763 , 30693 , 30622 , 30549 , 30476 , 30402 , 30326 ,
 30249 , 30171 , 30092 , 30012 , 29930 , 29848 , 29764 , 29680 , 29594 , 29507 ,
 29419 , 29330 , 29239 , 29148 , 29055 , 28962 , 28867 , 28771 , 28674 , 28577 ,
 28478 , 28377 , 28276 , 28174 , 28071 , 27966 , 27861 , 27755 , 27647 , 27538 ,
 27429 , 27318 , 27207 , 27094 , 26980 , 26866 , 26750 , 26633 , 26515 , 26397 ,
 26277 , 26156 , 26034 , 25912 , 25788 , 25663 , 25538 , 25411 , 25284 , 25155 ,
 25026 , 24895 , 24764 , 24632 , 24498 , 24364 , 24229 , 24093 , 23956 , 23819 ,
 23680 , 23541 , 23400 , 23259 , 23117 , 22973 , 22830 , 22685 , 22539 , 22393 ,
 22245 , 22097 , 21948 , 21798 , 21647 , 21496 , 21344 , 21191 , 21037 , 20882 ,
 20726 , 20570 , 20413 , 20255 , 20097 , 19937 , 19777 , 19616 , 19455 , 19293 ,
 19130 , 18966 , 18801 , 18636 , 18470 , 18304 , 18136 , 17968 , 17800 , 17630 ,
 17461 , 17290 , 17119 , 16947 , 16774 , 16601 , 16427 , 16253 , 16078 , 15902 ,
 15726 , 15549 , 15371 , 15193 , 15015 , 14835 , 14656 , 14475 , 14295 , 14113 ,
 13931 , 13749 , 13566 , 13383 , 13199 , 13014 , 12829 , 12644 , 12458 , 12271 ,
 12085 , 11897 , 11710 , 11521 , 11333 , 11144 , 10954 , 10764 , 10574 , 10383 ,
 10192 , 10001 , 9809 , 9617 , 9424 , 9231 , 9038 , 8844 , 8650 , 8456 ,
  8262 , 8067 , 7871 , 7676 , 7480 , 7284 , 7088 , 6891 , 6694 , 6497 ,
  6300 , 6102 , 5904 , 5706 , 5508 , 5309 , 5111 , 4912 , 4713 , 4513 ,
  4314 , 4114 , 3915 , 3715 , 3515 , 3315 , 3114 , 2914 , 2713 , 2513 ,
  2312 , 2111 , 1910 , 1709 , 1508 , 1307 , 1106 , 905 , 704 , 503 ,
   301 , 100 , -100 , -301 , -503 , -704 , -905 , -1106 , -1307 , -1508 ,
 -1709 , -1910 , -2111 , -2312 , -2513 , -2713 , -2914 , -3114 , -3315 , -3515 ,
 -3715 , -3915 , -4114 , -4314 , -4513 , -4713 , -4912 , -5111 , -5309 , -5508 ,
 -5706 , -5904 , -6102 , -6300 , -6497 , -6694 , -6891 , -7088 , -7284 , -7480 ,
 -7676 , -7871 , -8067 , -8262 , -8456 , -8650 , -8844 , -9038 , -9231 , -9424 ,
 -9617 , -9809 ,-10001 ,-10192 ,-10383 ,-10574 ,-10764 ,-10954 ,-11144 ,-11333 ,
-11521 ,-11710 ,-11897 ,-12085 ,-12271 ,-12458 ,-12644 ,-12829 ,-13014 ,-13199 ,
-13383 ,-13566 ,-13749 ,-13931 ,-14113 ,-14295 ,-14475 ,-14656 ,-14835 ,-15015 ,
-15193 ,-15371 ,-15549 ,-15726 ,-15902 ,-16078 ,-16253 ,-16427 ,-16601 ,-16774 ,
-16947 ,-17119 ,-17290 ,-17461 ,-17630 ,-17800 ,-17968 ,-18136 ,-18304 ,-18470 ,
-18636 ,-18801 ,-18966 ,-19130 ,-19293 ,-19455 ,-19616 ,-19777 ,-19937 ,-20097 ,
-20255 ,-20413 ,-20570 ,-20726 ,-20882 ,-21037 ,-21191 ,-21344 ,-21496 ,-21647 ,
-21798 ,-21948 ,-22097 ,-22245 ,-22393 ,-22539 ,-22685 ,-22830 ,-22973 ,-23117 ,
-23259 ,-23400 ,-23541 ,-23680 ,-23819 ,-23956 ,-24093 ,-24229 ,-24364 ,-24498 ,
-24632 ,-24764 ,-24895 ,-25026 ,-25155 ,-25284 ,-25411 ,-25538 ,-25663 ,-25788 ,
-25912 ,-26034 ,-26156 ,-26277 ,-26397 ,-26515 ,-26633 ,-26750 ,-26866 ,-26980 ,
-27094 ,-27207 ,-27318 ,-27429 ,-27538 ,-27647 ,-27755 ,-27861 ,-27966 ,-28071 ,
-28174 ,-28276 ,-28377 ,-28478 ,-28577 ,-28674 ,-28771 ,-28867 ,-28962 ,-29055 ,
-29148 ,-29239 ,-29330 ,-29419 ,-29507 ,-29594 ,-29680 ,-29764 ,-29848 ,-29930 ,
-30012 ,-30092 ,-30171 ,-30249 ,-30326 ,-30402 ,-30476 ,-30549 ,-30622 ,-30693 ,
-30763 ,-30831 ,-30899 ,-30965 ,-31031 ,-31095 ,-31157 ,-31219 ,-31280 ,-31339 ,
-31397 ,-31454 ,-31510 ,-31565 ,-31618 ,-31670 ,-31721 ,-31771 ,-31820 ,-31867 ,
-31914 ,-31959 ,-32003 ,-32045 ,-32087 ,-32127 ,-32166 ,-32204 ,-32240 ,-32275 ,
-32310 ,-32343 ,-32374 ,-32405 ,-32434 ,-32462 ,-32489 ,-32514 ,-32539 ,-32562 ,
-32584 ,-32604 ,-32624 ,-32642 ,-32659 ,-32675 ,-32689 ,-32703 ,-32715 ,-32725 ,
-32735 ,-32743 ,-32750 ,-32756 ,-32761 ,-32764 ,-32767 ,-32767 ,-32767 ,-32766 ,
-32763 ,-32759 ,-32754 ,-32747 ,-32739 ,-32730 ,-32720 ,-32709 ,-32696 ,-32682 ,
-32667 ,-32651 ,-32633 ,-32614 ,-32594 ,-32573 ,-32550 ,-32527 ,-32502 ,-32476 ,
-32448 ,-32419 ,-32390 ,-32359 ,-32326 ,-32293 ,-32258 ,-32222 ,-32185 ,-32146 ,
-32107 ,-32066 ,-32024 ,-31981 ,-31936 ,-31891 ,-31844 ,-31796 ,-31747 ,-31696 ,
-31644 ,-31592 ,-31538 ,-31482 ,-31426 ,-31368 ,-31310 ,-31250 ,-31189 ,-31126 ,
-31063 ,-30998 ,-30932 ,-30865 ,-30797 ,-30728 ,-30657 ,-30586 ,-30513 ,-30439 ,
-30364 ,-30288 ,-30210 ,-30132 ,-30052 ,-29971 ,-29889 ,-29806 ,-29722 ,-29637 ,
-29550 ,-29463 ,-29374 ,-29285 ,-29194 ,-29102 ,-29009 ,-28915 ,-28819 ,-28723 ,
-28626 ,-28527 ,-28428 ,-28327 ,-28225 ,-28123 ,-28019 ,-27914 ,-27808 ,-27701 ,
-27593 ,-27484 ,-27374 ,-27263 ,-27150 ,-27037 ,-26923 ,-26808 ,-26692 ,-26574 ,
-26456 ,-26337 ,-26217 ,-26095 ,-25973 ,-25850 ,-25726 ,-25601 ,-25475 ,-25347 ,
-25219 ,-25090 ,-24961 ,-24830 ,-24698 ,-24565 ,-24432 ,-24297 ,-24161 ,-24025 ,
-23888 ,-23750 ,-23610 ,-23470 ,-23330 ,-23188 ,-23045 ,-22902 ,-22757 ,-22612 ,
-22466 ,-22319 ,-22171 ,-22023 ,-21873 ,-21723 ,-21572 ,-21420 ,-21267 ,-21114 ,
-20959 ,-20804 ,-20648 ,-20492 ,-20334 ,-20176 ,-20017 ,-19857 ,-19697 ,-19536 ,
-19374 ,-19211 ,-19048 ,-18884 ,-18719 ,-18553 ,-18387 ,-18220 ,-18052 ,-17884 ,
-17715 ,-17546 ,-17375 ,-17204 ,-17033 ,-16860 ,-16688 ,-16514 ,-16340 ,-16165 ,
-15990 ,-15814 ,-15637 ,-15460 ,-15282 ,-15104 ,-14925 ,-14746 ,-14566 ,-14385 ,
-14204 ,-14022 ,-13840 ,-13658 ,-13474 ,-13291 ,-13106 ,-12922 ,-12737 ,-12551 ,
-12365 ,-12178 ,-11991 ,-11804 ,-11616 ,-11427 ,-11238 ,-11049 ,-10859 ,-10669 ,
-10479 ,-10288 ,-10097 , -9905 , -9713 , -9521 , -9328 , -9135 , -8941 , -8747 ,
 -8553 , -8359 , -8164 , -7969 , -7774 , -7578 , -7382 , -7186 , -6989 , -6793 ,
 -6596 , -6398 , -6201 , -6003 , -5805 , -5607 , -5409 , -5210 , -5011 , -4812 ,
 -4613 , -4414 , -4214 , -4015 , -3815 , -3615 , -3415 , -3214 , -3014 , -2814 ,
 -2613 , -2412 , -2212 , -2011 , -1810 , -1609 , -1408 , -1207 , -1006 , -804 ,
  -603 , -402 , -201 , 0
};




int16_t Cos(int16_t angle)
{
    angle += 256;
    if(angle >= 1024)
        angle -= 1024;
    return sinTab[angle];
}

int16_t Sin(int16_t angle)
{
    if(angle >= 1024)
        angle -= 1024;
    return sinTab[angle];
}

int16_t under_over_16_cp(int16_t angle)
{
    if(angle > 32767)
        angle = angle - 32768;
    else if(angle < 0)
        angle = angle + 32768;
    else
        angle = angle;
    return angle;
}
# 158 "program/motor_FOC.c"
void _Iuvw_to_Idq(AMotor* Motor, int32_t sin, int32_t cos)
{
  int32_t Iu, Iv, Iw, Ialfa, Ibeta, Id, Iq;

  Iu = Motor->info.i16_Iu;
  Iv = Motor->info.i16_Iv;
  Iw = Motor->info.i16_Iw;


  Ialfa = Iu;
  Ibeta = ((Iv - Iw) * 18918) >> 15;


  Id = ( (Ialfa * cos) + (Ibeta * sin)) >> 15;
  Iq = (-(Ialfa * sin) + (Ibeta * cos)) >> 15;


  Motor->info.i16_Ialfa = Ialfa;
  Motor->info.i16_Ibeta = Ibeta;
  Motor->info.i16_Id = Id;
  Motor->info.i16_Iq = Iq;
}
# 191 "program/motor_FOC.c"
void Vdq_to_SVPWM_2R(AMotor* Motor, EPWM_T* epwm, int32_t pwm_full_scale, int32_t pwm_max_duty, int32_t sin, int32_t cos)
{
    int32_t Vd, Vq;
    uint16_t zone, Taoff, Tboff, Tcoff;
    uint32_t PWM0H_duty, PWM2H_duty, PWM4H_duty ;

    Vd = Motor->cmd.i16_Vd;
    Vq = Motor->cmd.i16_Vq;

    SVPWM_2R(Vd, Vq, sin, cos, pwm_full_scale, Motor, &zone, &Taoff, &Tboff, &Tcoff);



    PWM0H_duty = pwm_full_scale - Taoff;
    PWM2H_duty = pwm_full_scale - Tboff;
    PWM4H_duty = pwm_full_scale - Tcoff;


    if(PWM0H_duty > pwm_max_duty)
        PWM0H_duty = pwm_max_duty;
    if(PWM2H_duty > pwm_max_duty)
        PWM2H_duty = pwm_max_duty;
    if(PWM4H_duty > pwm_max_duty)
        PWM4H_duty = pwm_max_duty;


    epwm->CMPDAT[0] = PWM0H_duty;
    epwm->CMPDAT[2] = PWM2H_duty;
    epwm->CMPDAT[4] = PWM4H_duty;


    Motor->cmd.i16_Duty0 = PWM0H_duty;
    Motor->cmd.i16_Duty2 = PWM2H_duty;
    Motor->cmd.i16_Duty4 = PWM4H_duty;
}
