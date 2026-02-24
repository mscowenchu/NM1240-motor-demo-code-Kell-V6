# 1 "program/protocol.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 393 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "program/protocol.c" 2
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
# 2 "program/protocol.c" 2
# 1 "program\\protocol.h" 1
# 32 "program\\protocol.h"
typedef union{
    uint32_t LONG;
    struct{
        uint16_t LWord;
        uint16_t HWord;
    }Word;
    struct{
        uint8_t LLByte;
        uint8_t LHByte;
        uint8_t HLByte;
        uint8_t HHByte;
    }Byte;
}MASK_T;

typedef union{
    int32_t LONG;
    struct{
        int16_t LWord;
        int16_t HWord;
    }Word;
    struct{
        int8_t LLByte;
        int8_t LHByte;
        int8_t HLByte;
        int8_t HHByte;
    }Byte;
}VAR_T;


typedef struct
{
    uint8_t name[50];
    uint32_t address;
    MASK_T bit_mask;
    VAR_T step_value;
    VAR_T min_value;
    VAR_T max_value;
    uint8_t signed_int;
}CMD_TypeDef;

typedef struct
{
    uint8_t name[50];
    uint32_t address;
    MASK_T bit_mask;
    VAR_T min_value;
    VAR_T max_value;
    uint8_t signed_int;
}VAR_STRUCT_TypeDef;

typedef struct
{
VAR_T data;
uint8_t changed_flag;
}Temp_T;

typedef struct
{
  uint8_t flag;
  uint8_t cmd_group;
} Refresh_req_TypeDef;
# 174 "program\\protocol.h"
void initial_SYS_name(uint8_t sys_index);
void initial_CMD_name(uint8_t cmd_index, CMD_TypeDef* CMD_LIST, const uint8_t size);
void initial_INFO_name(uint8_t info_index);
void initial_MICRO_name(uint8_t micro_index);
void initial_MACRO_name(uint8_t macro_index);

void initial_SYS(uint8_t sys_index);
void initial_CMD(uint8_t cmd_index, CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp);
void initial_INFO(uint8_t info_index);
void initial_micro_fig(uint8_t micro_index);
void initial_macro_fig(uint8_t macro_index);

void Update_microcosm_fig(void);
void Update_macrocosm_fig(void);
void TX_information(void);
void Update_time_scale(uint8_t rx_data[]);
void Command_update(void);
void RX_system_command(uint8_t rx_data[]);
void RX_command(uint8_t rx_data[], CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp);
void Refresh(CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp, const uint8_t size);
void Refresh_sys(void);

void TX_data_to_PC(int16_t rotor_speed, uint16_t min_cmd_speed_rpm);
void Trigger_update_micro_fig(void);
void Request_decode(uint8_t rx_data[]);

void Initialize_FMC(void);
uint8_t data_flash_is_empty(void);
void read_data_flash_to_CMD_temp(void);
void read_data_flash_to_GUI(uint8_t cmd_group, const uint8_t size);
void write_data_flash_from_GUI(CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp, uint8_t cmd_group, const uint8_t size);
void erase_data_flash(void);
void Load_from_data_flash(void);
void Initialize_USCI_UART(void);
# 3 "program/protocol.c" 2
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
# 4 "program/protocol.c" 2
# 17 "program/protocol.c"
VAR_STRUCT_TypeDef const SYS_LIST[] =
{

    {"MotorA.cmd.u8_start\n" , (uint32_t)&MotorA.cmd.u8_start , 0x000000FF , 0 , 1 , 0}
};

CMD_TypeDef const GROUP0_CMD_LIST[] =
{

    {"MotorA.cmd.u8_direction\n" , (uint32_t)&MotorA.cmd.u8_direction , 0x000000FF , 1 , 0 , 1 , 0},
    {"MotorA.cmd.i16_rotor_speed_target\n" , (uint32_t)&MotorA.cmd.i16_rotor_speed_target , 0x0000FFFF , 500 , 0 , 3000 , 1},
    {"MotorA.cmd.i16_Duty0\n" , (uint32_t)&MotorA.cmd.i16_Duty0 , 0x0000FFFF , 10 , 0 , 100 , 1}
};

CMD_TypeDef const GROUP1_CMD_LIST[] =
{

    {"PI_Speed.kp_15\n" , (uint32_t)&PI_Speed.kp_15 , 0xFFFFFFFF , 2000 , 0 , 0x7FFFFFFF , 0},
    {"PI_Speed.ki_15\n" , (uint32_t)&PI_Speed.ki_15 , 0xFFFFFFFF , 50 , 0 , 0x7FFFFFFF , 0}
};

CMD_TypeDef const GROUP2_CMD_LIST[] =
{

    {"PI_Iq.kp_15\n" , (uint32_t)&PI_Iq.kp_15 , 0xFFFFFFFF , 50 , 0 , 0x7FFFFFFF , 0},
    {"PI_Iq.ki_15\n" , (uint32_t)&PI_Iq.ki_15 , 0xFFFFFFFF , 10 , 0 , 0x7FFFFFFF , 0}
};

CMD_TypeDef const GROUP3_CMD_LIST[] =
{

    {"PI_Id.kp_15\n" , (uint32_t)&PI_Id.kp_15 , 0xFFFFFFFF , 50 , 0 , 0x7FFFFFFF , 0},
    {"PI_Id.ki_15\n" , (uint32_t)&PI_Id.ki_15 , 0xFFFFFFFF , 10 , 0 , 0x7FFFFFFF , 0}
};

VAR_STRUCT_TypeDef const INFO_LIST[] =
{

    {"MotorA.cmd.i16_rotor_speed_target\n" , (uint32_t)&MotorA.cmd.i16_rotor_speed_target , 0x0000FFFF , -5000 , 5000 , 1},
    {"MotorA.info.i16_rotor_speed\n" , (uint32_t)&MotorA.info.i16_rotor_speed , 0x0000FFFF , -5000 , 5000 , 1},
    {"MotorA.info.i16_Id\n" , (uint32_t)&MotorA.info.i16_Id , 0x0000FFFF , -32768 , 32767 , 1},
    {"MotorA.info.i16_Iq\n" , (uint32_t)&MotorA.info.i16_Iq , 0x0000FFFF , -32768 , 32767 , 1}
};

VAR_STRUCT_TypeDef const MICRO_LIST[] =
{

    {"MotorA.info.i16_Iu\n" , (uint32_t)&MotorA.info.i16_Iu , 0x0000FFFF , -32768 , 32767 , 1},
    {"MotorA.info.i16_Iv\n" , (uint32_t)&MotorA.info.i16_Iv , 0x0000FFFF , -32768 , 32767 , 1},
    {"MotorA.info.i16_Iw\n" , (uint32_t)&MotorA.info.i16_Iw , 0x0000FFFF , -32768 , 32767 , 1},
    {"MotorA.cmd.i16_angle\n" , (uint32_t)&MotorA.cmd.i16_angle , 0x0000FFFF , 0 , 1024 , 1},
    {"MotorA.info.i16_Id\n" , (uint32_t)&MotorA.info.i16_Id , 0x0000FFFF , -32768 , 32767 , 1},
    {"MotorA.info.i16_Iq\n" , (uint32_t)&MotorA.info.i16_Iq , 0x0000FFFF , -32768 , 32767 , 1}
};

VAR_STRUCT_TypeDef const MACRO_LIST[] =
{

    {"MotorA.cmd.i16_rotor_speed_target\n" , (uint32_t)&MotorA.cmd.i16_rotor_speed_target , 0x0000FFFF , -5000 , 5000 , 1},
    {"MotorA.info.i16_rotor_speed\n" , (uint32_t)&MotorA.info.i16_rotor_speed , 0x0000FFFF , -5000 , 5000 , 1},
    {"MotorA.info.i16_Id\n" , (uint32_t)&MotorA.info.i16_Id , 0x0000FFFF , -32768 , 32767 , 1},
    {"MotorA.info.i16_Iq\n" , (uint32_t)&MotorA.info.i16_Iq , 0x0000FFFF , -32768 , 32767 , 1}
};

uint16_t TIME_SCALE_CNT = 151;
uint8_t Update_microcosm_fig_req = 0;
uint8_t Update_macrocosm_fig_req = 0;
uint8_t Tx_information_req = 0;
uint8_t Update_microcosm_fig_trigger = 0;
Refresh_req_TypeDef Refresh_group_req = {0, 0};
uint8_t Refresh_sys_req = 0;
uint32_t NVIC_IRQ_TEMP, PWM_CTL_TEMP;

uint8_t TIME_SCALE = 2;

uint8_t RX_data[7] = {0};
uint8_t TX_data[100] = {0};
uint8_t UART_TX_TCNT = 0;
uint8_t UART_RX_CNT = 0;
uint16_t MICRO_TX_CNT;
uint32_t GROUP_CMD_BASE[4] = {(uint32_t)&GROUP0_CMD_LIST[0].name[0], (uint32_t)&GROUP1_CMD_LIST[0].name[0], (uint32_t)&GROUP2_CMD_LIST[0].name[0], (uint32_t)&GROUP3_CMD_LIST[0].name[0]};
uint8_t GROUP_CMD_SIZE[4] = {sizeof(GROUP0_CMD_LIST)/76, sizeof(GROUP1_CMD_LIST)/76, sizeof(GROUP2_CMD_LIST)/76, sizeof(GROUP3_CMD_LIST)/76};


Temp_T GROUP0_CMD_temp[sizeof(GROUP0_CMD_LIST)/76];
Temp_T GROUP1_CMD_temp[sizeof(GROUP1_CMD_LIST)/76];
Temp_T GROUP2_CMD_temp[sizeof(GROUP2_CMD_LIST)/76];
Temp_T GROUP3_CMD_temp[sizeof(GROUP3_CMD_LIST)/76];
Temp_T SYS_temp[sizeof(SYS_LIST)/72];

uint32_t GROUP_TEMP_BASE[4] = {(uint32_t)&GROUP0_CMD_temp[0].data, (uint32_t)&GROUP1_CMD_temp[0].data, (uint32_t)&GROUP2_CMD_temp[0].data, (uint32_t)&GROUP3_CMD_temp[0].data};


void initial_SYS_name(uint8_t sys_index)
{
    uint8_t name_width_cnt = 0;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    if (sys_index < sizeof(SYS_LIST)/72) {

        TX_data[0] = 0x50;
    }
    else {

        TX_data[0] = 0xCC;
        TX_data[1] = '\n';
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = 2; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
        return;
    }


    for (name_width_cnt=0;name_width_cnt<50;name_width_cnt++)
    {
        TX_data[name_width_cnt+1] = SYS_LIST[sys_index].name[name_width_cnt];
        if (SYS_LIST[sys_index].name[name_width_cnt] == '\n')
        {
            UART_TX_TCNT = name_width_cnt + 2;
            break;
        }
    }

    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_CMD_name(uint8_t cmd_index, CMD_TypeDef* CMD_LIST, const uint8_t size)
{
    uint8_t name_width_cnt = 0;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    if (cmd_index < size) {

        TX_data[0] = 0x51;
    }
    else {

        TX_data[0] = 0xCC;
        TX_data[1] = '\n';
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = 2; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
        return;
    }


    for (name_width_cnt=0;name_width_cnt<50;name_width_cnt++)
    {
        TX_data[name_width_cnt+1] = CMD_LIST[cmd_index].name[name_width_cnt];
        if (CMD_LIST[cmd_index].name[name_width_cnt] == '\n')
        {
            UART_TX_TCNT = name_width_cnt + 2;
            break;
        }
    }

    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_INFO_name(uint8_t info_index)
{
    uint8_t name_width_cnt = 0;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    if (info_index < sizeof(INFO_LIST)/72) {

        TX_data[0] = 0x52;
    }
    else {

        TX_data[0] = 0xCC;
        TX_data[1] = '\n';
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = 2; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
        return;
    }


    for (name_width_cnt=0;name_width_cnt<50;name_width_cnt++)
    {
        TX_data[name_width_cnt+1] = INFO_LIST[info_index].name[name_width_cnt];
        if (INFO_LIST[info_index].name[name_width_cnt] == '\n')
        {
            UART_TX_TCNT = name_width_cnt + 2;
            break;
        }
    }

    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_MICRO_name(uint8_t micro_index)
{
    uint8_t name_width_cnt = 0;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    if (micro_index < sizeof(MICRO_LIST)/72) {

        TX_data[0] = 0x53;
    }
    else {

        TX_data[0] = 0xCC;
        TX_data[1] = '\n';
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = 2; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
        return;
    }


    for (name_width_cnt=0;name_width_cnt<50;name_width_cnt++)
    {
        TX_data[name_width_cnt+1] = MICRO_LIST[micro_index].name[name_width_cnt];
        if (MICRO_LIST[micro_index].name[name_width_cnt] == '\n')
        {
            UART_TX_TCNT = name_width_cnt + 2;
            break;
        }
    }

    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_MACRO_name(uint8_t macro_index)
{
    uint8_t name_width_cnt = 0;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    if (macro_index < sizeof(MACRO_LIST)/72) {

        TX_data[0] = 0x54;
    }
    else {

        TX_data[0] = 0xCC;
        TX_data[1] = '\n';
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = 2; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
        return;
    }


    for (name_width_cnt=0;name_width_cnt<50;name_width_cnt++)
    {
        TX_data[name_width_cnt+1] = MACRO_LIST[macro_index].name[name_width_cnt];
        if (MACRO_LIST[macro_index].name[name_width_cnt] == '\n')
        {
            UART_TX_TCNT = name_width_cnt + 2;
            break;
        }
    }

    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_SYS(uint8_t sys_index)
{
    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x60;
    TX_data[1] = SYS_LIST[sys_index].signed_int;

    TX_data[2] = SYS_LIST[sys_index].bit_mask.Byte.HHByte;
    TX_data[3] = SYS_LIST[sys_index].bit_mask.Byte.HLByte;
    TX_data[4] = SYS_LIST[sys_index].bit_mask.Byte.LHByte;
    TX_data[5] = SYS_LIST[sys_index].bit_mask.Byte.LLByte;

    TX_data[6] = SYS_LIST[sys_index].min_value.Byte.HHByte;
    TX_data[7] = SYS_LIST[sys_index].min_value.Byte.HLByte;
    TX_data[8] = SYS_LIST[sys_index].min_value.Byte.LHByte;
    TX_data[9] = SYS_LIST[sys_index].min_value.Byte.LLByte;

    TX_data[10] = SYS_LIST[sys_index].max_value.Byte.HHByte;
    TX_data[11] = SYS_LIST[sys_index].max_value.Byte.HLByte;
    TX_data[12] = SYS_LIST[sys_index].max_value.Byte.LHByte;
    TX_data[13] = SYS_LIST[sys_index].max_value.Byte.LLByte;

    TX_data[14] = (*(uint8_t*) (SYS_LIST[sys_index].address + 3)) & SYS_LIST[sys_index].bit_mask.Byte.HHByte;
    TX_data[15] = (*(uint8_t*) (SYS_LIST[sys_index].address + 2)) & SYS_LIST[sys_index].bit_mask.Byte.HLByte;
    TX_data[16] = (*(uint8_t*) (SYS_LIST[sys_index].address + 1)) & SYS_LIST[sys_index].bit_mask.Byte.LHByte;
    TX_data[17] = (*(uint8_t*) (SYS_LIST[sys_index].address )) & SYS_LIST[sys_index].bit_mask.Byte.LLByte;

    UART_TX_TCNT = 18;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_CMD(uint8_t cmd_index, CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp)
{
    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x61;
    TX_data[1] = CMD_LIST[cmd_index].signed_int;

    TX_data[2] = CMD_LIST[cmd_index].bit_mask.Byte.HHByte;
    TX_data[3] = CMD_LIST[cmd_index].bit_mask.Byte.HLByte;
    TX_data[4] = CMD_LIST[cmd_index].bit_mask.Byte.LHByte;
    TX_data[5] = CMD_LIST[cmd_index].bit_mask.Byte.LLByte;

    TX_data[6] = CMD_LIST[cmd_index].step_value.Byte.HHByte;
    TX_data[7] = CMD_LIST[cmd_index].step_value.Byte.HLByte;
    TX_data[8] = CMD_LIST[cmd_index].step_value.Byte.LHByte;
    TX_data[9] = CMD_LIST[cmd_index].step_value.Byte.LLByte;

    TX_data[10] = CMD_LIST[cmd_index].min_value.Byte.HHByte;
    TX_data[11] = CMD_LIST[cmd_index].min_value.Byte.HLByte;
    TX_data[12] = CMD_LIST[cmd_index].min_value.Byte.LHByte;
    TX_data[13] = CMD_LIST[cmd_index].min_value.Byte.LLByte;

    TX_data[14] = CMD_LIST[cmd_index].max_value.Byte.HHByte;
    TX_data[15] = CMD_LIST[cmd_index].max_value.Byte.HLByte;
    TX_data[16] = CMD_LIST[cmd_index].max_value.Byte.LHByte;
    TX_data[17] = CMD_LIST[cmd_index].max_value.Byte.LLByte;

    TX_data[18] = (*(uint8_t*) (CMD_LIST[cmd_index].address + 3)) & CMD_LIST[cmd_index].bit_mask.Byte.HHByte;
    TX_data[19] = (*(uint8_t*) (CMD_LIST[cmd_index].address + 2)) & CMD_LIST[cmd_index].bit_mask.Byte.HLByte;
    TX_data[20] = (*(uint8_t*) (CMD_LIST[cmd_index].address + 1)) & CMD_LIST[cmd_index].bit_mask.Byte.LHByte;
    TX_data[21] = (*(uint8_t*) (CMD_LIST[cmd_index].address )) & CMD_LIST[cmd_index].bit_mask.Byte.LLByte;

    CMD_temp[cmd_index].changed_flag = 0;
    UART_TX_TCNT = 22;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_INFO(uint8_t info_index)
{
    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x62;
    TX_data[1] = INFO_LIST[info_index].signed_int;

    TX_data[2] = INFO_LIST[info_index].bit_mask.Byte.HHByte;
    TX_data[3] = INFO_LIST[info_index].bit_mask.Byte.HLByte;
    TX_data[4] = INFO_LIST[info_index].bit_mask.Byte.LHByte;
    TX_data[5] = INFO_LIST[info_index].bit_mask.Byte.LLByte;

    UART_TX_TCNT = 6;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_micro_fig(uint8_t micro_index)
{
    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x63;
    TX_data[1] = MICRO_LIST[micro_index].bit_mask.Byte.HHByte;
    TX_data[2] = MICRO_LIST[micro_index].bit_mask.Byte.HLByte;
    TX_data[3] = MICRO_LIST[micro_index].bit_mask.Byte.LHByte;
    TX_data[4] = MICRO_LIST[micro_index].bit_mask.Byte.LLByte;

    TX_data[5] = MICRO_LIST[micro_index].min_value.Byte.HHByte;
    TX_data[6] = MICRO_LIST[micro_index].min_value.Byte.HLByte;
    TX_data[7] = MICRO_LIST[micro_index].min_value.Byte.LHByte;
    TX_data[8] = MICRO_LIST[micro_index].min_value.Byte.LLByte;

    TX_data[9] = MICRO_LIST[micro_index].max_value.Byte.HHByte;
    TX_data[10] = MICRO_LIST[micro_index].max_value.Byte.HLByte;
    TX_data[11] = MICRO_LIST[micro_index].max_value.Byte.LHByte;
    TX_data[12] = MICRO_LIST[micro_index].max_value.Byte.LLByte;

    TX_data[13] = MICRO_LIST[micro_index].signed_int;
    UART_TX_TCNT = 14;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void initial_macro_fig(uint8_t macro_index)
{
    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x64;
    TX_data[1] = MACRO_LIST[macro_index].bit_mask.Byte.HHByte;
    TX_data[2] = MACRO_LIST[macro_index].bit_mask.Byte.HLByte;
    TX_data[3] = MACRO_LIST[macro_index].bit_mask.Byte.LHByte;
    TX_data[4] = MACRO_LIST[macro_index].bit_mask.Byte.LLByte;

    TX_data[5] = MACRO_LIST[macro_index].min_value.Byte.HHByte;
    TX_data[6] = MACRO_LIST[macro_index].min_value.Byte.HLByte;
    TX_data[7] = MACRO_LIST[macro_index].min_value.Byte.LHByte;
    TX_data[8] = MACRO_LIST[macro_index].min_value.Byte.LLByte;

    TX_data[9] = MACRO_LIST[macro_index].max_value.Byte.HHByte;
    TX_data[10] = MACRO_LIST[macro_index].max_value.Byte.HLByte;
    TX_data[11] = MACRO_LIST[macro_index].max_value.Byte.LHByte;
    TX_data[12] = MACRO_LIST[macro_index].max_value.Byte.LLByte;

    TX_data[13] = MACRO_LIST[macro_index].signed_int;
    UART_TX_TCNT = 14;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void RX_command(uint8_t rx_data[], CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp)
{
    uint8_t cmd_index = rx_data[1];

    switch(CMD_LIST[cmd_index].bit_mask.LONG)
    {
        case 0xFFFFFFFF:
            CMD_temp[cmd_index].data.Byte.HHByte = rx_data[3];
            CMD_temp[cmd_index].data.Byte.HLByte = rx_data[4];
        case 0x0000FFFF:
            CMD_temp[cmd_index].data.Byte.LHByte = rx_data[5];
        case 0x000000FF:
            CMD_temp[cmd_index].data.Byte.LLByte = rx_data[6];
    }
    CMD_temp[cmd_index].changed_flag = 1;
}

void RX_system_command(uint8_t rx_data[])
{
    uint8_t sys_index;

    sys_index = rx_data[1];

    switch(SYS_LIST[sys_index].bit_mask.LONG)
    {
        case 0xFFFFFFFF:
            SYS_temp[sys_index].data.Byte.HHByte = rx_data[2];
            SYS_temp[sys_index].data.Byte.HLByte = rx_data[3];
        case 0x0000FFFF:
            SYS_temp[sys_index].data.Byte.LHByte = rx_data[4];
        case 0x000000FF:
            SYS_temp[sys_index].data.Byte.LLByte = rx_data[5];
    }
    SYS_temp[sys_index].changed_flag = 1;
}
void Command_update()
{
    uint8_t cmd_index, sys_index;

    for(cmd_index=0;cmd_index<sizeof(GROUP0_CMD_LIST)/76;cmd_index++)
    {
        if (GROUP0_CMD_temp[cmd_index].changed_flag == 1)
        {
            switch(GROUP0_CMD_LIST[cmd_index].bit_mask.LONG)
            {
                case 0xFFFFFFFF:
                    (*(uint8_t*) (GROUP0_CMD_LIST[cmd_index].address + 3)) = GROUP0_CMD_temp[cmd_index].data.Byte.HHByte;
                    (*(uint8_t*) (GROUP0_CMD_LIST[cmd_index].address + 2)) = GROUP0_CMD_temp[cmd_index].data.Byte.HLByte;
                case 0x0000FFFF:
                    (*(uint8_t*) (GROUP0_CMD_LIST[cmd_index].address + 1)) = GROUP0_CMD_temp[cmd_index].data.Byte.LHByte;
                case 0x000000FF:
                    (*(uint8_t*) (GROUP0_CMD_LIST[cmd_index].address)) = GROUP0_CMD_temp[cmd_index].data.Byte.LLByte;
            }
            GROUP0_CMD_temp[cmd_index].changed_flag = 0;
        }
    }
    for(cmd_index=0;cmd_index<sizeof(GROUP1_CMD_LIST)/76;cmd_index++)
    {
        if (GROUP1_CMD_temp[cmd_index].changed_flag == 1)
        {
            switch(GROUP1_CMD_LIST[cmd_index].bit_mask.LONG)
            {
                case 0xFFFFFFFF:
                    (*(uint8_t*) (GROUP1_CMD_LIST[cmd_index].address + 3)) = GROUP1_CMD_temp[cmd_index].data.Byte.HHByte;
                    (*(uint8_t*) (GROUP1_CMD_LIST[cmd_index].address + 2)) = GROUP1_CMD_temp[cmd_index].data.Byte.HLByte;
                case 0x0000FFFF:
                    (*(uint8_t*) (GROUP1_CMD_LIST[cmd_index].address + 1)) = GROUP1_CMD_temp[cmd_index].data.Byte.LHByte;
                case 0x000000FF:
                    (*(uint8_t*) (GROUP1_CMD_LIST[cmd_index].address)) = GROUP1_CMD_temp[cmd_index].data.Byte.LLByte;
            }
            GROUP1_CMD_temp[cmd_index].changed_flag = 0;
        }
    }
    for(cmd_index=0;cmd_index<sizeof(GROUP2_CMD_LIST)/76;cmd_index++)
    {
        if (GROUP2_CMD_temp[cmd_index].changed_flag == 1)
        {
            switch(GROUP2_CMD_LIST[cmd_index].bit_mask.LONG)
            {
                case 0xFFFFFFFF:
                    (*(uint8_t*) (GROUP2_CMD_LIST[cmd_index].address + 3)) = GROUP2_CMD_temp[cmd_index].data.Byte.HHByte;
                    (*(uint8_t*) (GROUP2_CMD_LIST[cmd_index].address + 2)) = GROUP2_CMD_temp[cmd_index].data.Byte.HLByte;
                case 0x0000FFFF:
                    (*(uint8_t*) (GROUP2_CMD_LIST[cmd_index].address + 1)) = GROUP2_CMD_temp[cmd_index].data.Byte.LHByte;
                case 0x000000FF:
                    (*(uint8_t*) (GROUP2_CMD_LIST[cmd_index].address)) = GROUP2_CMD_temp[cmd_index].data.Byte.LLByte;
            }
            GROUP2_CMD_temp[cmd_index].changed_flag = 0;
        }
    }
    for(cmd_index=0;cmd_index<sizeof(GROUP3_CMD_LIST)/76;cmd_index++)
    {
        if (GROUP3_CMD_temp[cmd_index].changed_flag == 1)
        {
            switch(GROUP3_CMD_LIST[cmd_index].bit_mask.LONG)
            {
                case 0xFFFFFFFF:
                    (*(uint8_t*) (GROUP3_CMD_LIST[cmd_index].address + 3)) = GROUP3_CMD_temp[cmd_index].data.Byte.HHByte;
                    (*(uint8_t*) (GROUP3_CMD_LIST[cmd_index].address + 2)) = GROUP3_CMD_temp[cmd_index].data.Byte.HLByte;
                case 0x0000FFFF:
                    (*(uint8_t*) (GROUP3_CMD_LIST[cmd_index].address + 1)) = GROUP3_CMD_temp[cmd_index].data.Byte.LHByte;
                case 0x000000FF:
                    (*(uint8_t*) (GROUP3_CMD_LIST[cmd_index].address)) = GROUP3_CMD_temp[cmd_index].data.Byte.LLByte;
            }
            GROUP3_CMD_temp[cmd_index].changed_flag = 0;
        }
    }
    for(sys_index=0;sys_index<sizeof(SYS_LIST)/72;sys_index++)
    {
        if (SYS_temp[sys_index].changed_flag == 1)
        {
            switch(SYS_LIST[sys_index].bit_mask.LONG)
            {
                case 0xFFFFFFFF:
                    (*(uint8_t*) (SYS_LIST[sys_index].address + 3)) = SYS_temp[sys_index].data.Byte.HHByte;
                    (*(uint8_t*) (SYS_LIST[sys_index].address + 2)) = SYS_temp[sys_index].data.Byte.HLByte;
                case 0x0000FFFF:
                    (*(uint8_t*) (SYS_LIST[sys_index].address + 1)) = SYS_temp[sys_index].data.Byte.LHByte;
                case 0x000000FF:
                    (*(uint8_t*) (SYS_LIST[sys_index].address)) = SYS_temp[sys_index].data.Byte.LLByte;
            }
            SYS_temp[sys_index].changed_flag = 0;
        }
    }
}

void Update_microcosm_fig()
{
    uint8_t micro_index;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x30;
    for(micro_index=0;micro_index<sizeof(MICRO_LIST)/72;micro_index++)
    {
        TX_data[micro_index*2 + 1] = (*(uint8_t*) (MICRO_LIST[micro_index].address + 1)) & MICRO_LIST[micro_index].bit_mask.Byte.LHByte;
        TX_data[micro_index*2 + 2] = (*(uint8_t*) (MICRO_LIST[micro_index].address )) & MICRO_LIST[micro_index].bit_mask.Byte.LLByte;
    }
    UART_TX_TCNT = sizeof(MICRO_LIST)/72 * 2 + 1;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void Update_macrocosm_fig()
{
    uint8_t macro_index;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x31;
    for(macro_index=0;macro_index<sizeof(MACRO_LIST)/72;macro_index++)
    {
        TX_data[macro_index*2 + 1] = (*(uint8_t*) (MACRO_LIST[macro_index].address + 1)) & MACRO_LIST[macro_index].bit_mask.Byte.LHByte;
        TX_data[macro_index*2 + 2] = (*(uint8_t*) (MACRO_LIST[macro_index].address )) & MACRO_LIST[macro_index].bit_mask.Byte.LLByte;
    }

    UART_TX_TCNT = sizeof(MACRO_LIST)/72 * 2 + 1;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void TX_information()
{
    uint8_t info_index;

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x32;
    for(info_index=0;info_index<sizeof(INFO_LIST)/72;info_index++)
    {
        TX_data[info_index*2 + 1] = (*(uint8_t*) (INFO_LIST[info_index].address + 1)) & INFO_LIST[info_index].bit_mask.Byte.LHByte;
        TX_data[info_index*2 + 2] = (*(uint8_t*) (INFO_LIST[info_index].address )) & INFO_LIST[info_index].bit_mask.Byte.LLByte;
    }

    UART_TX_TCNT = 2 * sizeof(INFO_LIST)/72 + 1;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void Refresh(CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp, const uint8_t size)
{
    uint8_t cmd_index = 0;
    for (cmd_index = 0;cmd_index<size;cmd_index++)
    {
        while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);

        TX_data[cmd_index*4 + 0] = (*(uint8_t*) (CMD_LIST[cmd_index].address + 3)) & CMD_LIST[cmd_index].bit_mask.Byte.HHByte;
        TX_data[cmd_index*4 + 1] = (*(uint8_t*) (CMD_LIST[cmd_index].address + 2)) & CMD_LIST[cmd_index].bit_mask.Byte.HLByte;
        TX_data[cmd_index*4 + 2] = (*(uint8_t*) (CMD_LIST[cmd_index].address + 1)) & CMD_LIST[cmd_index].bit_mask.Byte.LHByte;
        TX_data[cmd_index*4 + 3] = (*(uint8_t*) (CMD_LIST[cmd_index].address )) & CMD_LIST[cmd_index].bit_mask.Byte.LLByte;
        CMD_temp[cmd_index].changed_flag = 0;
    }
    UART_TX_TCNT = size << 2;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

void Refresh_sys(void)
{
    uint8_t sys_index = 0;
    for (sys_index = 0;sys_index<sizeof(SYS_LIST)/72;sys_index++)
    {
        while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);

        TX_data[sys_index*4 + 0] = (*(uint8_t*) (SYS_LIST[sys_index].address + 3)) & SYS_LIST[sys_index].bit_mask.Byte.HHByte;
        TX_data[sys_index*4 + 1] = (*(uint8_t*) (SYS_LIST[sys_index].address + 2)) & SYS_LIST[sys_index].bit_mask.Byte.HLByte;
        TX_data[sys_index*4 + 2] = (*(uint8_t*) (SYS_LIST[sys_index].address + 1)) & SYS_LIST[sys_index].bit_mask.Byte.LHByte;
        TX_data[sys_index*4 + 3] = (*(uint8_t*) (SYS_LIST[sys_index].address )) & SYS_LIST[sys_index].bit_mask.Byte.LLByte;
        SYS_temp[sys_index].changed_flag = 0;
    }
    UART_TX_TCNT = sizeof(SYS_LIST)/72 << 2;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
}

uint32_t au32Config[2];
void Initialize_FMC()
{


  SYS_UnlockReg();
  ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL |= (0x1ul << (0));
  ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL |= (0x1ul << (4));

  au32Config[0] = FMC_Read(0x00300000UL);
  au32Config[1] = FMC_Read(0x00300000UL + 4);

  if((au32Config[0] == 0xFFFFFFFF) && (au32Config[1] == 0xFFFFFFFF))
  {
    FMC_Erase(0x00300000UL);
    FMC_Write(0x00300000UL, 0xFFFFFFFE );


    FMC_Erase(0x00300000UL + 4);
    FMC_Write(0x00300000UL + 4, (0x00010000UL - 512));

  }

  if((au32Config[0] != 0xFFFFFFFE ) && (au32Config[1] != (0x00010000UL - 512)))
  {
    FMC_Erase(0x00300000UL);
    FMC_Write(0x00300000UL, 0xFFFFFFFE);


    FMC_Erase(0x00300000UL + 4);
    FMC_Write(0x00300000UL + 4, (0x00010000UL - 512));

  }

  SYS_LockReg();
}

uint8_t data_flash_is_empty(void)
{
    uint8_t data_flash_empty_flag;
    uint8_t cmd_index, data_flash_empty_amount = 0;

    SYS_UnlockReg();
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL |= (0x1ul << (3));

    for(cmd_index = 0; cmd_index < sizeof(GROUP0_CMD_LIST)/76; cmd_index++)
    {
        if(FMC_Read((0x00010000UL - 512) + cmd_index * 4) == 0xFFFFFFFF) data_flash_empty_amount++;
    }
    for(cmd_index = 0; cmd_index < sizeof(GROUP1_CMD_LIST)/76; cmd_index++)
    {
        if(FMC_Read((0x00010000UL - 512) + (cmd_index + 20) * 4) == 0xFFFFFFFF) data_flash_empty_amount++;
    }
    for(cmd_index = 0; cmd_index < sizeof(GROUP2_CMD_LIST)/76; cmd_index++)
    {
        if(FMC_Read((0x00010000UL - 512) + (cmd_index + 40) * 4) == 0xFFFFFFFF) data_flash_empty_amount++;
    }
    for(cmd_index = 0; cmd_index < sizeof(GROUP3_CMD_LIST)/76; cmd_index++)
    {
        if(FMC_Read((0x00010000UL - 512) + (cmd_index + 60) * 4) == 0xFFFFFFFF) data_flash_empty_amount++;
    }

    if(data_flash_empty_amount == (sizeof(GROUP0_CMD_LIST)/76 + sizeof(GROUP1_CMD_LIST)/76 + sizeof(GROUP2_CMD_LIST)/76 + sizeof(GROUP3_CMD_LIST)/76))
        data_flash_empty_flag = 1;
    else
        data_flash_empty_flag = 0;

    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL &= ~(0x1ul << (3));
    SYS_LockReg();

    return (data_flash_empty_flag);
}

void read_data_flash_to_CMD_temp()
{
    VAR_T data_flash_temp;
    uint8_t cmd_index;

    SYS_UnlockReg();
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL |= (0x1ul << (3));

    for(cmd_index = 0; cmd_index < sizeof(GROUP0_CMD_LIST)/76; cmd_index++)
    {
        data_flash_temp.LONG = FMC_Read((0x00010000UL - 512) + cmd_index * 4);
        switch(GROUP0_CMD_LIST[cmd_index].bit_mask.LONG)
        {
            case 0xFFFFFFFF:
                GROUP0_CMD_temp[cmd_index].data.Word.HWord = data_flash_temp.Word.HWord;
            case 0x0000FFFF:
                GROUP0_CMD_temp[cmd_index].data.Byte.LHByte = data_flash_temp.Byte.LHByte;
            case 0x000000FF:
                GROUP0_CMD_temp[cmd_index].data.Byte.LLByte = data_flash_temp.Byte.LLByte;
        }
        GROUP0_CMD_temp[cmd_index].changed_flag = 1;
    }

    for(cmd_index = 0; cmd_index < sizeof(GROUP1_CMD_LIST)/76; cmd_index++)
    {
        data_flash_temp.LONG = FMC_Read((0x00010000UL - 512) + (cmd_index + 20) * 4);
        switch(GROUP1_CMD_LIST[cmd_index].bit_mask.LONG)
        {
            case 0xFFFFFFFF:
                GROUP1_CMD_temp[cmd_index].data.Word.HWord = data_flash_temp.Word.HWord;
            case 0x0000FFFF:
                GROUP1_CMD_temp[cmd_index].data.Byte.LHByte = data_flash_temp.Byte.LHByte;
            case 0x000000FF:
                GROUP1_CMD_temp[cmd_index].data.Byte.LLByte = data_flash_temp.Byte.LLByte;
        }
        GROUP1_CMD_temp[cmd_index].changed_flag = 1;
    }
    for(cmd_index = 0; cmd_index < sizeof(GROUP2_CMD_LIST)/76; cmd_index++)
    {
        data_flash_temp.LONG = FMC_Read((0x00010000UL - 512) + (cmd_index + 40) * 4);
        switch(GROUP2_CMD_LIST[cmd_index].bit_mask.LONG)
        {
            case 0xFFFFFFFF:
                GROUP2_CMD_temp[cmd_index].data.Word.HWord = data_flash_temp.Word.HWord;
            case 0x0000FFFF:
                GROUP2_CMD_temp[cmd_index].data.Byte.LHByte = data_flash_temp.Byte.LHByte;
            case 0x000000FF:
                GROUP2_CMD_temp[cmd_index].data.Byte.LLByte = data_flash_temp.Byte.LLByte;
        }
        GROUP2_CMD_temp[cmd_index].changed_flag = 1;
    }
    for(cmd_index = 0; cmd_index < sizeof(GROUP3_CMD_LIST)/76; cmd_index++)
    {
        data_flash_temp.LONG = FMC_Read((0x00010000UL - 512) + (cmd_index + 60) * 4);
        switch(GROUP1_CMD_LIST[cmd_index].bit_mask.LONG)
        {
            case 0xFFFFFFFF:
                GROUP3_CMD_temp[cmd_index].data.Word.HWord = data_flash_temp.Word.HWord;
            case 0x0000FFFF:
                GROUP3_CMD_temp[cmd_index].data.Byte.LHByte = data_flash_temp.Byte.LHByte;
            case 0x000000FF:
                GROUP3_CMD_temp[cmd_index].data.Byte.LLByte = data_flash_temp.Byte.LLByte;
        }
        GROUP3_CMD_temp[cmd_index].changed_flag = 1;
    }
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL &= ~(0x1ul << (3));
    SYS_LockReg();

}

void read_data_flash_to_GUI(uint8_t cmd_group, const uint8_t size)
{
    VAR_T data_flash_temp;
    uint8_t cmd_index;
    uint32_t EPWM_CTL_TEMP;
    EPWM_CTL_TEMP = ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL;
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL &= ~0x3F;
    NVIC_IRQ_TEMP = ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0];



        ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = 0xFFFFFFFF & ~(1UL << GDMA0_IRQn);

    SYS_UnlockReg();
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL |= (0x1ul << (3));

    for(cmd_index = 0; cmd_index < size; cmd_index++)
    {
        TX_data[cmd_index*6] = 0x40;
        TX_data[cmd_index*6 + 1] = cmd_index;

        data_flash_temp.LONG = FMC_Read((0x00010000UL - 512) + (cmd_index + cmd_group*20) * 4);
        TX_data[cmd_index*6 + 2] = data_flash_temp.Byte.HHByte;
        TX_data[cmd_index*6 + 3] = data_flash_temp.Byte.HLByte;
        TX_data[cmd_index*6 + 4] = data_flash_temp.Byte.LHByte;
        TX_data[cmd_index*6 + 5] = data_flash_temp.Byte.LLByte;
    }

    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL &= ~(0x1ul << (3));
    SYS_LockReg();

    UART_TX_TCNT = 6 * size;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = NVIC_IRQ_TEMP;
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL = EPWM_CTL_TEMP;
}


void write_data_flash_from_GUI(CMD_TypeDef* CMD_LIST, Temp_T* CMD_temp, uint8_t cmd_group, const uint8_t size)
{
    uint8_t cmd_index;
    uint32_t EPWM_CTL_TEMP;
    EPWM_CTL_TEMP = ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL;
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL &= ~0x3F;
    NVIC_IRQ_TEMP = ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0];



        ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = 0xFFFFFFFF & ~(1UL << GDMA0_IRQn);

    SYS_UnlockReg();
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL |= (0x1ul << (3));


    for(cmd_index = 0; cmd_index < size; cmd_index++)
    {
        FMC_Write((0x00010000UL - 512) + (cmd_index + 20*cmd_group) * 4, CMD_temp[cmd_index].data.LONG);
        CMD_temp[cmd_index].changed_flag = 1;
    }

    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL &= ~(0x1ul << (3));
    SYS_LockReg();
    __enable_irq();

    while((((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->BUFSTS & (0x1ul << (8))) == 0);
    TX_data[0] = 0x41;
    UART_TX_TCNT = 1;
    { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = NVIC_IRQ_TEMP;
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL = EPWM_CTL_TEMP;
}

void erase_data_flash()
{
    int32_t FMC_Erase_status;
    uint32_t EPWM_CTL_TEMP;
    EPWM_CTL_TEMP = ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL;
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL &= ~0x3F;
    NVIC_IRQ_TEMP = ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0];



        ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = 0xFFFFFFFF & ~(1UL << GDMA0_IRQn);

    SYS_UnlockReg();
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL |= (0x1ul << (3));

    FMC_Erase_status = FMC_Erase((0x00010000UL - 512));

    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCTL &= ~(0x1ul << (3));
    SYS_LockReg();
    __enable_irq();
    if (FMC_Erase_status == 0) {
        TX_data[0] = 0x42;
        UART_TX_TCNT = 1;
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };

    }
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = NVIC_IRQ_TEMP;
    ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL = EPWM_CTL_TEMP;
}

void Load_from_data_flash()
{
    if (data_flash_is_empty() == 0) {
        read_data_flash_to_CMD_temp();
    }
}

void Update_time_scale(uint8_t rx_data[])
{
    VAR_T time_scale_cnt_temp;

    time_scale_cnt_temp.Byte.HHByte = rx_data[1];
    time_scale_cnt_temp.Byte.HLByte = rx_data[2];
    time_scale_cnt_temp.Byte.LHByte = rx_data[3];
    time_scale_cnt_temp.Byte.LLByte = rx_data[4];

    TIME_SCALE_CNT = time_scale_cnt_temp.LONG;
    Update_microcosm_fig_req = 0;
    Update_microcosm_fig_trigger = 0;
    MICRO_TX_CNT = 0;
}
void TX_data_to_PC(int16_t rotor_speed, uint16_t min_cmd_speed_rpm)
{
    rotor_speed = ((rotor_speed)>=0 ? (rotor_speed) : -(rotor_speed));


    if ((rotor_speed>=0)&&(rotor_speed < min_cmd_speed_rpm)&&(Update_microcosm_fig_req==1)) {
            Update_microcosm_fig_trigger = 1;
        }
    if (Update_microcosm_fig_trigger == 1){

        Update_microcosm_fig();
        MICRO_TX_CNT ++;
        if (MICRO_TX_CNT == TIME_SCALE_CNT)
        {
            Update_microcosm_fig_req = 0;
            Update_microcosm_fig_trigger = 0;
            MICRO_TX_CNT = 0;
        }
    }
    else if (Update_macrocosm_fig_req == 1) {
        Update_macrocosm_fig_req = 0;
        Update_macrocosm_fig();
    }
    else if (Tx_information_req == 1) {
        Tx_information_req = 0;
        TX_information();
    }
    else if (Refresh_group_req.flag == 1) {
        Refresh_group_req.flag = 0;
        Refresh((CMD_TypeDef *)GROUP_CMD_BASE[Refresh_group_req.cmd_group], (Temp_T *)GROUP_TEMP_BASE[Refresh_group_req.cmd_group], GROUP_CMD_SIZE[Refresh_group_req.cmd_group]);
    }
    else if (Refresh_sys_req == 1) {
        Refresh_sys_req = 0;
        Refresh_sys();
    }
}

void Trigger_update_micro_fig()
{
    if (Update_microcosm_fig_req == 1)
        Update_microcosm_fig_trigger = 1;
}

void Request_decode(uint8_t rx_data[])
{
    if ((rx_data[0] == 0x30) && (Update_microcosm_fig_trigger == 0)) {
        Update_microcosm_fig_req = 1;
    }
    else if (rx_data[0] == 0x31) {
        Update_macrocosm_fig_req = 1;
    }
    else if (rx_data[0] == 0x32) {
        Tx_information_req = 1;
    }
    else if (rx_data[0] == 0x33) {
        Update_time_scale(rx_data);
    }
    else if (rx_data[0] == 0x34) {
        RX_command(rx_data, (CMD_TypeDef *)GROUP_CMD_BASE[rx_data[2]], (Temp_T *)GROUP_TEMP_BASE[rx_data[2]]);
    }
    else if (rx_data[0] == 0x35) {
        RX_system_command(rx_data);
    }
    else if (rx_data[0] == 0x36) {
        Refresh_group_req.flag = 1;
        Refresh_group_req.cmd_group = rx_data[1];
    }
    else if (rx_data[0] == 0x37) {
        Refresh_sys_req = 1;
    }
    else if (rx_data[0] == 0x40) {
        read_data_flash_to_GUI(rx_data[1], GROUP_CMD_SIZE[rx_data[1]]);
    }
    else if (rx_data[0] == 0x41) {
        write_data_flash_from_GUI((CMD_TypeDef *)GROUP_CMD_BASE[rx_data[1]], (Temp_T *)GROUP_TEMP_BASE[rx_data[1]], rx_data[1], GROUP_CMD_SIZE[rx_data[1]]);
    }
    else if (rx_data[0] == 0x42) {
        erase_data_flash();
    }
    else if (rx_data[0] == 0x70) {
        PWM_CTL_TEMP = ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL;
        ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL &= ~0x3F;
        NVIC_IRQ_TEMP = ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0];



            ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = 0xFFFFFFFF & ~(1UL << GDMA0_IRQn);

        TX_data[0] = 0x70;
        UART_TX_TCNT = 2;
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
    }
    else if (rx_data[0] == 0x50) {
        initial_SYS_name(rx_data[1]);
    }
    else if (rx_data[0] == 0x51) {
        initial_CMD_name(rx_data[1], (CMD_TypeDef *)GROUP_CMD_BASE[rx_data[2]], GROUP_CMD_SIZE[rx_data[2]]);
    }
    else if (rx_data[0] == 0x52) {
        initial_INFO_name(rx_data[1]);
    }
    else if (rx_data[0] == 0x53) {
        initial_MICRO_name(rx_data[1]);
    }
    else if (rx_data[0] == 0x54) {
        initial_MACRO_name(rx_data[1]);
    }
    else if (rx_data[0] == 0x60) {
        initial_SYS(rx_data[1]);
    }
    else if (rx_data[0] == 0x61) {
        initial_CMD(rx_data[1], (CMD_TypeDef *)GROUP_CMD_BASE[rx_data[2]], (Temp_T *)GROUP_TEMP_BASE[rx_data[2]]);
    }
    else if (rx_data[0] == 0x62) {
        initial_INFO(rx_data[1]);
    }
    else if (rx_data[0] == 0x63) {
        initial_micro_fig(rx_data[1]);
    }
    else if (rx_data[0] == 0x64) {
        initial_macro_fig(rx_data[1]);
    }
    else if (rx_data[0] == 0x55) {
        TX_data[0] = 0x55;
        UART_TX_TCNT = 1;
        { ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->TCNT = UART_TX_TCNT; (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL |= (0x1ul << (0))); };
        ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = NVIC_IRQ_TEMP;
        ((EPWM_T *) ((( uint32_t)0x40000000) + 0x40000))->CTL = PWM_CTL_TEMP;
    }
    else if (rx_data[0] == 0x88) {
        SYS_UnlockReg();
        ((SYS_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRST0 = (0x1ul << (0));
        SYS_LockReg();
    }
}

void UART_TX_MFP(GPIO_T *port, uint8_t pin)
{
    uint32_t port_offset = ((uint32_t)&port->MODE - ((( uint32_t)0x50000000) + 0x04000)) >> 4;
    uint32_t field_len = 4;
    *(uint32_t *) (((( uint32_t)0x50000000) + 0x00000) + 0x30 + port_offset) &= ~(0xF << (pin*field_len));
    *(uint32_t *) (((( uint32_t)0x50000000) + 0x00000) + 0x30 + port_offset) |= (0xB << (pin*field_len));
    field_len = 2;
    port->MODE = (port->MODE & ~(0x3 << (pin*field_len))) | (0x1UL << (pin*field_len));
}

void UART_RX_MFP(GPIO_T *port, uint32_t pin)
{
    uint32_t port_offset = ((uint32_t)&port->MODE - ((( uint32_t)0x50000000) + 0x04000)) >> 4;
    uint32_t field_len = 4;
    *(uint32_t *) (((( uint32_t)0x50000000) + 0x00000) + 0x30 + port_offset) &= ~(0xF << (pin*field_len));
    *(uint32_t *) (((( uint32_t)0x50000000) + 0x00000) + 0x30 + port_offset) |= (0xB << (pin*field_len));
    field_len = 2;
    port->MODE = (port->MODE & ~(0x3 << (pin*field_len))) | (0x0UL << (pin*field_len));
}

void Initialize_USCI_UART(void)
{

    ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK = ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->APBCLK | (1<<(24+2));


    ((SYS_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRST1 |= (1<<(24+2));
    ((SYS_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRST1 &= ~(1<<(24+2));


    UUART_Open(((UUART_T *) ((( uint32_t)0x40200000) + 0x70000)), 4000000);


    UART_TX_MFP(((GPIO_T *) (((( uint32_t)0x50000000) + 0x04000) + 0x000C0)), 7);
    UART_RX_MFP(((GPIO_T *) (((( uint32_t)0x50000000) + 0x04000) + 0x00100)), 0);


        Initialize_GDMA();

}
# 1062 "program/protocol.c"
    void GDMA0_IRQHandler (void)
    {

        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL &= ~(0x1ul << (18));
        (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL |= (0x1ul << (0)));
        Request_decode(RX_data);

    }

    void GDMA1_IRQHandler (void)
    {

        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL &= ~(0x1ul << (18));

    }

    void Initialize_GDMA(void)
    {

        ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->AHBCLK |= (0x1ul << (8));


        SYS_UnlockReg();
        ((SYS_T *) ((( uint32_t)0x50000000) + 0x00000))->IPRST0 |= (0x1ul << (2));
        SYS_LockReg();


        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL = (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL & ~((0x3ul << (12)) | (0x1ul << (9)))) | ((0x0UL) << (12));
        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL = (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL & ~((0x3ul << (12)) | (0x1ul << (9)))) | ((0x0UL) << (12));


        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL = (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL & ~((0x3ul << (2)))) | ((0x1UL) << (2));
        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL = (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL & ~((0x3ul << (2)))) | ((0x1UL) << (2));


        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL = (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL & ~(0xF << (4))) |
                     (((0x8UL) | (0x0UL)) << (4));
        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL = (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->CTL & ~(0xF << (4))) |
                     (((0x0UL) | (0x4UL)) << (4));


        (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->TCNT = 7);


        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->DSTB = (uint32_t)&RX_data[0];
        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->DSTB = (uint32_t)&((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->TXDAT;


        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->SRCB = (uint32_t)&((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->RXDAT;
        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08020))->SRCB = (uint32_t)&TX_data[0];


        ((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->DMACTL |= (0x1ul << (0));


        ((UUART_T *) ((( uint32_t)0x40200000) + 0x70000))->DMACTL |= (0x1ul << (3)) | (0x1ul << (2)) | (0x1ul << (1));





        ((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL |= (0x1ul << (8));


        (((GDMA_T *) ((( uint32_t)0x50000000) + 0x08000))->CTL |= (0x1ul << (0)));
    }
