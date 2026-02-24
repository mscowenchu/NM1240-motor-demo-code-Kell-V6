# 1 "C:/Users/JPChang0/Desktop/nuvoTon/!Project-NuMotor Solution/!Development of NM1244/[Example Code] NuMotor NM1240_Rev4.1_20250806/SW/Library/Device/Nuvoton/NM1240/Source/system_NM1240.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 393 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:/Users/JPChang0/Desktop/nuvoTon/!Project-NuMotor Solution/!Development of NM1244/[Example Code] NuMotor NM1240_Rev4.1_20250806/SW/Library/Device/Nuvoton/NM1240/Source/system_NM1240.c" 2
# 12 "C:/Users/JPChang0/Desktop/nuvoTon/!Project-NuMotor Solution/!Development of NM1244/[Example Code] NuMotor NM1240_Rev4.1_20250806/SW/Library/Device/Nuvoton/NM1240/Source/system_NM1240.c"
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
# 13 "C:/Users/JPChang0/Desktop/nuvoTon/!Project-NuMotor Solution/!Development of NM1244/[Example Code] NuMotor NM1240_Rev4.1_20250806/SW/Library/Device/Nuvoton/NM1240/Source/system_NM1240.c" 2
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
# 14 "C:/Users/JPChang0/Desktop/nuvoTon/!Project-NuMotor Solution/!Development of NM1244/[Example Code] NuMotor NM1240_Rev4.1_20250806/SW/Library/Device/Nuvoton/NM1240/Source/system_NM1240.c" 2







uint32_t __HIRC = 48000000;
uint32_t SystemCoreClock = 48000000;
uint32_t CyclesPerUs = (48000000 / 1000000);






void SystemCoreClockUpdate(void)
{
    uint32_t u32Freq, u32ClkSrc;
    uint32_t u32HclkDiv;

    __HIRC = (((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PWRCTL & (0x1ul << (24))) ? 60000000 : 48000000;
    u32ClkSrc = (((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKSEL0 & (0x3ul << (0))) >> (0);
    if (u32ClkSrc == 0)
    {
        u32Freq = (24000000UL);
    }
    else if (u32ClkSrc == 1)
    {
        u32Freq = ( 10000UL);
    }
    else if (u32ClkSrc == 3)
    {
        u32Freq = __HIRC;
    }
    else
    {
        u32Freq = 0;
    }
    u32HclkDiv = ((((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->CLKDIV & (0xful << (0))) >> (0)) + 1;


    SystemCoreClock = u32Freq / u32HclkDiv;

    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;




}
# 78 "C:/Users/JPChang0/Desktop/nuvoTon/!Project-NuMotor Solution/!Development of NM1244/[Example Code] NuMotor NM1240_Rev4.1_20250806/SW/Library/Device/Nuvoton/NM1240/Source/system_NM1240.c"
void SystemInit(void)
{
}
