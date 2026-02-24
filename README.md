# NM1240 Motor Demo Code (Keil V6)

> **English Summary**: This repository contains BLDC fan motor control demo code for the Nuvoton NM1240 MCU (ARM Cortex-M0, 60 MHz), built with Keil MDK-ARM V6. It implements FOC (Field-Oriented Control) with 3 Hall sensors, supporting 2-shunt FOC, 1-shunt FOC, and six-step commutation modes. Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.

---

## 📋 專案簡介

本專案為 **Nuvoton NM1240** 微控制器（ARM Cortex-M0，60 MHz）的 **BLDC 風扇馬達控制**示範程式碼，採用 **FOC（磁場導向控制）**演算法搭配 3 個 Hall 感測器，使用 **Keil MDK-ARM V6** 工具鏈開發。

版權所有：Nuvoton Technology Corp. © 2021

---

## ⚙️ 支援的控制模式

透過 `system_parameter.H` 中的編譯巨集選擇控制模式（三擇一）：

| 模式 | 巨集設定 | 說明 |
|------|---------|------|
| **FOC + 2 分流電阻**（預設啟用） | `P_FOC_CONTROL=1` + `#define P_2R_FOC` | 雙電阻取樣 FOC |
| **FOC + 1 分流電阻** | `P_FOC_CONTROL=1` + `#define P_1R_FOC` | 單電阻取樣 FOC |
| **六步換相** | `P_SIX_STEP_CONTROL=1` | 傳統六步控制 |

---

## 📁 專案檔案結構

```
SampleCode/SampleCode/
├── UserProj/                         ← 主要馬達控制專案
│   ├── NM1240_1R_2R_6stp_Hall_FOC.uvprojx  ← Keil 專案檔
│   ├── program/                      ← 核心原始碼（21 個檔案）
│   │   ├── main.c                    — 主程式入口、系統初始化流程、錯誤 LED 處理
│   │   ├── system_initialize.C       — 所有 MCU 週邊初始化（Clock、GPIO、ADC、EPWM、ECAP、Timer、UART、FMC 等）
│   │   ├── system_parameter.H        — 系統參數（PWM 頻率、電壓/電流限制、PI 參數、Hall 對應表）
│   │   ├── INT_ISR.C                 — 中斷服務程式（PWM ISR → FOC 運算、ADC ISR、Timer0 ISR、Hall ISR）
│   │   ├── motor_FOC.c              — FOC 核心演算法：Clark/Park 轉換、_Iuvw_to_Idq()、Vdq_to_SVPWM_2R/1R()
│   │   ├── motor_FOC.h              — FOC inline 函式：SIN()/COS() 查表、Update_Hall_Angle()、三相電流重建
│   │   ├── motor_funtions.c         — 馬達功能：Hall 位置偵測、轉速估算、VR 讀取、啟動/停止控制
│   │   ├── motor_functions.h        — 馬達功能宣告、GPIO 腳位定義（LED、DIR、測試腳）
│   │   ├── motor_six_step.c/h       — 六步換相邏輯
│   │   ├── PI_control.c/h           — PI 控制器（速度環、Id/Iq 電流環）
│   │   ├── protocol.c/h             — UART 通訊協定（供 PC GUI 工具使用）
│   │   ├── variable_typedefine.h    — 結構定義（AMotor 含 cmd、info、ctrl、other 子結構）
│   │   ├── variable_initialize.c    — 全域變數初始化
│   │   ├── DataFlash.c/h            — Data Flash 讀寫（參數儲存/載入）
│   │   ├── temp.c                   — 溫度感測器讀取
│   │   ├── svpwm.h                  — SVPWM 函式宣告
│   │   ├── svpwm_v02.lib            — 預編譯 SVPWM 函式庫（由 Nuvoton 提供）
│   │   └── system_initialize.H      — 系統初始化函式宣告
│   ├── lst/                         — 編譯器列表檔（建置產物）
│   └── obj/                         — 編譯器輸出檔（建置產物）
├── Template/                        — NM1240 基礎範本專案（UART printf 示範）
└── Hard_Fault_Sample/               — Hard Fault 錯誤處理示範
```

---

## 🔄 FOC 演算法流程

```
ADC 取樣 (Iu, Iv)
     │
     ▼
Re_Construct_3_Phase_Current()  → 重建三相電流 (Iu, Iv, Iw)
     │
     ▼
_Iuvw_to_Idq()                → Clark + Park 轉換 → 取得 Id、Iq
     │
     ▼
PI_Id_current() → Vd           → d 軸電流環 PI 控制
PI_Iq_current() → Vq           → q 軸電流環 PI 控制
     │
     ▼
Vdq_to_SVPWM()                → 反 Park + 反 Clark + SVPWM → 更新 PWM 占空比
```

---

## 📊 關鍵系統參數（來源：`system_parameter.H`）

| 參數 | 數值 | 說明 |
|------|------|------|
| SYSTEM_CLOCK | 60 MHz | MCU 時鐘（HIRC） |
| C_MOTOR_PWM_FREQ | 15 kHz | PWM 切換頻率 |
| C_MOTOR_POLE | 4 | 馬達極數 |
| C_MAX_MOTOR_SPEED | 3000 rpm | 最大轉子轉速 |
| C_Tdead | 1 μs | 死區時間 |
| SHUNT_R_mOhm | 100 mΩ | 分流電阻值 |
| C_DCBUS_Voltage | 24 V | 直流母線電壓 |
| VDD | 5 V | MCU 供電電壓 |
| C_I_Kp | 500 | 電流環比例增益 |
| C_I_Ki | 50 | 電流環積分增益 |
| C_SP_Kp | 163839 | 速度環比例增益 |
| C_SP_Ki | 3.28 | 速度環積分增益 |

---

## 📐 數學格式說明

- 採用 **Q15 定點數**處理電流／電壓（Cortex-M0 無 FPU）
- 採用 **Q26 定點數**進行角度累積
- SIN 查表：1024 筆，Q15 值域（-32767 ~ +32767）
- Clark 轉換常數：18918/32768 ≈ 1/√3

---

## 🔌 硬體腳位對應（關鍵腳位）

| 功能 | 腳位 | 說明 |
|------|------|------|
| Hall U (IC0) | PF0 | Hall 感測器 U 相輸入 |
| Hall V (IC1) | PF1 | Hall 感測器 V 相輸入 |
| Hall W (IC2) | PF2 | Hall 感測器 W 相輸入 |
| UART TX | PD7 | UART2 傳送 |
| UART RX | PE0 | UART2 接收 |
| LED | PF4 | 狀態／錯誤 LED |
| DIR Switch | PA6 | 馬達方向命令 |
| DAC CS | PD6 | 外部 DAC 晶片選擇 |

---

## 🚀 建置步驟

1. 以 **Keil MDK-ARM** 開啟 `SampleCode/SampleCode/UserProj/NM1240_1R_2R_6stp_Hall_FOC.uvprojx`
2. 在 `system_parameter.H` 中選擇控制模式（FOC 2R／FOC 1R／六步換相）
3. 按 **F7** 建置，並透過 **Nu-Link** 燒錄至 NM1240 示範板

---

## 📝 注意事項

- 標頭檔使用**大寫副檔名**（如 `.H`、`.C`），此為本專案慣例
- 修改參數前請確認馬達規格，避免損壞硬體
- `obj/` 與 `lst/` 為建置產物目錄，已列入 `.gitignore`

---

## 📄 授權

Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.

## 🔗 相關連結

- [Nuvoton 官方網站](https://www.nuvoton.com/)
- [NM1240 產品頁面](https://www.nuvoton.com/products/microcontrollers/arm-cortex-m0-mcus/)
- [Keil MDK 下載](https://www.keil.com/download/product/)