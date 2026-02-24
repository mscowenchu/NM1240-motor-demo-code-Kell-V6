# NM1240 Motor Demo Code (Keil V6)

## 📋 專案簡介

本專案為 **Nuvoton NM1240** 微控制器的馬達控制示範程式碼，使用 **Keil MDK V6** 開發環境。

NM1240 是一款基於 ARM Cortex-M0 核心的馬達控制專用微控制器，適用於 BLDC（無刷直流馬達）和 PMSM（永磁同步馬達）的驅動應用。

## 📁 專案結構

```
SampleCode/
├── Hard_Fault_Sample/     # Hard Fault 處理範例
│   ├── IAR/               # IAR Embedded Workbench 專案檔
│   ├── KEIL/              # Keil MDK 專案檔
│   └── main.c
├── Template/              # 空白範本專案
│   ├── IAR/               # IAR Embedded Workbench 專案檔
│   ├── Keil/              # Keil MDK 專案檔
│   └── main.c
└── UserProj/              # 馬達控制主專案
    ├── program/           # 原始碼
    ├── RTE/               # Run-Time Environment 設定
    ├── NM1240_1R_2R_6stp_Hall_FOC.uvprojx  # Keil V6 專案檔
    └── Nu_Link_Driver.ini                  # Nu-Link 除錯器設定
```

### 原始碼說明（UserProj/program/）

| 檔案 | 說明 |
|------|------|
| `main.c` | 主程式進入點，系統初始化與主迴圈 |
| `system_parameter.H` | 系統參數定義（電壓、電流、PWM 頻率等） |
| `system_initialize.C / .H` | 系統初始化（時鐘、GPIO、ADC、PWM 等） |
| `variable_initialize.c` | 變數初始化 |
| `variable_typedefine.h` | 型別定義 |
| `motor_funtions.c` | 馬達控制核心函式 |
| `motor_functions.h` | 馬達控制函式標頭檔 |
| `motor_FOC.c / .h` | FOC 演算法實作 |
| `motor_six_step.c / .h` | 六步換相控制 |
| `PI_control.c / .h` | PI 控制器 |
| `INT_ISR.C` | 中斷服務程式（PWM、ADC 中斷處理） |
| `protocol.c / .h` | UART 通訊協定 |
| `DataFlash.c / .h` | 資料快閃記憶體操作 |
| `svpwm.h` | SVPWM 標頭檔 |
| `svpwm_v02.lib` | SVPWM 函式庫 |
| `temp.c` | 溫度處理 |

## 🛠️ 開發環境需求

- **IDE**: Keil MDK V6（或更新版本）
- **目標晶片**: Nuvoton NM1240 系列（NM1244D48）
- **程式語言**: C
- **除錯器**: Nu-Link 或相容的 SWD 除錯器
- **時脈**: 內部 HIRC 60 MHz

## 🚀 快速開始

1. **下載專案**
   ```bash
   git clone https://github.com/mscowenchu/NM1240-motor-demo-code-Kell-V6.git
   ```

2. **開啟專案**
   - 使用 Keil MDK V6 開啟 `SampleCode/UserProj/NM1240_1R_2R_6stp_Hall_FOC.uvprojx`

3. **編譯與燒錄**
   - 點選 Keil 的 Build 按鈕編譯專案
   - 連接 Nu-Link 除錯器
   - 點選 Download 按鈕燒錄至 NM1240

## ⚙️ 關鍵參數設定

主要參數位於 `system_parameter.H`，可根據實際馬達規格調整：

- **PWM 頻率**: 依應用需求設定
- **電壓 / 電流限制**: 根據馬達與驅動板規格
- **控制模式**: 支援速度環、電流環控制

## ✨ 功能特色

- FOC（磁場導向控制）演算法
- 六步換相控制模式
- 3 Hall 感測器輸入（ECAP）
- 2 分流電阻電流感測（ADC）
- PI 速度控制器
- UART 通訊協定
- 資料快閃記憶體參數儲存
- SVPWM（空間向量脈寬調變）

## 📝 注意事項

- 本專案使用 **Q15 / Q26 定點數格式** 進行數學運算
- 標頭檔使用**大寫副檔名**（如 `.H`）
- 修改參數前請確認馬達規格，避免損壞硬體

## 📄 授權

本專案僅供學習與參考使用。

## 🔗 相關連結

- [Nuvoton 官方網站](https://www.nuvoton.com/)
- [NM1240 產品頁面](https://www.nuvoton.com/products/microcontrollers/arm-cortex-m0-mcus/)
- [Keil MDK 下載](https://www.keil.com/download/product/)
