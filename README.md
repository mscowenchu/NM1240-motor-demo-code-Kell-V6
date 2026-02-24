# NM1240 Motor Demo Code (Keil V6)

## 📋 專案簡介

本專案為 **Nuvoton NM1240** 微控制器的馬達控制示範程式碼，使用 **Keil MDK V6** 開發環境。

NM1240 是一款基於 ARM Cortex-M0 核心的馬達控制專用微控制器，適用於 BLDC（無刷直流馬達）和 PMSM（永磁同步馬達）的驅動應用。

## 📁 專案結構

```
SampleCode.7z                          # 壓縮檔，包含完整的示範程式碼
SampleCode/SampleCode/UserProj/        # 主要使用者專案
SampleCode/SampleCode/Hard_Fault_Sample/  # Hard Fault 示範專案
SampleCode/SampleCode/Template/        # 專案範本
```

### 原始碼說明（`SampleCode/SampleCode/UserProj/program/`）

| 檔案 | 說明 |
|------|------|
| `main.c` | 主程式進入點，系統初始化與主迴圈 |
| `system_parameter.H` | 系統參數定義（電壓、電流、PWM 頻率等） |
| `system_initialize.C` / `system_initialize.H` | 系統初始化（時脈、周邊設定） |
| `variable_initialize.c` / `variable_typedefine.h` | 變數初始化與型別定義 |
| `motor_funtions.c` / `motor_functions.h` | 馬達控制核心函式 |
| `motor_FOC.c` / `motor_FOC.h` | FOC（磁場導向控制）演算法 |
| `motor_six_step.c` / `motor_six_step.h` | 六步換相控制 |
| `INT_ISR.C` | 中斷服務程式（PWM、ADC 中斷處理） |
| `PI_control.c` / `PI_control.h` | PI 控制器 |
| `protocol.c` / `protocol.h` | 通訊協定 |
| `DataFlash.c` / `DataFlash.h` | 資料快閃記憶體存取 |
| `svpwm.h` / `svpwm_v02.lib` | 空間向量 PWM 函式庫 |

## 🛠️ 開發環境需求

- **IDE**: Keil MDK V6（或更新版本）
- **目標晶片**: Nuvoton NM1240（ARM Cortex-M0）
- **程式語言**: C
- **除錯器**: Nu-Link 或相容的 SWD 除錯器

## 🚀 快速開始

1. **下載專案**
   ```bash
   git clone https://github.com/mscowenchu/NM1240-motor-demo-code-Kell-V6.git
   ```

2. **解壓縮原始碼**
   - 使用 [7-Zip](https://www.7-zip.org/) 解壓縮 `SampleCode.7z`

3. **開啟專案**
   - 使用 Keil MDK V6 開啟解壓縮後的專案檔（`.uvprojx`）

4. **編譯與燒錄**
   - 點選 Keil 的 Build 按鈕編譯專案
   - 連接 Nu-Link 除錯器
   - 點選 Download 按鈕燒錄至 NM1240

## ⚙️ 關鍵參數設定

主要參數位於 `system_parameter.H`，可根據實際馬達規格調整：

- **PWM 頻率**: 依應用需求設定
- **電壓 / 電流限制**: 根據馬達與驅動板規格
- **控制模式**: 支援速度環、電流環控制

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