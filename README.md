# NM1240 Motor Demo Code (Keil V6)

## 📋 專案簡介

本專案為 **Nuvoton NM1240** 微控制器的馬達控制示範程式碼，使用 **Keil MDK V6** 開發環境。

NM1240 是一款基於 ARM Cortex-M0 核心的馬達控制專用微控制器，適用於 BLDC（無刷直流馬達）和 PMSM（永磁同步馬達）的驅動應用。

## 📁 專案結構

```
SampleCode.7z          # 壓縮檔，包含完整的示範程式碼
SampleCode/            # 已解壓縮的原始碼
CONTRIBUTING.md        # 貢獻指南與下一步建議
```

### 原始碼說明（`SampleCode/SampleCode/UserProj/program/`）

| 檔案 | 說明 |
|------|------|
| `main.c` | 主程式進入點，系統初始化與主迴圈 |
| `system_parameter.H` | 系統參數定義（電壓、電流、PWM 頻率等） |
| `system_initialize.C` | 系統週邊初始化（PWM、ADC、GPIO） |
| `variable_initialize.c` | 變數初始化 |
| `variable_typedefine.h` | 變數型別定義 |
| `INT_ISR.C` | 中斷服務程式（PWM、ADC 中斷處理） |
| `motor_funtions.c` | 馬達控制核心函式 |
| `motor_FOC.c` | FOC（磁場導向控制）演算法 |
| `motor_six_step.c` | 六步換相控制 |
| `PI_control.c` | PI 控制器實作 |
| `protocol.c` | 通訊協定處理 |

## 🛠️ 開發環境需求

- **IDE**: Keil MDK V6（或更新版本）
- **目標晶片**: Nuvoton NM1240（ARM Cortex-M0）
- **程式語言**: C
- **除錯器**: Nu-Link 或相容的 SWD 除(debug symbol ends)

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
   - 連接 Nu-Link 除debug symbol ends)
   - 點選 Download 按鈕燒錄至 NM1240

## ⚙️ 關鍵參數設定

主要參數位於 `system_parameter.H`，可根據實際馬達規格調整：

- **PWM 頻率**: 依應用需求設定
- **電壓 / 電流限制**: 根據馬達與驅動板規格
- **控制模式**: 支援速度環、電流環控制

## 🎯 下一步 (Next Steps)

完成快速開始後，建議您依以下順序進行：

1. **閱讀原始碼** — 從 `system_parameter.H` 開始，了解馬達參數設定
2. **理解控制架構** — 閱讀 `main.c` 與 `INT_ISR.C`，理解主迴圈與中斷流程
3. **學習 FOC 演算法** — 閱讀 `motor_FOC.c` 與 `PI_control.c`，理解磁場導向控制
4. **調整參數** — 根據您的馬達規格，修改 `system_parameter.H` 中的參數
5. **硬體測試** — 連接 NM1240 開發板、驅動板與馬達，進行實機測試

> 📖 更詳細的步驟說明請參考 [CONTRIBUTING.md](CONTRIBUTING.md)

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