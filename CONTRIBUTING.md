# 貢獻指南 (Contributing Guide)

感謝您對 NM1240 馬達控制示範專案的興趣！以下是參與貢獻的指南。

## 🚀 下一步建議

### 1. 環境設定
- 安裝 [Keil MDK V6](https://www.keil.com/download/product/) 或更新版本
- 安裝 [Nu-Link 驅動程式](https://www.nuvoton.com/) 以進行除錯與燒錄
- 使用 7-Zip 解壓縮 `SampleCode.7z`（若尚未解壓縮）

### 2. 開啟與編譯專案
- 在 Keil MDK 中開啟 `SampleCode/SampleCode/UserProj/NM1240_1R_2R_6stp_Hall_FOC.uvprojx`
- 點選 **Build** (F7) 編譯專案
- 確認編譯無錯誤

### 3. 理解程式碼架構
建議依以下順序閱讀原始碼：

1. **`system_parameter.H`** — 系統參數定義，了解馬達規格設定
2. **`variable_typedefine.h`** — 變數型別定義
3. **`variable_initialize.c`** — 變數初始化
4. **`system_initialize.C`** — 系統週邊初始化（PWM、ADC、GPIO 等）
5. **`main.c`** — 主程式迴圈與狀態機
6. **`INT_ISR.C`** — 中斷服務程式（PWM/ADC 中斷）
7. **`motor_six_step.c`** — 六步換相控制
8. **`motor_FOC.c`** — FOC（磁場導向控制）演算法
9. **`PI_control.c`** — PI 控制器實作
10. **`protocol.c`** — 通訊協定處理

### 4. 參數調整
在 `system_parameter.H` 中可調整以下關鍵參數：
- PWM 載波頻率
- 電流/電壓限制值
- PI 控制器增益（Kp、Ki）
- 馬達極對數

### 5. 硬體連接
- 準備 NM1240 開發板與馬達驅動板
- 連接 Nu-Link 除錯器
- 連接 BLDC 或 PMSM 馬達
- 確認電源供應正確

## ⚠️ 注意事項

- 修改馬達參數前，請確認馬達規格書
- 首次運行建議先以**低轉速**測試
- 本專案使用 **Q15/Q26 定點數格式**，修改數學運算時需特別注意精度
