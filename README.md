# Jerk Control — S 曲線加減速控制庫

一個輕量級的 C/C++ 函式庫，用於生成 **S 曲線（Jerk-limited）** 速度規劃，實現平滑的梯形加減速運動控制。透過限制加加速度（Jerk），消除傳統梯形速度規劃中加速度突變所造成的機械衝擊與振動。

## 特點

- 🚀 **S 曲線速度規劃** — 以 Jerk 限制為基礎，自動生成七段式加減速曲線
- 🔧 **自動參數最佳化** — 根據目標距離自動調整速度與加速度，確保運動可行性
- 📐 **即時速度計算** — 給定時間步長即可逐步取得當前速度，適用於即時控制迴路
- 🧩 **C/C++ 相容** — 標頭檔同時支援 C 與 C++ 編譯器

## 原理

S 曲線加減速將運動過程分為 **七個階段**：

| 階段 | 說明 | 加速度變化 |
|:----:|------|:----------:|
| 1 | 加加速段（Jerk > 0） | 線性增加 |
| 2 | 等加速段 | 恆定 |
| 3 | 減加速段（Jerk < 0） | 線性減少至 0 |
| 4 | 等速段 | 0 |
| 5 | 加減速段（Jerk < 0） | 線性減少 |
| 6 | 等減速段 | 恆定（負值） |
| 7 | 減減速段（Jerk > 0） | 線性增加至 0 |

當目標距離較短時，函式庫會自動縮減最大速度或加速度，使曲線在有限距離內仍然可行。

## 專案結構

```
Jerk_control/
├── src/
│   ├── Jerk_control.h      # 標頭檔（結構體定義與函式宣告）
│   └── Jerk_control.cpp    # 核心演算法實作
├── test/
│   └── test.c              # 測試程式（含 SVG 圖形輸出）
└── README.md
```

## API 說明

### 資料結構

```c
// 加速度—時間節點
typedef struct jerkType {
    double timing;       // 時間點
    double Accelerate;   // 該時間點的加速度值
} jerkType;

// 速度、加速度、加加速度設定
typedef struct VAJ_set {
    double velocity;     // 最大速度
    double accelerate;   // 最大加速度
    double jerk;         // 最大加加速度（Jerk）
} VAJ_set;
```

### 函式

#### `VAJ_process`

```c
void VAJ_process(
    double distance,        // 目標移動距離
    VAJ_set VAJ,            // 速度/加速度/Jerk 設定
    jerkType *Jerk_array    // 輸出：8 個時間—加速度節點陣列
);
```

根據目標距離與 VAJ 設定，計算並填入 S 曲線的 8 個關鍵時間—加速度節點。函式內部會自動呼叫最佳化邏輯，在距離不足時降低最大速度或加速度。

#### `velocity_process`

```c
double velocity_process(
    double dt,                      // 時間步長
    int i,                          // 目前步數（時間 = i × dt）
    const jerkType Jerk_array[8],   // VAJ_process 產生的節點陣列
    bool *finished,                 // 輸出：是否已完成整段運動
    bool *t4_ed                     // 輸出：是否已進入減速階段
);
```

給定時間步長與步數，回傳當前的速度值。可在控制迴路中逐步呼叫，直到 `*finished` 為 `true`。

## 使用範例

```c
#include "Jerk_control.h"
#include <stdio.h>

int main() {
    const double dt = 0.005;
    const VAJ_set settings = { 70.0, 100.0, 140.0 };  // 速度, 加速度, Jerk
    jerkType array[8];
    bool finished = false, t4_ed = false;

    // 規劃 200 單位距離的 S 曲線
    VAJ_process(200.0, settings, array);

    // 逐步取得速度
    double total_distance = 0.0;
    for (int i = 0; !finished; i++) {
        double speed = velocity_process(dt, i, array, &finished, &t4_ed);
        total_distance += speed * dt;
        printf("t=%.3f  v=%.4f\n", i * dt, speed);
    }

    printf("總距離: %.4f\n", total_distance);
    return 0;
}
```

## 編譯與測試

### 編譯測試程式

```bash
# 使用 g++ 編譯
g++ -o test test/test.c src/Jerk_control.cpp -lm

# 執行測試
./test
```

測試程式會對多組距離進行驗證，並輸出 `vaj_graph.svg` 速度曲線圖形。

### 測試距離

測試涵蓋以下距離，驗證短距離自動降速與長距離完整 S 曲線：

`20.0` · `50.0` · `80.0` · `100.0` · `150.0` · `200.0` · `500.0` · `1000.0` · `2000.0`

## 授權條款

本專案為開源專案，詳見倉庫中的授權檔案。
