# Y方向标准差平滑度滤波 - 实现文档

## 一、问题

深度相机横放后，图像 Y 方向远端（y≈72+）产生大量噪声像素。
这些噪声像素通过了 depthDiff > 50mm 的阈值检测，被错误标记为物体，导致边界扩大、测量不准。

### Log 数据对比

**物体行 (y=40)** — diff 集中在 114~134mm，波动 20mm：
```
116, 122, 122, 122, 128, 128, 128, 122, 118, 122, 122, 122, 128, 134...
标准差 ≈ 6mm
```

**噪声行 (y=75)** — diff 散布，波动大：
```
标准差 ≈ 20-30mm
```

**本质区别**：物体表面平坦，同一行的 depthDiff 值集中；噪声随机散布，同一行的 depthDiff 值分散。

### 为什么不做 X 方向

每列（x 固定）跨越整个 Y 范围，由于 45° 倾斜，不同 y 位置的 depthDiff 有自然渐变。
一列的标准差天然 20-28mm，全部超过阈值，会把整个物体误杀。
所以**只做 Y 方向（逐行）**。

## 二、方案：Y 方向标准差滤波

### 算法

对 mask 中每一行 y（不做 X 方向）：
1. 收集该行所有 mask=1 像素的 depthDiff 值
2. 像素数 < 3 → 跳过（样本太少，标准差无意义）
3. 计算这些值的**标准差**
4. 标准差 > 15mm → 判定为噪声行，将该行所有 mask=1 像素置为 mask=0

### 阈值选择

| 指标 | 物体行 | 噪声行 | 阈值 |
|------|--------|--------|------|
| 标准差 | ≈ 6mm | ≈ 20-30mm | **15mm** |

## 三、代码改动

### 改动文件

**仅一个文件**：`ObjectDimensionCalculator.java`

### 改动 1：新增常量（第 21 行后）

```java
private static final int MIN_CONSECUTIVE_PIXELS = 8;  // 连续像素法
private static final float SMOOTHNESS_STDDEV_THRESHOLD = 15.0f; // Y方向标准差阈值 (mm)
private static final int SMOOTHNESS_MIN_PIXELS = 3;             // 最少像素数
```

### 改动 2：新增方法（放在 `findEdgeByDirectConsecutivePixels()` 之后）

```java
/**
 * Y方向标准差平滑度滤波
 *
 * 逐行计算 mask=1 像素 depthDiff 的标准差。
 * 标准差 > 阈值的行判定为噪声，清除该行 mask。
 * 注意：不做 X 方向，因为 45° 倾斜导致列内 depthDiff 自然渐变。
 */
private void smoothnessFilterByStdDev() {
    int cols = baselineDepth.length;
    int rows = baselineDepth[0].length;

    for (int y = 0; y < rows; y++) {
        float[] diffs = new float[cols];
        int count = 0;
        for (int x = 0; x < cols; x++) {
            if (mask[x][y] == 1) {
                diffs[count++] = baselineDepth[x][y] - currentDepth[x][y];
            }
        }
        if (count < SMOOTHNESS_MIN_PIXELS) continue;

        float mean = 0;
        for (int i = 0; i < count; i++) mean += diffs[i];
        mean /= count;

        float variance = 0;
        for (int i = 0; i < count; i++) {
            float d = diffs[i] - mean;
            variance += d * d;
        }
        float stdDev = (float) Math.sqrt(variance / count);

        if (stdDev > SMOOTHNESS_STDDEV_THRESHOLD) {
            for (int x = 0; x < cols; x++) {
                if (mask[x][y] == 1) {
                    mask[x][y] = 0;
                    objectPixelCount--;
                }
            }
            Log.d(TAG, String.format("平滑度滤波: 噪声行 y=%d, stdDev=%.1fmm, 清除%d像素",
                    y, stdDev, count));
        }
    }
}
```

### 改动 3：在 3 个计算方法中调用（generateMask() 之后）

#### 位置 A：`calculate()` 方法（第 104 行后）

```java
// Step 1: 生成物体 mask
generateMask();

// Step 1.5: Y方向标准差平滑度滤波
smoothnessFilterByStdDev();

if (objectPixelCount == 0) {
```

#### 位置 B：`calculateDimensionsByCalibratedRatio()` 方法（第 138 行后）

```java
// Step 1: 生成物体 mask
generateMask();

// Step 1.5: Y方向标准差平滑度滤波
smoothnessFilterByStdDev();

if (objectPixelCount == 0) {
```

#### 位置 C：`calculateDimensionsByCalibratedRatioWithMedianData()` 方法（第 335 行后）

```java
// Step 1: 生成物体 mask
generateMask();

// Step 1.5: Y方向标准差平滑度滤波
smoothnessFilterByStdDev();

if (objectPixelCount == 0) {
```

## 四、处理流程（改动后）

```
generateMask()              → 初始 mask（含噪声像素）
smoothnessFilterByStdDev()   → Y方向逐行标准差滤波，清除噪声行
findEdgeByDirectConsecutivePixels() → 基于清洁的 mask 检测边界
calculateHeightByDepthDiff() → 计算高度
计算长宽                      → 基于清洁的边界
```

## 五、不改的文件

- `ObjectMeasurer.java` — 不改，它只调用 calculator 的公共方法
- `A010RawDataActivity.java` — 不改
