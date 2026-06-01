# 基于物体高度自动确定 mask 阈值

## 一、问题

当前动态阈值机制依赖像素范围稳定性来判断阈值：
- 像素变化 < 3 → 稳定计数++
- 像素变化 ≥ 3 → 不稳定计数++，3次不稳定就阈值 +5mm
- 阈值一路涨到 100mm 锁定

问题：噪声大时像素范围持续跳动，阈值一直涨，最终超过物体高度，导致所有物体像素都被过滤掉，检测不到物体。

## 二、新方案

### 核心思路

**物体高度本身就能确定合理的 mask 阈值。**

- `height` = `maxDepthDiff × cos(45°)`，其中 `maxDepthDiff` 是 mask 中所有像素的最大 `baselineDepth - currentDepth`
- maxDepthDiff 代表物体最靠近相机的点的深度差
- 阈值设为 maxDepthDiff 的 90%，就能保留物体核心像素、过滤噪声

### 算法

```
1. 用初始阈值 50mm 计算 mask → 得到初步物体像素
2. 从 mask 像素中计算 maxDepthDiff → height = maxDepthDiff × cos(45°)
3. 锁定阈值 = maxDepthDiff × 0.9（不用 cos，因为 mask 用的是原始深度差）
4. 后续帧直接用锁定阈值生成 mask
```

注意：`generateMask()` 中判断的是 `depthDiff > threshold`（原始深度差，不是高度），所以锁定阈值应该基于 `maxDepthDiff` 而不是 `height`。

### 示例

```
物体 maxDepthDiff = 80mm
height = 80 × cos(45°) = 56.6mm
锁定阈值 = 80 × 0.9 = 72mm

mask 条件：depthDiff > 72mm
→ 噪声像素（depthDiff 20~40mm）被过滤
→ 物体核心像素（depthDiff 72~80mm）保留
→ 物体边缘像素（depthDiff 50~72mm）也会被过滤（可接受）
```

## 三、代码改动

### 改动文件

**仅一个文件**：`ObjectMeasurementActivity.java`

### 改动 1：新增常量（第 96 行后）

```java
private static final float MAX_THRESHOLD_MM = 100.0f;     // 阈值上限 (mm)
private static final int HEIGHT_LOCK_FRAMES = 10;          // 用于确定高度的帧数
private static final float HEIGHT_LOCK_RATIO = 0.9f;       // 阈值 = maxDepthDiff × 此比例
```

### 改动 2：新增成员变量（第 104 行后）

```java
// 基于高度的阈值锁定
private float heightBasedThreshold = 0;      // 基于高度确定的阈值
private boolean heightThresholdLocked = false; // 是否已通过高度锁定阈值
private List<Float> earlyMaxDepthDiffs = new ArrayList<>(); // 前几帧的 maxDepthDiff
```

### 改动 3：需要从 Calculator 获取 maxDepthDiff

在 `ObjectDimensionCalculator.java` 中新增 getter 方法：

```java
// 新增公共方法（放在 getter 区域）
public float getMaxDepthDiff() {
    return maxDepthDiffValue;
}
```

### 改动 4：修改 `processMeasurementSample()`（第 1070~1163 行）

**当前逻辑**：每帧都调用 `updateDynamicThreshold()` 基于像素稳定性调整阈值。

**改为**：

```java
private void processMeasurementSample() {
    if (latestMedianDepth == null || baselineDepth == null) {
        return;
    }

    currentDepth = copyDepthArray(latestMedianDepth);

    // 使用当前阈值创建计算器
    ObjectDimensionCalculator calculator = new ObjectDimensionCalculator(
            baselineDepth, currentDepth,
            (float)FOV_HORIZONTAL_DEG, (float)FOV_VERTICAL_DEG, currentMaskThreshold, noiseMask
    );

    ObjectDimensionCalculator.DimensionResult result = calculator
            .calculateDimensionsByCalibratedRatioWithMedianData(pixelSizeX, pixelSizeY);

    String maskStats = calculator.getMaskDepthDiffStats();

    int pxMin = calculator.getXMin();
    int pxMax = calculator.getXMax();
    int pyMin = calculator.getYMin();
    int pyMax = calculator.getYMax();

    int xSpan = (pxMax >= pxMin) ? (pxMax - pxMin + 1) : 0;
    int ySpan = (pyMax >= pyMin) ? (pyMax - pyMin + 1) : 0;

    // ====== 新的阈值逻辑：基于高度自动锁定 ======
    if (!heightThresholdLocked && result.height > 0) {
        // 获取原始 maxDepthDiff（未乘 cos45°）
        float maxDepthDiff = calculator.getMaxDepthDiff();
        if (maxDepthDiff > 0) {
            earlyMaxDepthDiffs.add(maxDepthDiff);

            // 收集够 HEIGHT_LOCK_FRAMES 帧后，取中位数确定阈值
            if (earlyMaxDepthDiffs.size() >= HEIGHT_LOCK_FRAMES) {
                float medianDiff = median(earlyMaxDepthDiffs);
                heightBasedThreshold = medianDiff * HEIGHT_LOCK_RATIO;
                // 确保不低于初始阈值
                if (heightBasedThreshold < OBJECT_THRESHOLD_MM) {
                    heightBasedThreshold = OBJECT_THRESHOLD_MM;
                }
                currentMaskThreshold = heightBasedThreshold;
                heightThresholdLocked = true;
                Log.d(TAG, String.format("基于高度锁定阈值: maxDepthDiff=%.1fmm, 阈值=%.1fmm (×%.0f)",
                        medianDiff, heightBasedThreshold, HEIGHT_LOCK_RATIO * 100));
            }
        }
    }

    String thresholdStatus;
    if (heightThresholdLocked) {
        thresholdStatus = String.format("阈值:%.0fmm(高度锁定)", currentMaskThreshold);
    } else {
        thresholdStatus = String.format("阈值:%.0fmm 采样:%d/%d", currentMaskThreshold,
                earlyMaxDepthDiffs.size(), HEIGHT_LOCK_FRAMES);
    }

    maskStats += String.format(
            "\n边界像素范围: x[%d-%d] y[%d-%d] (%d×%d)\n",
            pxMin, pxMax, pyMin, pyMax, xSpan, ySpan);
    maskStats += "\n───── 动态阈值 ─────\n" + thresholdStatus + "\n";
    final String maskStatsFinal = maskStats;

    updateDiffHeatmap(calculator);

    if (result == null || "未检测到物体".equals(result.message) || result.validPixelCount == 0) {
        runOnUiThread(() -> {
            StringBuilder sb = new StringBuilder();
            sb.append("未检测到物体\n\n请将物体放置在桌面上\n");
            sb.append("(物体需至少占8个连续像素)\n\n");
            sb.append("───── Mask 诊断 ─────\n");
            sb.append(maskStatsFinal);
            infoText.setText(sb.toString());
        });
        return;
    }

    float diffW = Math.abs(result.width - lastWidth);
    float diffL = Math.abs(result.length - lastLength);
    float diffH = Math.abs(result.height - lastHeight);

    lastWidth = result.width;
    lastLength = result.length;
    lastHeight = result.height;
    lastResult = result;

    measureCount++;
    if (result.width > 0) measureWidths.add(result.width);
    if (result.length > 0) measureLengths.add(result.length);
    if (result.height > 0) measureHeights.add(result.height);

    runOnUiThread(() -> {
        updateMeasurementUI(result, diffW, diffL, diffH, measureCount, maskStatsFinal);
    });

    if (measureCount >= FIXED_MEASURE_COUNT) {
        int finalWidth = mode(measureWidths);
        int finalLength = mode(measureLengths);
        int finalHeight = mode(measureHeights);
        float volumeCm3 = (float)(finalWidth * finalLength * finalHeight) / 1000.0f;

        ObjectDimensionCalculator.DimensionResult finalResult =
            new ObjectDimensionCalculator.DimensionResult(
                finalWidth, finalLength, finalHeight,
                result.rawPixelCount, result.validPixelCount,
                "测量完成 (" + FIXED_MEASURE_COUNT + "帧众数)",
                result.xMin, result.xMax, result.yMin, result.yMax
            );
        finishMeasurement(finalResult, volumeCm3);
    }
}
```

### 改动 5：新增 median() 方法

```java
/**
 * 计算中位数
 */
private float median(List<Float> values) {
    List<Float> sorted = new ArrayList<>(values);
    Collections.sort(sorted);
    int size = sorted.size();
    if (size % 2 == 1) {
        return sorted.get(size / 2);
    } else {
        return (sorted.get(size / 2 - 1) + sorted.get(size / 2)) / 2.0f;
    }
}
```

需要新增 import：`import java.util.Collections;`

### 改动 6：修改 `resetDynamicThreshold()`（第 1348 行）

```java
private void resetDynamicThreshold() {
    currentMaskThreshold = OBJECT_THRESHOLD_MM;
    unstableThresholdCount = 0;
    stableThresholdCount = 0;
    thresholdLocked = false;
    lastXSpan = 0;
    lastYSpan = 0;
    // 新增：重置高度锁定
    heightBasedThreshold = 0;
    heightThresholdLocked = false;
    earlyMaxDepthDiffs.clear();
}
```

### 改动 7：删除旧的 `updateDynamicThreshold()` 方法（第 1267~1319 行）

整个方法不再需要，删除。同时可以删除不再使用的变量：
- `unstableThresholdCount`
- `stableThresholdCount`
- `thresholdLocked`
- `lastXSpan`
- `lastYSpan`

注意：这些变量在校准流程中可能也有使用，需要确认。如果校准流程中也用了 `updateDynamicThreshold()`，校准部分保持旧的稳定逻辑不变。

## 四、ObjectDimensionCalculator.java 改动

### 改动：新增 getMaxDepthDiff() getter

在公共 getter 区域（如 `getXMin()`、`getYMin()` 附近）新增：

```java
public float getMaxDepthDiff() {
    return maxDepthDiffValue;
}
```

## 五、流程示意

```
开始测量
  ↓
前10帧用 50mm 阈值，收集每帧的 maxDepthDiff
  ↓ (10帧后)
取 maxDepthDiff 中位数 × 0.9 = 锁定阈值（如 72mm）
  ↓
后续帧用 72mm 阈值生成 mask
  ↓
30帧测量完成，取众数输出
```

## 六、保留不变的部分

- 固定 30 帧众数测量 → 不变
- Y方向标准差滤波 → 不变
- 校准流程 → 不变（校准仍用旧的稳定逻辑）
- finishMeasurement() → 不变
