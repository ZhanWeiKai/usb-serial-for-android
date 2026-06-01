# 固定 30 次测量取众数输出结果

## 一、需求

取消"连续 N 次稳定才输出"的逻辑。
改为固定测量 30 帧，对 width/length/height 分别取整数众数（截断小数部分），作为最终结果。

## 二、算法

```
固定采集 30 帧，每帧记录 width、length、height
对每个维度：
  1. 截断小数部分转整数（(int) value）
  2. 过滤掉 0 值（未检测到物体的帧）
  3. 统计每个整数出现的次数
  4. 取出现次数最多的整数作为众数
  5. 如果多个整数出现次数相同（平局），取其中最小的
30 帧采集完毕 → 输出众数结果
```

### 示例

```
width:  17.8, 19.8, 17.6, 20.1, 18.2, 17.3, 19.8, 17.1, 18.5, 17.9
取整:  17,    19,    17,    20,    18,    17,    19,    17,    18,    17
统计: 17→5次, 18→2次, 19→2次, 20→1次
众数: 17
```

## 三、代码改动

### 改动文件

**仅一个文件**：`ObjectMeasurementActivity.java`

### 改动 1：新增常量（第 60 行后）

```java
private static final int STABLE_COUNT_MEASURE_AFTER_LOCK = 5; // 阈值锁定后: 连续5次稳定
private static final int FIXED_MEASURE_COUNT = 30; // 固定测量帧数
```

### 改动 2：新增众数计算方法

```java
/**
 * 计算整数众数
 *
 * 对 List<Float> 中的值：
 * 1. 截断小数转整数
 * 2. 过滤 0 值
 * 3. 统计出现次数
 * 4. 返回出现次数最多的整数（平局取最小）
 */
private int mode(List<Float> values) {
    Map<Integer, Integer> freq = new HashMap<>();
    for (float v : values) {
        int iv = (int) v; // 截断小数
        if (iv == 0) continue;
        freq.put(iv, freq.getOrDefault(iv, 0) + 1);
    }
    int bestValue = 0;
    int bestCount = 0;
    for (Map.Entry<Integer, Integer> entry : freq.entrySet()) {
        if (entry.getValue() > bestCount ||
            (entry.getValue() == bestCount && entry.getKey() < bestValue)) {
            bestCount = entry.getValue();
            bestValue = entry.getKey();
        }
    }
    return bestValue;
}
```

### 改动 3：修改测量逻辑（替换 handleMeasurementResult 中第 1114-1171 行）

当前逻辑：比较帧间差异判断稳定/不稳定，连续 N 次稳定取平均值。

改为：

```java
float diffW = Math.abs(result.width - lastWidth);
float diffL = Math.abs(result.length - lastLength);
float diffH = Math.abs(result.height - lastHeight);
float maxDimDiff = Math.max(diffW, Math.max(diffL, diffH));

// 记录上一帧数据（用于显示差异）
lastWidth = result.width;
lastLength = result.length;
lastHeight = result.height;
lastResult = result;

// 累积帧数据
measureCount++;
if (result.width > 0) measureWidths.add(result.width);
if (result.length > 0) measureLengths.add(result.length);
if (result.height > 0) measureHeights.add(result.height);

runOnUiThread(() -> {
    updateMeasurementUI(result, diffW, diffL, diffH, measureCount, maskStatsFinal);
});

// 固定 30 帧后输出众数结果
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
```

### 改动 4：UI 显示（第 1179 行 updateMeasurementUI）

```java
// 当前代码：
sb.append(String.format("稳定进度: %d/%d (阈值: %.0fmm)\n", count, requiredCount, DIMENSION_THRESHOLD_MM));

// 改为：
sb.append(String.format("测量进度: %d/%d\n", count, FIXED_MEASURE_COUNT));
```

### 改动 5：新增成员变量（替换 stableCount 等）

```java
// 新增
private int measureCount = 0;                  // 测量帧计数
private List<Float> measureWidths = new ArrayList<>();  // 累积宽度
private List<Float> measureLengths = new ArrayList<>(); // 累积长度
private List<Float> measureHeights = new ArrayList<>(); // 累积高度

// 可以删除以下不再使用的变量（如果只用于旧的稳定逻辑）：
// stableWidths, stableLengths, stableHeights, stableCount
```

注意：`stableWidths/stableLengths/stableHeights/stableCount` 可能在其他地方还有引用（如校准流程），需要保留用于校准，测量部分改用新的 `measureWidths/measureLengths/measureHeights/measureCount`。

### 改动 6：开始测量时重置计数（第 1025 行附近）

```java
measureCount = 0;
measureWidths.clear();
measureLengths.clear();
measureHeights.clear();
```

## 四、流程示意

```
开始测量
  ↓
每帧记录 width/length/height
  ↓
进度: 1/30 → 2/30 → ... → 30/30
  ↓
取各维度整数众数（截断小数）
  ↓
输出结果
```

## 五、保留不变的部分

- 动态阈值机制（收紧/锁定）→ 仍然生效，用于过滤噪声
- 校准流程 → 不变，仍然用连续稳定逻辑
- `finishMeasurement()` → 不变
