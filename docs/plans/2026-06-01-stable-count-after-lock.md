# 动态阈值锁定后测量稳定次数降为 5 帧

## 一、需求

当前：测量物体需要连续 10 帧稳定才输出结果。
改动：
1. 动态阈值锁定后，测量稳定次数从 10 次降为 5 次，加快测量速度。
2. 阈值收紧到上限 100mm 时，也视为锁定。

## 二、逻辑

```
阈值未锁定且 < 100mm → 测量需要 10 帧稳定
阈值锁定 或 达到 100mm → 测量只需 5 帧稳定（加速）
```

## 三、代码改动

### 改动文件

**仅一个文件**：`ObjectMeasurementActivity.java`

### 改动 1：新增常量（第 60 行后）

```java
private static final int STABLE_COUNT_MEASURE = 10;   // 测量: 连续10次稳定 = 5秒
private static final int STABLE_COUNT_MEASURE_AFTER_LOCK = 5; // 阈值锁定后: 连续5次稳定
```

### 改动 2：阈值到上限时也视为锁定（第 1289-1300 行）

```java
// 当前代码：
if (unstableThresholdCount >= 3 && !thresholdLocked) {
    currentMaskThreshold += 5.0f;
    if (currentMaskThreshold > MAX_THRESHOLD_MM) {
        currentMaskThreshold = MAX_THRESHOLD_MM;
    }
    unstableThresholdCount = 0;
    stableThresholdCount = 0;
    statusChange = String.format("阈值收紧→%.0fmm", currentMaskThreshold);
}

// 改为：
if (unstableThresholdCount >= 3 && !thresholdLocked) {
    currentMaskThreshold += 5.0f;
    if (currentMaskThreshold > MAX_THRESHOLD_MM) {
        currentMaskThreshold = MAX_THRESHOLD_MM;
    }
    // 阈值到达上限，视为锁定
    if (currentMaskThreshold >= MAX_THRESHOLD_MM) {
        thresholdLocked = true;
        statusChange = String.format("阈值已达上限%.0fmm(锁定)", currentMaskThreshold);
        Log.d(TAG, "动态阈值已达上限，锁定在: " + currentMaskThreshold + "mm");
    } else {
        // 阈值调整后，重置所有计数，重新观察
        unstableThresholdCount = 0;
        stableThresholdCount = 0;
        statusChange = String.format("阈值收紧→%.0fmm", currentMaskThreshold);
    }
    Log.d(TAG, "动态阈值收紧至: " + currentMaskThreshold + "mm (像素变化=" + pixelChange + ")");
}
```

### 改动 3：判断稳定完成时使用动态次数（第 1135 行）

```java
// 当前代码：
if (stableCount >= STABLE_COUNT_MEASURE) {

// 改为：
int requiredCount = thresholdLocked ? STABLE_COUNT_MEASURE_AFTER_LOCK : STABLE_COUNT_MEASURE;
if (stableCount >= requiredCount) {
```

### 改动 4：UI 显示稳定进度时使用动态次数（第 1179 行）

```java
// 当前代码：
sb.append(String.format("稳定进度: %d/%d (阈值: %.0fmm)\n", count, STABLE_COUNT_MEASURE, DIMENSION_THRESHOLD_MM));

// 改为：
int requiredCount = thresholdLocked ? STABLE_COUNT_MEASURE_AFTER_LOCK : STABLE_COUNT_MEASURE;
sb.append(String.format("稳定进度: %d/%d (阈值: %.0fmm)\n", count, requiredCount, DIMENSION_THRESHOLD_MM));
```

### 改动 5：开始测量时的 UI 提示（第 1048 行）

```java
// 当前代码：
infoText.setText(String.format("正在测量物体尺寸...\n\n请先点 [清除Mask] 消除噪声，再放置物体\n\n稳定进度: 0/%d", STABLE_COUNT_MEASURE));

// 改为：
infoText.setText(String.format("正在测量物体尺寸...\n\n请先点 [清除Mask] 消除噪声，再放置物体\n\n稳定进度: 0/%d(锁定后%d)", STABLE_COUNT_MEASURE, STABLE_COUNT_MEASURE_AFTER_LOCK));
```

### 改动 6：完成测量时的提示（第 1148 行）

```java
// 当前代码：
"测量完成 (" + STABLE_COUNT_MEASURE + "帧平均)"

// 改为：
"测量完成 (" + requiredCount + "帧平均)"
```
注意：这里 `requiredCount` 需要在改动 3 处保存为局部变量。

## 四、流程示意

```
开始测量
  ↓
阈值未锁定：稳定进度显示 x/10
  ↓ (阈值锁定，或达到100mm上限)
阈值已锁定：稳定进度显示 x/5
  ↓ (5帧稳定)
输出结果
```
