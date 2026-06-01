# 深度相机物体尺寸计算原理与横放适配方案

## 一、计算长宽高的每一步步骤和原理

### 1. 原始数据采集

**帧格式**：通过 USB 串口接收 A010 TOF 传感器数据

```
[0x00, 0xFF] [包长度2B] [元数据16B] [像素数据10000B] [校验1B] [0xDD]
```

- 分辨率：100x100 = 10000 像素
- 每像素 1 字节 (0-255)
- 帧率：15fps

### 2. 像素值转物理距离

传感器原始像素值通过平方公式转换为毫米距离：

```
distance_mm = (pixelValue / 5.1)²
```

| 像素值 | 距离(mm) | 距离(cm) |
|--------|----------|----------|
| 50     | 96       | 9.6      |
| 100    | 384      | 38.4     |
| 150    | 865      | 86.5     |
| 200    | 1538     | 153.8    |

像素值 0 = 无效数据，距离设为 0。

### 3. 基线校准

**目的**：获取空桌面的深度数据作为参考基准。

**流程**：
1. 桌面上不放物体，采集多帧（默认 10 帧一个窗口）
2. 对每个像素位置，取多帧的中值作为基线值（去除瞬时噪声）
3. 连续多个窗口的基线差异 < 14mm 才认为校准稳定
4. 计算像素尺寸：
   ```
   centerDepth = baselineDepth[50][50]   // 中心点深度
   pixelSizeX = 2 × centerDepth × tan(70°/2) / 100   // X方向 mm/像素
   pixelSizeY = 2 × centerDepth × tan(60°/2) / 100   // Y方向 mm/像素
   ```

### 4. 物体分割（生成 Mask）

对比空桌面基线和放置物体后的深度图，逐像素判断：

```
depthDiff = baselineDepth[x][y] - currentDepth[x][y]
```

**标记为物体像素 (mask=1) 的条件**：
- `depthDiff > 50mm`（阈值，可动态调整）
- baseline 值在 100~800mm 范围内
- depthDiff 在 0~800mm 范围内
- 不在噪声掩码中

### 5. 边界检测（连续像素法）

在 mask 上扫描，过滤噪声：

- 扫描每列 x：如果该列有**连续 >= 8 个** mask=1 像素，该列有效
- 扫描每行 y：如果该行有**连续 >= 8 个** mask=1 像素，该行有效
- 得到边界：`xMin, xMax, yMin, yMax`

### 6. 高度计算（最先算）

```
maxDepthDiff = max(baselineDepth[x][y] - currentDepth[x][y])   // 物体最突出的点
height = maxDepthDiff × cos(45°)                                   // ≈ maxDepthDiff × 0.707
```

**为什么要乘 cos(45°)**：传感器倾斜 45° 安装，测到的深度差是沿光束方向的，
投影到垂直方向才是真实高度。

```
        传感器
         \  ← 45° 倾斜
          \  maxDepthDiff（沿光束方向）
           \
            | height（垂直方向）
            | = maxDepthDiff × cos(45°)
            |
        ══════════ 桌面
```

### 7. 物体表面深度

```
objectSurfaceDepth = baselineDepth_at_maxDiff - maxDepthDiff
```

传感器到物体顶面的真实距离（比桌面更近），后续计算像素尺寸用。

### 8. 动态像素尺寸

```
pixelSizeX = 2.0 × objectSurfaceDepth × tan(70°/2) / 100
pixelSizeY = 2.0 × objectSurfaceDepth × tan(60°/2) / 100
```

**推导**：在深度 d 处，70° 视场角能看到的总宽度 = `2 × d × tan(35°)`，
除以 100 个像素 = 每像素多少 mm。物体越近，每像素越小。

### 9. 长宽计算

```
width  = (xMax - xMin + 1) × pixelSizeX
length = (yMax - yMin + 1) × pixelSizeY
```

像素跨度 × 每像素物理尺寸 = 真实尺寸。

### 10. 多帧降噪

- 每个采样窗口采集 10 帧
- 测量模式：选 maxDepthDiff 最接近中位数的帧（避免异常帧）
- 连续测量稳定后（尺寸波动 < 20mm，连续 10 次稳定），取平均值

---

## 二、横放适配方案

### 问题描述

深度相机由**竖着 45° 倾斜**改为**横着 45° 倾斜**俯拍桌面物体，图像旋转了 90°。

### 物理映射变化

**竖着放时（当前代码正确的映射）**：

```
        [传感器正面]
             \
              \  X轴(图像) → 物理前后方向 = 物体的宽
               \
                \ Y轴(图像) → 物理左右方向 = 物体的长
                 \
             ══════\═══════ 桌面
```

| 图像轴 | FOV | 物理方向 | 对应物体 |
|--------|-----|----------|----------|
| X | 70° (fovHorizontal) | 前后 | 宽(width) |
| Y | 60° (fovVertical) | 左右 | 长(length) |

**横着放后（旋转90°）**：

```
          [传感器侧面]
              \
               \  X轴(图像) → 物理左右方向 = 物体的长 ⬅️ 变了！
                \
                 \ Y轴(图像) → 物理前后方向 = 物体的宽 ⬅️ 变了！
                  \
              ══════\═══════ 桌面
```

| 图像轴 | FOV | 物理方向 | 对应物体 |
|--------|-----|----------|----------|
| X | 70° (fovHorizontal) | 左右 | 长(length) |
| Y | 60° (fovVertical) | 前后 | 宽(width) |

### 关键结论：FOV 不需要换，宽长结果需要换

**FOV 是传感器硬件属性，随传感器一起旋转**：
- 传感器 X 轴始终是 70°，旋转后 X 轴变成了物理左右方向
- 传感器 Y 轴始终是 60°，旋转后 Y 轴变成了物理前后方向
- pixelSizeX 和 pixelSizeY 的计算不受影响

**数值验证（举例）**：

假设物体：长 200mm（左右）、宽 100mm（前后），传感器到物体表面 400mm

竖放时：
- pixelSizeX = 2×400×tan(35°)/100 = 5.60 mm/像素（X=前后）
- pixelSizeY = 2×400×tan(30°)/100 = 4.62 mm/像素（Y=左右）
- width = 18像素 × 5.60 = 100.8mm（前后=宽）✓
- length = 43像素 × 4.62 = 198.7mm（左右=长）✓

横放时（物体在图像中 X/Y 对调）：
- pixelSizeX = 5.60 mm/像素（X=左右了）
- pixelSizeY = 4.62 mm/像素（Y=前后了）
- width = 36像素 × 5.60 = 201.6mm → **实际是左右方向，应该是长**
- length = 22像素 × 4.62 = 101.6mm → **实际是前后方向，应该是宽**

**数值是对的，但宽和长的标签反了！**

如果错误地交换 FOV（改为 60°/70°）：
- pixelSizeX = 2×400×tan(30°)/100 = 4.62 mm/像素 ← 错误！应该是 5.60
- width = 36 × 4.62 = 166mm ← 错误！应该是 100mm
- length = 22 × 5.60 = 123mm ← 错误！应该是 200mm

**结论：交换 FOV 会导致计算结果错误。正确做法是交换宽长结果。**

### 需要修改的地方

#### 1. ObjectDimensionCalculator.java — calculateDimensionsByCalibratedRatioWithMedianData()

交换最终的 width 和 length 赋值：

```java
// 修改前（竖放）
width = xSpan * dynamicPixelSizeX;
length = ySpan * dynamicPixelSizeY;

// 修改后（横放）
width = ySpan * dynamicPixelSizeY;    // Y=前后 → 宽
length = xSpan * dynamicPixelSizeX;   // X=左右 → 长
```

#### 2. ObjectDimensionCalculator.java — calculateDimensionsByCalibratedRatio()

同样交换：

```java
// 修改前
width = xSpan * pixelSizeX;
length = ySpan * pixelSizeY;

// 修改后
width = ySpan * pixelSizeY;
length = xSpan * pixelSizeX;
```

#### 3. ObjectDimensionCalculator.java — calculateDimensionsBy3DTransform()

3D 变换方法也需要交换宽长：

```java
// 修改前
width = maxX - minX;   // X方向范围
length = maxY - minY;  // Y方向范围

// 修改后
width = maxY - minY;   // Y=前后 → 宽
length = maxX - minX;  // X=左右 → 长
```

#### 4. 横放远端噪声问题

横放后 Y 方向的远端（高 y 值）离传感器更远，TOF 精度下降导致噪声增大。
可通过 noiseMask 或收紧 baseline 有效范围来解决（后续优化）。

---

## 三、公式汇总

| 步骤 | 公式 | 说明 |
|------|------|------|
| 像素→距离 | `d = (p / 5.1)²` mm | UNIT=0 模式 |
| 物体检测 | `baseline - current > 50mm` | 阈值分割 |
| 边界检测 | 连续 >= 8 个 mask=1 像素 | 过滤噪声 |
| 高度 | `maxDiff × cos(45°)` | 光束方向投影到垂直方向 |
| 物体表面深度 | `baseline_at_maxDiff - maxDiff` | 传感器到物体顶面 |
| 像素尺寸X | `2 × depth × tan(35°) / 100` mm/像素 | 70° FOV（硬件常量） |
| 像素尺寸Y | `2 × depth × tan(30°) / 100` mm/像素 | 60° FOV（硬件常量） |
| **竖放宽** | `xSpan × pixelSizeX` | X=前后 |
| **竖放长** | `ySpan × pixelSizeY` | Y=左右 |
| **横放宽** | `ySpan × pixelSizeY` | Y=前后 |
| **横放长** | `xSpan × pixelSizeX` | X=左右 |
| 体积 | `W × L × H / 1000` cm³ | 长方体包围盒 |
