package com.hoho.android.usbserial.examples;

/**
 * 物体三维尺寸计算器
 *
 * 基于深度数据计算放置在桌面上的物体的长、宽、高
 *
 * 使用方法:
 * 1. 先采集空桌面的深度数据作为 baselineDepth
 * 2. 放置物体后采集 currentDepth
 * 3. 调用 calculate() 方法计算物体尺寸
 */
public class ObjectDimensionCalculator {

    private static final String TAG = "DimensionCalc";
    private static final double COS_45 = Math.cos(Math.toRadians(45)); // ≈ 0.707
    private static final int MIN_PROJECTION_COUNT = 8; // 行/列至少需要的 mask=1 像素数

    // 输入数据
    private float[][] baselineDepth;  // 空桌面深度 (mm)
    private float[][] currentDepth;   // 放置物体后的深度 (mm)
    private float pixelSizeX;         // 每个像素 X 方向实际宽度 (mm)
    private float pixelSizeY;         // 每个像素 Y 方向实际长度 (mm)
    private float threshold;          // 物体检测阈值 (mm)
    private boolean[][] noiseMask;    // 噪声像素黑名单，true=排除

    // 计算结果
    private int[][] mask;             // 物体掩码: 1=物体, 0=背景
    private float width;              // 物体宽度 W (mm)
    private float length;             // 物体长度 L (mm)
    private float height;             // 物体高度 H (mm)

    // 行列投影
    private int[] colCount;              // 每列 mask=1 像素数
    private int[] rowCount;              // 每行 mask=1 像素数

    // 调试信息
    private int xMin, xMax, yMin, yMax;  // 投影后的有效范围
    private int objectPixelCount;        // mask=1 原始像素数
    private int validPixelCount;         // 投影过滤后的有效像素数

    /**
     * 构造函数
     *
     * @param baselineDepth 空桌面深度数组 [width][height], 单位: mm
     * @param currentDepth  放置物体后的深度数组 [width][height], 单位: mm
     * @param pixelSizeX    每个像素 X 方向对应的实际宽度 (mm)
     * @param pixelSizeY    每个像素 Y 方向对应的实际长度 (mm)
     * @param threshold     物体检测阈值 (mm), 建议 10~30 mm
     */
    public ObjectDimensionCalculator(float[][] baselineDepth, float[][] currentDepth,
                                     float pixelSizeX, float pixelSizeY, float threshold) {
        this(baselineDepth, currentDepth, pixelSizeX, pixelSizeY, threshold, null);
    }

    public ObjectDimensionCalculator(float[][] baselineDepth, float[][] currentDepth,
                                     float pixelSizeX, float pixelSizeY, float threshold,
                                     boolean[][] noiseMask) {
        this.baselineDepth = baselineDepth;
        this.currentDepth = currentDepth;
        this.pixelSizeX = pixelSizeX;
        this.pixelSizeY = pixelSizeY;
        this.threshold = threshold;
        this.noiseMask = noiseMask;
        this.mask = new int[baselineDepth.length][baselineDepth[0].length];
    }

    /**
     * 执行完整的尺寸计算
     *
     * @return DimensionResult 包含长宽高厚的结果对象
     */
    public DimensionResult calculate() {
        // Step 1: 生成物体 mask
        generateMask();

        // 检查是否检测到物体
        if (objectPixelCount == 0) {
            return new DimensionResult(0, 0, 0, 0, 0, "未检测到物体");
        }

        // Step 2: 行列投影法计算宽度、长度、高度
        calculateDimensionsByProjection();

        if (xMin == -1 || yMin == -1) {
            return new DimensionResult(0, 0, 0, objectPixelCount, 0,
                    "像素过于分散(mask=" + objectPixelCount + ")，未构成有效物体");
        }

        return new DimensionResult(width, length, height, objectPixelCount, validPixelCount, "计算成功");
    }

    /**
     * Step 1: 生成物体 mask
     *
     * 对每个像素 (x, y):
     * - 如果 baselineDepth[x][y] - currentDepth[x][y] > threshold → mask[x][y] = 1 (物体)
     * - 否则 mask[x][y] = 0 (背景)
     */
    private void generateMask() {
        int cols = baselineDepth.length;
        int rows = baselineDepth[0].length;

        objectPixelCount = 0;

        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                if (noiseMask != null && noiseMask[x][y]) {
                    mask[x][y] = 0;
                    continue;
                }

                float bDepth = baselineDepth[x][y];
                float cDepth = currentDepth[x][y];

                // 1. 基本有效性检查
                if (bDepth <= 0 || cDepth <= 0) {
                    mask[x][y] = 0;
                    continue;
                }

                // 2. baseline 值必须在合理范围内（比如 100-1000mm）
                // 超出范围的像素可能是噪声
                if (bDepth < 100 || bDepth > 1000) {
                    mask[x][y] = 0;
                    continue;
                }

                // 3. 深度差必须在合理范围内
                float depthDiff = bDepth - cDepth;
                if (depthDiff < 0 || depthDiff > 800) {
                    mask[x][y] = 0;
                    continue;
                }

                if (depthDiff > threshold) {
                    mask[x][y] = 1;
                    objectPixelCount++;
                } else {
                    mask[x][y] = 0;
                }
            }
        }
    }

    /**
     * Step 2: 行列投影法计算宽度、长度、高度
     *
     * 统计每列(x)和每行(y)的 mask=1 像素数，
     * 只有 >= MIN_PROJECTION_COUNT 的行/列才被视为有效物体范围，
     * 天然过滤掉零散噪声像素。
     */
    private void calculateDimensionsByProjection() {
        int cols = baselineDepth.length;
        int rows = baselineDepth[0].length;

        colCount = new int[cols];
        rowCount = new int[rows];

        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                if (mask[x][y] == 1) {
                    colCount[x]++;
                    rowCount[y]++;
                }
            }
        }

        // 找最长连续有效列段 → 物体宽度范围
        int[] colRange = findLongestRun(colCount, cols);
        xMin = colRange[0];
        xMax = colRange[1];

        // 找最长连续有效行段 → 物体长度范围
        int[] rowRange = findLongestRun(rowCount, rows);
        yMin = rowRange[0];
        yMax = rowRange[1];

        if (xMin == -1 || yMin == -1) {
            validPixelCount = 0;
            width = 0;
            length = 0;
            height = 0;
            return;
        }

        width = (xMax - xMin + 1) * pixelSizeX;
        length = (yMax - yMin + 1) * pixelSizeY;

        // 统计有效像素 & 高度: 最长连续段矩形内取 max 深度差
        validPixelCount = 0;
        float maxDepthDiff = 0;
        for (int x = xMin; x <= xMax; x++) {
            if (colCount[x] < MIN_PROJECTION_COUNT) continue;
            for (int y = yMin; y <= yMax; y++) {
                if (rowCount[y] < MIN_PROJECTION_COUNT) continue;
                if (mask[x][y] == 1) {
                    validPixelCount++;
                    float diff = baselineDepth[x][y] - currentDepth[x][y];
                    if (diff > maxDepthDiff) maxDepthDiff = diff;
                }
            }
        }
        height = maxDepthDiff * (float) COS_45;
    }

    /**
     * 在投影数组中找最长连续有效段 (count >= MIN_PROJECTION_COUNT)
     * @return [start, end]，未找到返回 [-1, -1]
     */
    private int[] findLongestRun(int[] counts, int length) {
        int bestStart = -1, bestEnd = -1, bestLen = 0;
        int curStart = -1, curLen = 0;

        for (int i = 0; i < length; i++) {
            if (counts[i] >= MIN_PROJECTION_COUNT) {
                if (curStart == -1) curStart = i;
                curLen++;
                if (curLen > bestLen) {
                    bestLen = curLen;
                    bestStart = curStart;
                    bestEnd = i;
                }
            } else {
                curStart = -1;
                curLen = 0;
            }
        }
        return new int[]{bestStart, bestEnd};
    }

    /**
     * 获取 mask=1 像素的深度差分布统计
     */
    public String getMaskDepthDiffStats() {
        if (objectPixelCount == 0) return "无 mask=1 像素";

        int cols = baselineDepth.length;
        int rows = baselineDepth[0].length;
        float minDiff = Float.MAX_VALUE, maxDiff = 0, sumDiff = 0;
        int[] histogram = new int[10];

        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                if (mask[x][y] == 1) {
                    float diff = baselineDepth[x][y] - currentDepth[x][y];
                    if (diff < minDiff) minDiff = diff;
                    if (diff > maxDiff) maxDiff = diff;
                    sumDiff += diff;

                    int bin = (int) ((diff - threshold) / 10);
                    if (bin < 0) bin = 0;
                    if (bin >= histogram.length) bin = histogram.length - 1;
                    histogram[bin]++;
                }
            }
        }

        float avgDiff = sumDiff / objectPixelCount;

        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Mask=1 像素: %d 个\n", objectPixelCount));
        sb.append(String.format("深度差: min=%.0f max=%.0f avg=%.0f mm\n", minDiff, maxDiff, avgDiff));
        sb.append("分布:\n");
        for (int i = 0; i < histogram.length; i++) {
            if (histogram[i] > 0) {
                int lo = (int) threshold + i * 10;
                int hi = lo + 10;
                if (i == histogram.length - 1) {
                    sb.append(String.format("  %d+ mm: %d 个\n", lo, histogram[i]));
                } else {
                    sb.append(String.format("  %d-%d mm: %d 个\n", lo, hi, histogram[i]));
                }
            }
        }
        return sb.toString();
    }

    /**
     * 获取调试信息字符串
     */
    public String getDebugInfo() {
        StringBuilder sb = new StringBuilder();
        sb.append("═══ 物体尺寸计算调试信息 ═══\n");
        sb.append(String.format("原始Mask像素: %d → 投影有效: %d\n", objectPixelCount, validPixelCount));
        sb.append(String.format("像素尺寸: X=%.1f mm, Y=%.1f mm\n", pixelSizeX, pixelSizeY));
        sb.append(String.format("检测阈值: %.0f mm\n", threshold));
        sb.append(String.format("投影阈值: %d 像素/行列\n", MIN_PROJECTION_COUNT));
        sb.append("───── 投影结果 ─────\n");
        if (xMin == -1 || yMin == -1) {
            sb.append("投影: 未找到有效物体区域\n");
        } else {
            sb.append(String.format("有效列范围: x[%d-%d] (%d列)\n", xMin, xMax, xMax - xMin + 1));
            sb.append(String.format("有效行范围: y[%d-%d] (%d行)\n", yMin, yMax, yMax - yMin + 1));
        }
        sb.append("───────────────────\n");
        sb.append(String.format("宽度 W: %.1f mm\n", width));
        sb.append(String.format("长度 L: %.1f mm\n", length));
        sb.append(String.format("高度 H: %.1f mm (cos45°校正)\n", height));
        sb.append("═══════════════════════════");
        return sb.toString();
    }

    // ==================== Getters ====================

    public int[][] getMask() { return mask; }
    public float getWidth() { return width; }
    public float getLength() { return length; }
    public float getHeight() { return height; }
    public int getObjectPixelCount() { return objectPixelCount; }
    public int getValidPixelCount() { return validPixelCount; }
    public int getXMin() { return xMin; }
    public int getXMax() { return xMax; }
    public int getYMin() { return yMin; }
    public int getYMax() { return yMax; }

    /**
     * 计算结果封装类
     */
    public static class DimensionResult {
        public final float width;           // 宽度 (mm)
        public final float length;          // 长度 (mm)
        public final float height;          // 高度 (mm)
        public final int rawPixelCount;     // mask=1 原始像素数
        public final int validPixelCount;   // 投影过滤后的有效像素数
        public final String message;        // 状态消息

        public DimensionResult(float width, float length, float height,
                               int rawPixelCount, int validPixelCount, String message) {
            this.width = width;
            this.length = length;
            this.height = height;
            this.rawPixelCount = rawPixelCount;
            this.validPixelCount = validPixelCount;
            this.message = message;
        }

        @Override
        public String toString() {
            return String.format("W=%.2fmm, L=%.2fmm, H=%.2fmm, pixels=%d/%d | %s",
                    width, length, height, validPixelCount, rawPixelCount, message);
        }
    }
}
