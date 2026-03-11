package com.hoho.android.usbserial.examples;

/**
 * 物体三维尺寸计算器
 *
 * 基于深度数据计算放置在桌面上的物体的长、宽、高、厚度
 *
 * 使用方法:
 * 1. 先采集空桌面的深度数据作为 baselineDepth
 * 2. 放置物体后采集 currentDepth
 * 3. 调用 calculate() 方法计算物体尺寸
 */
public class ObjectDimensionCalculator {

    private static final String TAG = "DimensionCalc";
    private static final double COS_45 = Math.cos(Math.toRadians(45)); // ≈ 0.707

    // 输入数据
    private float[][] baselineDepth;  // 空桌面深度 (cm)
    private float[][] currentDepth;   // 放置物体后的深度 (cm)
    private float pixelSizeX;         // 每个像素 X 方向实际宽度 (cm)
    private float pixelSizeY;         // 每个像素 Y 方向实际长度 (cm)
    private float threshold;          // 物体检测阈值 (cm)

    // 计算结果
    private int[][] mask;             // 物体掩码: 1=物体, 0=背景
    private float width;              // 物体宽度 W (cm)
    private float length;             // 物体长度 L (cm)
    private float height;             // 物体高度 H (cm)
    private float thickness;          // 物体厚度 T (cm)

    // 调试信息
    private int xMin, xMax, yMin, yMax;  // mask 范围
    private int objectPixelCount;        // 物体像素数

    /**
     * 构造函数
     *
     * @param baselineDepth 空桌面深度数组 [width][height], 单位: cm
     * @param currentDepth  放置物体后的深度数组 [width][height], 单位: cm
     * @param pixelSizeX    每个像素 X 方向对应的实际宽度 (cm)
     * @param pixelSizeY    每个像素 Y 方向对应的实际长度 (cm)
     * @param threshold     物体检测阈值 (cm), 建议 1~3 cm
     */
    public ObjectDimensionCalculator(float[][] baselineDepth, float[][] currentDepth,
                                     float pixelSizeX, float pixelSizeY, float threshold) {
        this.baselineDepth = baselineDepth;
        this.currentDepth = currentDepth;
        this.pixelSizeX = pixelSizeX;
        this.pixelSizeY = pixelSizeY;
        this.threshold = threshold;
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
            return new DimensionResult(0, 0, 0, 0, "未检测到物体");
        }

        // Step 2: 计算物体高度 H
        calculateHeight();

        // Step 3 & 4: 计算物体宽度 W 和长度 L
        calculateWidthAndLength();

        // Step 5: 计算物体厚度 T
        calculateThickness();

        return new DimensionResult(width, length, height, thickness, "计算成功");
    }

    /**
     * Step 1: 生成物体 mask
     *
     * 对每个像素 (x, y):
     * - 如果 baselineDepth[x][y] - currentDepth[x][y] > threshold → mask[x][y] = 1 (物体)
     * - 否则 mask[x][y] = 0 (背景)
     */
    private void generateMask() {
        int width = baselineDepth.length;
        int height = baselineDepth[0].length;

        objectPixelCount = 0;
        xMin = width;
        xMax = -1;
        yMin = height;
        yMax = -1;

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                float depthDiff = baselineDepth[x][y] - currentDepth[x][y];

                if (depthDiff > threshold) {
                    mask[x][y] = 1;
                    objectPixelCount++;

                    // 更新边界
                    if (x < xMin) xMin = x;
                    if (x > xMax) xMax = x;
                    if (y < yMin) yMin = y;
                    if (y > yMax) yMax = y;
                } else {
                    mask[x][y] = 0;
                }
            }
        }
    }

    /**
     * Step 2: 计算物体高度 H
     *
     * 物体高度沿光轴:
     * H_raw = max(baselineDepth[x][y] - currentDepth[x][y] for mask[x][y] == 1)
     * H = H_raw * cos(45°)
     */
    private void calculateHeight() {
        float maxDepthDiff = 0;

        for (int x = xMin; x <= xMax; x++) {
            for (int y = yMin; y <= yMax; y++) {
                if (mask[x][y] == 1) {
                    float diff = baselineDepth[x][y] - currentDepth[x][y];
                    if (diff > maxDepthDiff) {
                        maxDepthDiff = diff;
                    }
                }
            }
        }

        // H = H_raw * cos(45°)
        height = maxDepthDiff * (float) COS_45;
    }

    /**
     * Step 3 & 4: 计算物体宽度 W 和长度 L
     *
     * W = (x_max - x_min + 1) * pixelSizeX
     * L = (y_max - y_min + 1) * pixelSizeY
     */
    private void calculateWidthAndLength() {
        width = (xMax - xMin + 1) * pixelSizeX;
        length = (yMax - yMin + 1) * pixelSizeY;
    }

    /**
     * Step 5: 计算物体厚度 T
     *
     * 找 mask 中深度最大值和最小值:
     * d_max = max(currentDepth[x][y] for mask[x][y]==1)
     * d_min = min(currentDepth[x][y] for mask[x][y]==1)
     * T_raw = d_max - d_min
     * T = T_raw * cos(45°)
     */
    private void calculateThickness() {
        float dMax = Float.MIN_VALUE;
        float dMin = Float.MAX_VALUE;

        for (int x = xMin; x <= xMax; x++) {
            for (int y = yMin; y <= yMax; y++) {
                if (mask[x][y] == 1) {
                    float depth = currentDepth[x][y];
                    if (depth > dMax) dMax = depth;
                    if (depth < dMin) dMin = depth;
                }
            }
        }

        // T = T_raw * cos(45°)
        thickness = (dMax - dMin) * (float) COS_45;
    }

    /**
     * 获取调试信息字符串
     */
    public String getDebugInfo() {
        StringBuilder sb = new StringBuilder();
        sb.append("═══ 物体尺寸计算调试信息 ═══\n");
        sb.append(String.format("Mask 范围: x[%d-%d], y[%d-%d]\n", xMin, xMax, yMin, yMax));
        sb.append(String.format("物体像素数: %d\n", objectPixelCount));
        sb.append(String.format("像素尺寸: X=%.3f cm, Y=%.3f cm\n", pixelSizeX, pixelSizeY));
        sb.append(String.format("检测阈值: %.2f cm\n", threshold));
        sb.append("───────────────────────────\n");
        sb.append(String.format("宽度 W: %.2f cm\n", width));
        sb.append(String.format("长度 L: %.2f cm\n", length));
        sb.append(String.format("高度 H: %.2f cm (含cos45°校正)\n", height));
        sb.append(String.format("厚度 T: %.2f cm (含cos45°校正)\n", thickness));
        sb.append("═══════════════════════════");
        return sb.toString();
    }

    // ==================== Getters ====================

    public int[][] getMask() { return mask; }
    public float getWidth() { return width; }
    public float getLength() { return length; }
    public float getHeight() { return height; }
    public float getThickness() { return thickness; }
    public int getObjectPixelCount() { return objectPixelCount; }
    public int getXMin() { return xMin; }
    public int getXMax() { return xMax; }
    public int getYMin() { return yMin; }
    public int getYMax() { return yMax; }

    /**
     * 计算结果封装类
     */
    public static class DimensionResult {
        public final float width;      // 宽度 (cm)
        public final float length;     // 长度 (cm)
        public final float height;     // 高度 (cm)
        public final float thickness;  // 厚度 (cm)
        public final String message;   // 状态消息

        public DimensionResult(float width, float length, float height, float thickness, String message) {
            this.width = width;
            this.length = length;
            this.height = height;
            this.thickness = thickness;
            this.message = message;
        }

        @Override
        public String toString() {
            return String.format("W=%.2fcm, L=%.2fcm, H=%.2fcm, T=%.2fcm | %s",
                    width, length, height, thickness, message);
        }
    }
}
