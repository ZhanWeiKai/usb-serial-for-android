package com.hoho.android.usbserial.examples;

import android.util.Log;

/**
 * 物体三维尺寸计算器
 *
 * 基于 3D 坐标变换计算放置在桌面上的物体的长、宽、高
 *
 * 原理：
 * 1. 深度图 → 3D 点云（相机坐标系）
 * 2. 45° 旋转变换 → 世界坐标系（垂直向下）
 * 3. 计算物体点云的 AABB（轴对齐包围盒）
 * 4. 从 AABB 获取真实长宽高
 */
public class ObjectDimensionCalculator {

    private static final String TAG = "DimensionCalc";
    private static final int MIN_PROJECTION_COUNT = 20; // 行/列至少需要的 mask=1 像素数

    // 45° 旋转相关常量
    private static final double TILT_ANGLE_DEG = 45.0;
    private static final double TILT_ANGLE_RAD = Math.toRadians(TILT_ANGLE_DEG);
    private static final double COS_TILT = Math.cos(TILT_ANGLE_RAD);  // cos(45°) ≈ 0.707
    private static final double SIN_TILT = Math.sin(TILT_ANGLE_RAD);  // sin(45°) ≈ 0.707

    // 输入数据
    private float[][] baselineDepth;  // 空桌面深度 (mm)
    private float[][] currentDepth;   // 放置物体后的深度 (mm)
    private float fovHorizontal;      // 水平视场角 (度)
    private float fovVertical;        // 垂直视场角 (度)
    private float threshold;          // 物体检测阈值 (mm)
    private boolean[][] noiseMask;    // 噪声像素黑名单，true=排除

    // 计算结果
    private int[][] mask;             // 物体掩码: 1=物体, 0=背景
    private float width;              // 物体宽度 W (mm) - X 方向
    private float length;             // 物体长度 L (mm) - Y 方向
    private float height;             // 物体高度 H (mm) - Z 方向

    // 行列投影（用于噪声过滤）
    private int[] colCount;
    private int[] rowCount;

    // 调试信息
    private int xMin, xMax, yMin, yMax;
    private int objectPixelCount;
    private int validPixelCount;
    private int maxDiffX = -1, maxDiffY = -1;
    private float maxDiffBaselineDepth = 0;
    private float maxDiffCurrentDepth = 0;

    // 3D 点云存储（世界坐标系）
    private float[] worldX;  // 物体点云的 X 坐标
    private float[] worldY;  // 物体点云的 Y 坐标
    private float[] worldZ;  // 物体点云的 Z 坐标
    private int pointCloudCount;

    /**
     * 构造函数
     *
     * @param baselineDepth 空桌面深度数组 [width][height], 单位: mm
     * @param currentDepth  放置物体后的深度数组 [width][height], 单位: mm
     * @param fovHorizontal 水平视场角 (度)，例如 70.0
     * @param fovVertical   垂直视场角 (度)，例如 60.0
     * @param threshold     物体检测阈值 (mm), 建议 50 mm
     */
    public ObjectDimensionCalculator(float[][] baselineDepth, float[][] currentDepth,
                                     float fovHorizontal, float fovVertical, float threshold) {
        this(baselineDepth, currentDepth, fovHorizontal, fovVertical, threshold, null);
    }

    public ObjectDimensionCalculator(float[][] baselineDepth, float[][] currentDepth,
                                     float fovHorizontal, float fovVertical, float threshold,
                                     boolean[][] noiseMask) {
        this.baselineDepth = baselineDepth;
        this.currentDepth = currentDepth;
        this.fovHorizontal = fovHorizontal;
        this.fovVertical = fovVertical;
        this.threshold = threshold;
        this.noiseMask = noiseMask;
        this.mask = new int[baselineDepth.length][baselineDepth[0].length];

        // 预分配点云存储空间（最大 100x100 = 10000 点）
        int maxPoints = baselineDepth.length * baselineDepth[0].length;
        this.worldX = new float[maxPoints];
        this.worldY = new float[maxPoints];
        this.worldZ = new float[maxPoints];
    }

    /**
     * 执行完整的尺寸计算
     */
    public DimensionResult calculate() {
        // Step 1: 生成物体 mask
        generateMask();

        if (objectPixelCount == 0) {
            return new DimensionResult(0, 0, 0, 0, 0, "未检测到物体", -1, -1, -1, -1);
        }

        // Step 2: 行列投影过滤噪声
        calculateProjection();

        if (xMin == -1 || yMin == -1) {
            return new DimensionResult(0, 0, 0, objectPixelCount, 0,
                    "像素过于分散(mask=" + objectPixelCount + ")，未构成有效物体",
                    xMin, xMax, yMin, yMax);
        }

        // Step 3: 3D 坐标变换 + AABB 计算长宽高
        calculateDimensionsBy3DTransform();

        return new DimensionResult(width, length, height, objectPixelCount, validPixelCount, "计算成功(3D变换)",
                xMin, xMax, yMin, yMax);
    }

    /**
     * Step 1: 生成物体 mask
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

                // 基本有效性检查
                if (bDepth <= 0 || cDepth <= 0) {
                    mask[x][y] = 0;
                    continue;
                }

                // baseline 值必须在合理范围内
                if (bDepth < 100 || bDepth > 800) {
                    mask[x][y] = 0;
                    continue;
                }

                // 深度差必须在合理范围内
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
     * Step 2: 行列投影过滤噪声
     */
    private void calculateProjection() {
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

        int[] colRange = findLongestRun(colCount, cols);
        xMin = colRange[0];
        xMax = colRange[1];

        int[] rowRange = findLongestRun(rowCount, rows);
        yMin = rowRange[0];
        yMax = rowRange[1];
    }

    /**
     * Step 3: 3D 坐标变换 + AABB 计算长宽高
     *
     * 核心算法：
     * 1. 将深度图转换为 3D 点云（相机坐标系）
     * 2. 应用 45° 旋转变换（绕 X 轴）到世界坐标系
     * 3. 计算 AABB 的 min/max 范围
     */
    private void calculateDimensionsBy3DTransform() {
        int cols = baselineDepth.length;
        int rows = baselineDepth[0].length;

        // 相机内参（基于 FOV 和图像尺寸）
        double tanHalfFovH = Math.tan(Math.toRadians(fovHorizontal / 2.0));
        double tanHalfFovV = Math.tan(Math.toRadians(fovVertical / 2.0));

        // 焦距（以像素为单位）
        double fx = (cols / 2.0) / tanHalfFovH;
        double fy = (rows / 2.0) / tanHalfFovV;

        // 主点（图像中心）
        double cx = cols / 2.0;
        double cy = rows / 2.0;

        // 收集物体点云（世界坐标系）
        pointCloudCount = 0;
        validPixelCount = 0;
        float maxDepthDiff = 0;

        // AABB 边界
        float minX = Float.MAX_VALUE, maxX = Float.MIN_VALUE;
        float minY = Float.MAX_VALUE, maxY = Float.MIN_VALUE;
        float minZ = Float.MAX_VALUE, maxZ = Float.MIN_VALUE;

        for (int x = xMin; x <= xMax; x++) {
            if (colCount[x] < MIN_PROJECTION_COUNT) continue;
            for (int y = yMin; y <= yMax; y++) {
                if (rowCount[y] < MIN_PROJECTION_COUNT) continue;
                if (mask[x][y] != 1) continue;

                validPixelCount++;

                // 深度值（使用 current 深度，即到物体表面的距离）
                float depth = currentDepth[x][y];
                if (depth <= 0) continue;

                // ========== 相机坐标系 ==========
                // X_cam = (u - cx) * Z / fx
                // Y_cam = (v - cy) * Z / fy
                // Z_cam = depth
                double xCam = (x - cx) * depth / fx;
                double yCam = (y - cy) * depth / fy;
                double zCam = depth;

                // ========== 45° 旋转变换（绕 X 轴）==========
                // 旋转矩阵 Rx:
                // | 1    0        0      |
                // | 0  cos(θ)  -sin(θ)   |
                // | 0  sin(θ)   cos(θ)   |
                //
                // 世界坐标系：
                // X_world = X_cam
                // Y_world = Y_cam * cos(45°) - Z_cam * sin(45°)
                // Z_world = Y_cam * sin(45°) + Z_cam * cos(45°)
                double xWorld = xCam;
                double yWorld = yCam * COS_TILT - zCam * SIN_TILT;
                double zWorld = yCam * SIN_TILT + zCam * COS_TILT;

                // 保存点云
                worldX[pointCloudCount] = (float) xWorld;
                worldY[pointCloudCount] = (float) yWorld;
                worldZ[pointCloudCount] = (float) zWorld;
                pointCloudCount++;

                // 更新 AABB
                if (xWorld < minX) minX = (float) xWorld;
                if (xWorld > maxX) maxX = (float) xWorld;
                if (yWorld < minY) minY = (float) yWorld;
                if (yWorld > maxY) maxY = (float) yWorld;
                if (zWorld < minZ) minZ = (float) zWorld;
                if (zWorld > maxZ) maxZ = (float) zWorld;

                // 记录最大深度差点
                float diff = baselineDepth[x][y] - currentDepth[x][y];
                if (diff > maxDepthDiff) {
                    maxDepthDiff = diff;
                    maxDiffX = x;
                    maxDiffY = y;
                    maxDiffBaselineDepth = baselineDepth[x][y];
                    maxDiffCurrentDepth = currentDepth[x][y];
                }
            }
        }

        // ========== 从 AABB 计算尺寸 ==========

        // 检查点云是否为空
        if (pointCloudCount == 0) {
            width = 0;
            length = 0;
            height = 0;
            return;
        }

        // 宽度 = X 方向范围
        width = maxX - minX;

        // 长度 = Y 方向范围
        length = maxY - minY;

        // 高度 = 最大深度差 × cos(45°)
        // 这是最可靠的方法，因为深度差直接测量了物体表面到桌面的距离
        // 乘以 cos(45°) 投影到垂直方向
        height = maxDepthDiff * (float) COS_TILT;

        // 确保非负
        if (width < 0) width = 0;
        if (length < 0) length = 0;
        if (height < 0) height = 0;

        // 输出计算结果日志
        Log.d("ObjectMeasurement", String.format(
                "═══ 3D计算结果 ═══\n" +
                "点云数量: %d, 有效像素: %d\n" +
                "AABB: X[%.1f~%.1f] Y[%.1f~%.1f] Z[%.1f~%.1f]\n" +
                "结果: W=%.1fmm, L=%.1fmm, H=%.1fmm\n" +
                "maxDepthDiff=%.1fmm",
                pointCloudCount, validPixelCount,
                minX, maxX, minY, maxY, minZ, maxZ,
                width, length, height, maxDepthDiff
        ));
    }

    /**
     * 在投影数组中找最长连续有效段
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

    // ==================== 统计信息 ====================

    public String getMaskDepthDiffStats() {
        if (objectPixelCount == 0) return "无 mask=1 像素";

        int cols = baselineDepth.length;
        int rows = baselineDepth[0].length;
        float minDiff = Float.MAX_VALUE, maxDiff = 0, sumDiff = 0;

        for (int x = 0; x < cols; x++) {
            for (int y = 0; y < rows; y++) {
                if (mask[x][y] == 1) {
                    float diff = baselineDepth[x][y] - currentDepth[x][y];
                    if (diff < minDiff) minDiff = diff;
                    if (diff > maxDiff) maxDiff = diff;
                    sumDiff += diff;
                }
            }
        }

        float avgDiff = sumDiff / objectPixelCount;

        return String.format("Mask=1: %d个, 深度差: min=%.0f max=%.0f avg=%.0f mm",
                objectPixelCount, minDiff, maxDiff, avgDiff);
    }

    public String getDebugInfo() {
        StringBuilder sb = new StringBuilder();
        sb.append("═══ 3D 坐标变换法 ═══\n");
        sb.append(String.format("原始Mask: %d → 有效: %d\n", objectPixelCount, validPixelCount));
        sb.append(String.format("FOV: H=%.0f° V=%.0f°, 倾斜: %.0f°\n", fovHorizontal, fovVertical, TILT_ANGLE_DEG));
        sb.append(String.format("点云数量: %d\n", pointCloudCount));
        sb.append("───── AABB 尺寸 ─────\n");
        sb.append(String.format("宽度 W(X): %.1f mm\n", width));
        sb.append(String.format("长度 L(Y): %.1f mm\n", length));
        sb.append(String.format("高度 H(Z): %.1f mm\n", height));
        sb.append("══════════════════════");
        return sb.toString();
    }

    // ==================== Getters ====================

    public int[][] getMask() { return mask; }
    public int[] getColCount() { return colCount; }
    public int[] getRowCount() { return rowCount; }
    public int getMinProjectionCount() { return MIN_PROJECTION_COUNT; }
    public float getWidth() { return width; }
    public float getLength() { return length; }
    public float getHeight() { return height; }
    public int getObjectPixelCount() { return objectPixelCount; }
    public int getValidPixelCount() { return validPixelCount; }
    public int getXMin() { return xMin; }
    public int getXMax() { return xMax; }
    public int getYMin() { return yMin; }
    public int getYMax() { return yMax; }
    public int getMaxDiffX() { return maxDiffX; }
    public int getMaxDiffY() { return maxDiffY; }
    public float getMaxDiffBaselineDepth() { return maxDiffBaselineDepth; }
    public float getMaxDiffCurrentDepth() { return maxDiffCurrentDepth; }

    /**
     * 获取中心深度（有效像素的平均深度或边界中心点深度）
     */
    public float getCenterDepth() {
        if (xMin < 0 || yMin < 0) return 0;

        // 方法1: 计算边界中心点的深度
        int centerX = (xMin + xMax) / 2;
        int centerY = (yMin + yMax) / 2;

        // 方法2: 计算有效像素的平均深度
        float sumDepth = 0;
        int count = 0;
        for (int x = xMin; x <= xMax; x++) {
            if (colCount[x] < MIN_PROJECTION_COUNT) continue;
            for (int y = yMin; y <= yMax; y++) {
                if (rowCount[y] < MIN_PROJECTION_COUNT) continue;
                if (mask[x][y] != 1) continue;
                float depth = currentDepth[x][y];
                if (depth > 0) {
                    sumDepth += depth;
                    count++;
                }
            }
        }

        if (count > 0) {
            return sumDepth / count;
        }

        // 如果没有有效像素，返回边界中心点深度
        return currentDepth[centerX][centerY];
    }

    public boolean isValidPixel(int x, int y) {
        if (mask[x][y] != 1) return false;
        if (colCount == null || rowCount == null) return false;
        return colCount[x] >= MIN_PROJECTION_COUNT && rowCount[y] >= MIN_PROJECTION_COUNT;
    }

    public float getDepthDiffAt(int x, int y) {
        if (x < 0 || x >= baselineDepth.length || y < 0 || y >= baselineDepth[0].length) {
            return 0;
        }
        return baselineDepth[x][y] - currentDepth[x][y];
    }

    /**
     * 计算结果封装类
     */
    public static class DimensionResult {
        public final float width;
        public final float length;
        public final float height;
        public final int rawPixelCount;
        public final int validPixelCount;
        public final String message;
        // 有效像素边界范围
        public final int xMin, xMax, yMin, yMax;

        public DimensionResult(float width, float length, float height,
                               int rawPixelCount, int validPixelCount, String message,
                               int xMin, int xMax, int yMin, int yMax) {
            this.width = width;
            this.length = length;
            this.height = height;
            this.rawPixelCount = rawPixelCount;
            this.validPixelCount = validPixelCount;
            this.message = message;
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        @Override
        public String toString() {
            return String.format("W=%.1fmm, L=%.1fmm, H=%.1fmm | %s", width, length, height, message);
        }
    }
}
