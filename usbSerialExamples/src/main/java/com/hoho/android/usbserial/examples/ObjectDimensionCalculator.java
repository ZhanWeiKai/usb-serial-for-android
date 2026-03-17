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
    private static final int MIN_PROJECTION_COUNT_ABSOLUTE = 10; // 绝对最小阈值（防止噪声）
    private static final float PROJECTION_THRESHOLD_RATIO = 0.7f; // 动态阈值 = maxCount * ratio
    private static final int MIN_CONSECUTIVE_PIXELS = 8;  // 连续像素法: 连续>=8个有效像素才算边界
    private int dynamicThresholdX = MIN_PROJECTION_COUNT_ABSOLUTE; // X方向动态阈值
    private int dynamicThresholdY = MIN_PROJECTION_COUNT_ABSOLUTE; // Y方向动态阈值

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

    // 新增: 用于新计算顺序
    private float maxDepthDiffValue = 0;  // 最大深度差值
    private float baselineDepthAtMaxDiff = 0;  // 最大深度差位置的基线深度

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
     * 使用校准比例计算物体尺寸（推荐方法）
     *
     * 原理：校准时确定了像素-物理尺寸的比例，测量时直接用这个比例计算。
     * 这种方法比 3D 变换更准确，因为避免了深度测量的系统误差。
     *
     * @param pixelSizeX X方向每像素对应的物理尺寸 (mm/像素)
     * @param pixelSizeY Y方向每像素对应的物理尺寸 (mm/像素)
     * @return 尺寸计算结果
     */
    public DimensionResult calculateDimensionsByCalibratedRatio(float pixelSizeX, float pixelSizeY) {
        // Step 1: 生成物体 mask
        generateMask();

        if (objectPixelCount == 0) {
            return new DimensionResult(0, 0, 0, 0, 0, "未检测到物体", -1, -1, -1, -1);
        }

        // Step 2: 行列投影
        calculateProjection();

        // 检查投影结果是否有效
        if (xMin < 0 || yMin < 0) {
            return new DimensionResult(0, 0, 0, objectPixelCount, 0, "投影范围无效", xMin, xMax, yMin, yMax);
        }

        // Step 3: 计算有效像素数
        validPixelCount = 0;
        for (int x = xMin; x <= xMax; x++) {
            if (colCount[x] < dynamicThresholdX) continue;
            for (int y = yMin; y <= yMax; y++) {
                if (rowCount[y] < dynamicThresholdY) continue;
                if (mask[x][y] == 1) {
                    validPixelCount++;
                }
            }
        }

        if (validPixelCount == 0) {
            return new DimensionResult(0, 0, 0, objectPixelCount, 0, "有效像素为空", xMin, xMax, yMin, yMax);
        }

        // Step 4: 使用校准比例计算长宽
        int xSpan = xMax - xMin + 1;
        int ySpan = yMax - yMin + 1;

        width = xSpan * pixelSizeX;
        length = ySpan * pixelSizeY;

        // Step 5: 高度仍用深度差异法
        calculateHeightByDepthDiff();

        Log.d(TAG, String.format(
                "═══ 校准比例计算结果 ═══\n" +
                "像素跨度: X=%d, Y=%d\n" +
                "校准比例: pixelSizeX=%.2f, pixelSizeY=%.2f mm/像素\n" +
                "计算结果: W=%.1fmm, L=%.1fmm, H=%.1fmm",
                xSpan, ySpan, pixelSizeX, pixelSizeY, width, length, height
        ));

        return new DimensionResult(width, length, height, objectPixelCount, validPixelCount,
                "计算成功(校准比例)", xMin, xMax, yMin, yMax);
    }

    /**
     * 使用深度差异计算高度
     * 同时保存最大深度差值和对应位置的基线深度，供后续计算物体表面深度使用
     */
    private void calculateHeightByDepthDiff() {
        float maxDepthDiff = 0;
        float baselineAtMaxDiff = 0;

        for (int x = xMin; x <= xMax; x++) {
            if (colCount[x] < dynamicThresholdX) continue;
            for (int y = yMin; y <= yMax; y++) {
                if (rowCount[y] < dynamicThresholdY) continue;
                if (mask[x][y] != 1) continue;

                float diff = baselineDepth[x][y] - currentDepth[x][y];
                if (diff > maxDepthDiff) {
                    maxDepthDiff = diff;
                    baselineAtMaxDiff = baselineDepth[x][y];  // 保存该位置的基线深度
                }
            }
        }

        // 保存最大深度差和对应基线深度，供后续计算物体表面深度
        maxDepthDiffValue = maxDepthDiff;
        baselineDepthAtMaxDiff = baselineAtMaxDiff;

        // 高度 = 最大深度差 × cos(45°)
        height = maxDepthDiff * (float) COS_TILT;
        if (height < 0) height = 0;
    }

    /**
     * 仅计算行列投影（供多帧中值滤波使用）
     *
     * 在 generateMask() 后调用，只计算 colCount/rowCount，
     * 不做边界检测，用于获取原始投影数据
     */
    public void calculateProjectionOnly() {
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
    }

    /**
     * 生成mask（公开方法，供外部调用）
     */
    public void generateMask() {
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
     * 获取colCount数组（用于多帧中值滤波）
     */
    public int[] getColCount() {
        if (colCount == null) return null;
        return colCount.clone();
    }

    /**
     * 获取rowCount数组（用于多帧中值滤波）
     */
    public int[] getRowCount() {
        if (rowCount == null) return null;
        return rowCount.clone();
    }

    /**
     * 使用中值降噪后的深度图计算尺寸
     *
     * 按照文档要求的流程：
     * 1. 生成 mask
     * 2. 用直接连续像素法确定边界
     * 3. 先计算高度
     * 4. 计算物体表面深度
     * 5. 计算像素尺寸
     * 6. 计算长宽
     *
     * @param pixelSizeX X方向每像素对应的物理尺寸 (mm/像素) - 校准时计算的
     * @param pixelSizeY Y方向每像素对应的物理尺寸 (mm/像素) - 校准时计算的
     * @return 尺寸计算结果
     */
    public DimensionResult calculateDimensionsByCalibratedRatioWithMedianData(
            float pixelSizeX, float pixelSizeY) {

        // Step 1: 生成物体 mask
        generateMask();

        if (objectPixelCount == 0) {
            return new DimensionResult(0, 0, 0, 0, 0, "未检测到物体", -1, -1, -1, -1);
        }

        // Step 2: 使用直接连续像素法确定边界 (按照文档要求)
        int[] bounds = findEdgeByDirectConsecutivePixels();
        xMin = bounds[0];
        xMax = bounds[1];
        yMin = bounds[2];
        yMax = bounds[3];

        // 检查边界是否有效
        if (xMin < 0 || yMin < 0) {
            return new DimensionResult(0, 0, 0, objectPixelCount, 0, "边界检测无效(物体可能太小)", xMin, xMax, yMin, yMax);
        }

        // Step 3: 计算有效像素数（在边界范围内的 mask=1 像素）
        validPixelCount = 0;
        int cols = baselineDepth.length;
        int rows = baselineDepth[0].length;

        // 同时计算投影（用于统计）
        colCount = new int[cols];
        rowCount = new int[rows];

        for (int x = xMin; x <= xMax && x < cols; x++) {
            for (int y = yMin; y <= yMax && y < rows; y++) {
                if (mask[x][y] == 1) {
                    validPixelCount++;
                    colCount[x]++;
                    rowCount[y]++;
                }
            }
        }

        if (validPixelCount == 0) {
            return new DimensionResult(0, 0, 0, objectPixelCount, 0, "有效像素为空", xMin, xMax, yMin, yMax);
        }

        // Step 4: 先计算高度 (maxDepthDiff × cos(45°))
        calculateHeightByDepthDiff();

        // Step 5: 计算物体表面深度 = 基线深度 - 最大深度差
        // 物体表面深度 = 传感器到物体表面的距离 (比桌面更近)
        float objectSurfaceDepth = baselineDepthAtMaxDiff - maxDepthDiffValue;
        if (objectSurfaceDepth <= 0) {
            // 回退到中心深度
            objectSurfaceDepth = getCenterDepth();
            if (objectSurfaceDepth <= 0) {
                objectSurfaceDepth = currentDepth[(xMin + xMax) / 2][(yMin + yMax) / 2];
            }
        }

        // Step 6: 基于物体表面深度动态计算像素尺寸
        // 物体表面越近，每像素对应的物理尺寸越小
        int xSpan = xMax - xMin + 1;
        int ySpan = yMax - yMin + 1;

        double tanFovHHalf = Math.tan(Math.toRadians(fovHorizontal / 2.0));
        double tanFovVHalf = Math.tan(Math.toRadians(fovVertical / 2.0));
        float dynamicPixelSizeX = (float) (2.0 * objectSurfaceDepth * tanFovHHalf / 100.0);
        float dynamicPixelSizeY = (float) (2.0 * objectSurfaceDepth * tanFovVHalf / 100.0);

        // Step 7: 最后计算长宽
        width = xSpan * dynamicPixelSizeX;
        length = ySpan * dynamicPixelSizeY;

        Log.d(TAG, String.format(
                "═══ 直接连续像素法计算结果 ═══\n" +
                "像素跨度: X=%d, Y=%d\n" +
                "最大深度差: %.1fmm\n" +
                "物体表面深度: %.1fmm\n" +
                "动态像素尺寸: X=%.2f, Y=%.2f mm/像素\n" +
                "计算结果: W=%.1fmm, L=%.1fmm, H=%.1fmm",
                xSpan, ySpan, maxDepthDiffValue, objectSurfaceDepth,
                dynamicPixelSizeX, dynamicPixelSizeY, width, length, height
        ));

        return new DimensionResult(width, length, height, objectPixelCount, validPixelCount,
                "计算成功(直接连续像素法)", xMin, xMax, yMin, yMax);
    }

    /**
     * 兼容旧接口的方法（保留向后兼容）
     */
    public DimensionResult calculateDimensionsByCalibratedRatioWithMedianData(
            float pixelSizeX, float pixelSizeY,
            int[] medianColCount, int[] medianRowCount) {
        // 忽略 medianColCount 和 medianRowCount，直接使用新方法
        return calculateDimensionsByCalibratedRatioWithMedianData(pixelSizeX, pixelSizeY);
    }

    /**
     * 计算行列投影（用于噪声过滤）
     *
     * colCount[x] = 第x列有多少个mask=1的像素
     * rowCount[y] = 第y行有多少个mask=1的像素
     *
     * 边界检测使用直接连续像素法（按照文档要求）
     */
    private void calculateProjection() {
        int cols = baselineDepth.length;
        int rows = baselineDepth[0].length;

        // 使用直接连续像素法确定边界（按照文档要求）
        int[] bounds = findEdgeByDirectConsecutivePixels();
        xMin = bounds[0];
        xMax = bounds[1];
        yMin = bounds[2];
        yMax = bounds[3];

        // 计算投影（用于统计和有效像素计数）
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

        // 计算动态阈值（保留用于其他计算）
        int maxColCount = 0, maxRowCount = 0;
        for (int x = 0; x < cols; x++) {
            if (colCount[x] > maxColCount) maxColCount = colCount[x];
        }
        for (int y = 0; y < rows; y++) {
            if (rowCount[y] > maxRowCount) maxRowCount = rowCount[y];
        }

        dynamicThresholdX = Math.max(MIN_PROJECTION_COUNT_ABSOLUTE,
                (int)(maxColCount * PROJECTION_THRESHOLD_RATIO));
        dynamicThresholdY = Math.max(MIN_PROJECTION_COUNT_ABSOLUTE,
                (int)(maxRowCount * PROJECTION_THRESHOLD_RATIO));

        Log.d(TAG, String.format("直接连续像素法边界: x[%d-%d] y[%d-%d]",
                xMin, xMax, yMin, yMax));
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
            if (colCount[x] < dynamicThresholdX) continue;
            for (int y = yMin; y <= yMax; y++) {
                if (rowCount[y] < dynamicThresholdY) continue;
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
     * 在投影数组中找最长连续有效段（保留原方法作为备用）
     */
    private int[] findLongestRun(int[] counts, int length, int threshold) {
        int bestStart = -1, bestEnd = -1, bestLen = 0;
        int curStart = -1, curLen = 0;

        for (int i = 0; i < length; i++) {
            if (counts[i] >= threshold) {
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
     * 混合法：梯度检测 + 收缩1像素
     *
     * 原理：
     * 1. 找count值变化最剧烈的位置（最大梯度）作为边界
     * 2. 向内收缩1像素，消除边缘模糊
     */
    private int[] findEdgeByGradient(int[] counts, int length) {
        if (length < 3) return new int[]{-1, -1};

        // Step 1: 找count最大值的位置（物体中心）
        int maxCount = 0, maxIndex = 0;
        for (int i = 0; i < length; i++) {
            if (counts[i] > maxCount) {
                maxCount = counts[i];
                maxIndex = i;
            }
        }

        if (maxCount < MIN_PROJECTION_COUNT_ABSOLUTE) {
            return new int[]{-1, -1}; // 没有有效物体
        }

        // Step 2: 从中心向左找最大正梯度
        int leftEdge = 0;
        int maxLeftGradient = 0;
        for (int i = 1; i <= maxIndex; i++) {
            int gradient = counts[i] - counts[i - 1];
            if (gradient > maxLeftGradient) {
                maxLeftGradient = gradient;
                leftEdge = i;
            }
        }

        // Step 3: 从中心向右找最大负梯度
        int rightEdge = length - 1;
        int maxRightGradient = 0; // 负值，所以用0初始化
        for (int i = maxIndex; i < length - 1; i++) {
            int gradient = counts[i + 1] - counts[i]; // 负值
            if (gradient < maxRightGradient) {
                maxRightGradient = gradient;
                rightEdge = i;
            }
        }

        // Step 4: 收缩1像素
        int finalLeft = leftEdge + 1;
        int finalRight = rightEdge - 1;

        // 确保范围有效
        if (finalLeft > finalRight) {
            finalLeft = leftEdge;
            finalRight = rightEdge;
        }

        return new int[]{finalLeft, finalRight};
    }

    /**
     * 连续像素法边界检测 - 直接在 mask 上扫描
     *
     * 原理：直接在 mask 上找连续 >=MIN_CONSECUTIVE_PIXELS(8) 个 mask=1 像素
     * 如果某行/列有连续 >=8 个 mask=1，该行/列有效
     * 边界 = min/max(有效行/列)
     *
     * @return [xMin, xMax, yMin, yMax]
     */
    private int[] findEdgeByDirectConsecutivePixels() {
        int cols = mask.length;
        int rows = mask[0].length;

        if (cols < MIN_CONSECUTIVE_PIXELS || rows < MIN_CONSECUTIVE_PIXELS) {
            return new int[]{-1, -1, -1, -1};
        }

        // 找有效列：每列有多少个连续的 mask=1
        int xMin = -1, xMax = -1;
        for (int x = 0; x < cols; x++) {
            int consecutive = 0;
            for (int y = 0; y < rows; y++) {
                if (mask[x][y] == 1) {
                    consecutive++;
                    if (consecutive >= MIN_CONSECUTIVE_PIXELS) {
                        // 该列有效
                        if (xMin < 0) xMin = x;
                        xMax = x;
                        break;  // 跳到下一列
                    }
                } else {
                    consecutive = 0;
                }
            }
        }

        // 找有效行：每行有多少个连续的 mask=1
        int yMin = -1, yMax = -1;
        for (int y = 0; y < rows; y++) {
            int consecutive = 0;
            for (int x = 0; x < cols; x++) {
                if (mask[x][y] == 1) {
                    consecutive++;
                    if (consecutive >= MIN_CONSECUTIVE_PIXELS) {
                        // 该行有效
                        if (yMin < 0) yMin = y;
                        yMax = y;
                        break;  // 跳到下一行
                    }
                } else {
                    consecutive = 0;
                }
            }
        }

        // 验证结果有效性
        if (xMin < 0 || yMin < 0) {
            return new int[]{-1, -1, -1, -1};
        }

        return new int[]{xMin, xMax, yMin, yMax};
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
    // getColCount 和 getRowCount 在前面已定义（支持克隆数组用于多帧滤波）
    public int getMinProjectionCount() { return dynamicThresholdX; }
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
            if (colCount[x] < dynamicThresholdX) continue;
            for (int y = yMin; y <= yMax; y++) {
                if (rowCount[y] < dynamicThresholdY) continue;
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
        return colCount[x] >= dynamicThresholdX && rowCount[y] >= dynamicThresholdY;
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
