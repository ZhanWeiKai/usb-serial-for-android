package com.hoho.android.usbserial.measurement.model;

/**
 * 物体测量结果
 */
public class MeasureResult {

    // ========== 核心结果 ==========

    /** 宽度 (mm) */
    public final float width;

    /** 长度 (mm) */
    public final float length;

    /** 高度 (mm) */
    public final float height;

    /** 体积 (cm³) */
    public final float volumeCm3;

    // ========== 质量指标 ==========

    /** 原始 mask 像素数 */
    public final int rawPixelCount;

    /** 有效像素数 */
    public final int validPixelCount;

    /** 最大深度差 (mm) */
    public final float maxDepthDiff;

    /** 物体表面深度 (mm) */
    public final float objectSurfaceDepth;

    // ========== 边界范围 ==========

    /** X方向最小像素索引 */
    public final int xMin;

    /** X方向最大像素索引 */
    public final int xMax;

    /** Y方向最小像素索引 */
    public final int yMin;

    /** Y方向最大像素索引 */
    public final int yMax;

    // ========== 状态信息 ==========

    /** 是否已稳定（连续10帧） */
    public final boolean isStable;

    /** 稳定帧计数 */
    public final int stableCount;

    /** 状态消息 */
    public final String message;

    public MeasureResult(float width, float length, float height,
                         int rawPixelCount, int validPixelCount,
                         float maxDepthDiff, float objectSurfaceDepth,
                         int xMin, int xMax, int yMin, int yMax,
                         boolean isStable, int stableCount, String message) {
        this.width = width;
        this.length = length;
        this.height = height;
        this.volumeCm3 = width * length * height / 1000f;
        this.rawPixelCount = rawPixelCount;
        this.validPixelCount = validPixelCount;
        this.maxDepthDiff = maxDepthDiff;
        this.objectSurfaceDepth = objectSurfaceDepth;
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;
        this.isStable = isStable;
        this.stableCount = stableCount;
        this.message = message;
    }

    /**
     * 创建空结果
     */
    public static MeasureResult empty() {
        return new MeasureResult(0, 0, 0, 0, 0, 0, 0,
                -1, -1, -1, -1, false, 0, "无结果");
    }

    /**
     * 创建错误结果
     */
    public static MeasureResult error(String message) {
        return new MeasureResult(0, 0, 0, 0, 0, 0, 0,
                -1, -1, -1, -1, false, 0, message);
    }

    // ========== 工具方法 ==========

    /** 宽度 (cm) */
    public float getWidthCm() {
        return width / 10f;
    }

    /** 长度 (cm) */
    public float getLengthCm() {
        return length / 10f;
    }

    /** 高度 (cm) */
    public float getHeightCm() {
        return height / 10f;
    }

    /** X方向像素跨度 */
    public int getXSpan() {
        return xMax >= xMin ? (xMax - xMin + 1) : 0;
    }

    /** Y方向像素跨度 */
    public int getYSpan() {
        return yMax >= yMin ? (yMax - yMin + 1) : 0;
    }

    /**
     * 格式化输出
     */
    public String getFormattedResult() {
        StringBuilder sb = new StringBuilder();
        sb.append("══════ 测量结果 ══════\n\n");
        sb.append(String.format("宽(W): %.1f mm  (%.1f cm)\n", width, getWidthCm()));
        sb.append(String.format("长(L): %.1f mm  (%.1f cm)\n", length, getLengthCm()));
        sb.append(String.format("高(H): %.1f mm  (%.1f cm)\n", height, getHeightCm()));
        sb.append("─────────────────────\n");
        sb.append(String.format("体积: %.1f cm³\n", volumeCm3));
        sb.append(String.format("有效像素: %d (原始: %d)\n", validPixelCount, rawPixelCount));
        sb.append(String.format("像素范围: x[%d-%d] y[%d-%d]\n", xMin, xMax, yMin, yMax));
        sb.append("═════════════════════");
        return sb.toString();
    }

    @Override
    public String toString() {
        return String.format("MeasureResult{W=%.1f, L=%.1f, H=%.1f mm, V=%.1fcm³, stable=%s}",
                width, length, height, volumeCm3, isStable ? "Y" : "N");
    }
}
