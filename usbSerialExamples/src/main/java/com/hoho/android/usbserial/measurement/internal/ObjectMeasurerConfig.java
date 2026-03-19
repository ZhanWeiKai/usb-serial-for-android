package com.hoho.android.usbserial.measurement.internal;

/**
 * ObjectMeasurer 配置类
 */
public class ObjectMeasurerConfig {

    // ========== 传感器参数 ==========

    /** 水平视场角 (度) */
    public final float fovHorizontal;

    /** 垂直视场角 (度) */
    public final float fovVertical;

    /** 俯视角度 (度) */
    public final float tiltAngle;

    // ========== 检测参数 ==========

    /** 物体检测阈值 (mm) */
    public final float objectThreshold;

    /** 阈值上限 (mm) */
    public final float maxThreshold;

    // ========== 采样参数 ==========

    /** 每窗口采样帧数 */
    public final int samplesPerWindow;

    /** 窗口间隔 (ms) */
    public final long sampleIntervalMs;

    /** 窗口内采样间隔 (ms) */
    public final long sampleFastIntervalMs;

    // ========== 稳定性参数 ==========

    /** 校准所需稳定帧数 */
    public final int stableCountCalibrate;

    /** 测量所需稳定帧数 */
    public final int stableCountMeasure;

    /** 帧间差异阈值 (mm) */
    public final float baselineAvgThreshold;

    /** 尺寸偏差阈值 (mm) */
    public final float dimensionThreshold;

    // ========== 边界检测参数 ==========

    /** 最小连续像素数 */
    public final int minConsecutivePixels;

    // ========== 计算缓存 ==========

    /** tan(FOV_H/2) */
    public final double tanFovHHalf;

    /** tan(FOV_V/2) */
    public final double tanFovVHalf;

    /** cos(tiltAngle) */
    public final double cosTilt;

    /** sin(tiltAngle) */
    public final double sinTilt;

    private ObjectMeasurerConfig(Builder builder) {
        this.fovHorizontal = builder.fovHorizontal;
        this.fovVertical = builder.fovVertical;
        this.tiltAngle = builder.tiltAngle;
        this.objectThreshold = builder.objectThreshold;
        this.maxThreshold = builder.maxThreshold;
        this.samplesPerWindow = builder.samplesPerWindow;
        this.sampleIntervalMs = builder.sampleIntervalMs;
        this.sampleFastIntervalMs = builder.sampleIntervalMs / builder.samplesPerWindow;
        this.stableCountCalibrate = builder.stableCountCalibrate;
        this.stableCountMeasure = builder.stableCountMeasure;
        this.baselineAvgThreshold = builder.baselineAvgThreshold;
        this.dimensionThreshold = builder.dimensionThreshold;
        this.minConsecutivePixels = builder.minConsecutivePixels;

        // 预计算三角函数
        this.tanFovHHalf = Math.tan(Math.toRadians(fovHorizontal / 2.0));
        this.tanFovVHalf = Math.tan(Math.toRadians(fovVertical / 2.0));
        this.cosTilt = Math.cos(Math.toRadians(tiltAngle));
        this.sinTilt = Math.sin(Math.toRadians(tiltAngle));
    }

    /**
     * 计算像素尺寸
     * @param depth 深度值 (mm)
     * @return [pixelSizeX, pixelSizeY]
     */
    public float[] calculatePixelSize(float depth) {
        float pixelSizeX = (float) (2.0 * depth * tanFovHHalf / 100.0);
        float pixelSizeY = (float) (2.0 * depth * tanFovVHalf / 100.0);
        return new float[]{pixelSizeX, pixelSizeY};
    }

    /**
     * 创建默认配置
     */
    public static ObjectMeasurerConfig createDefault() {
        return new Builder().build();
    }

    public static class Builder {
        private float fovHorizontal = 70f;
        private float fovVertical = 60f;
        private float tiltAngle = 45f;
        private float objectThreshold = 50f;
        private float maxThreshold = 100f;
        private int samplesPerWindow = 10;
        private long sampleIntervalMs = 500;
        private int stableCountCalibrate = 10;
        private int stableCountMeasure = 10;
        private float baselineAvgThreshold = 14f;
        private float dimensionThreshold = 20f;
        private int minConsecutivePixels = 8;

        public Builder setFov(float horizontal, float vertical) {
            this.fovHorizontal = horizontal;
            this.fovVertical = vertical;
            return this;
        }

        public Builder setTiltAngle(float angle) {
            this.tiltAngle = angle;
            return this;
        }

        public Builder setObjectThreshold(float threshold) {
            this.objectThreshold = threshold;
            return this;
        }

        public Builder setMaxThreshold(float threshold) {
            this.maxThreshold = threshold;
            return this;
        }

        public Builder setSamplesPerWindow(int count) {
            this.samplesPerWindow = count;
            return this;
        }

        public Builder setSampleInterval(long intervalMs) {
            this.sampleIntervalMs = intervalMs;
            return this;
        }

        public Builder setStableCountCalibrate(int count) {
            this.stableCountCalibrate = count;
            return this;
        }

        public Builder setStableCountMeasure(int count) {
            this.stableCountMeasure = count;
            return this;
        }

        public Builder setBaselineAvgThreshold(float threshold) {
            this.baselineAvgThreshold = threshold;
            return this;
        }

        public Builder setDimensionThreshold(float threshold) {
            this.dimensionThreshold = threshold;
            return this;
        }

        public Builder setMinConsecutivePixels(int count) {
            this.minConsecutivePixels = count;
            return this;
        }

        public ObjectMeasurerConfig build() {
            return new ObjectMeasurerConfig(this);
        }
    }
}
