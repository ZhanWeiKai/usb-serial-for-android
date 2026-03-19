package com.hoho.android.usbserial.measurement.model;

/**
 * 基线校准结果
 */
public class BaselineResult {

    /** 基线深度矩阵 [100][100]，单位 mm */
    public final float[][] baselineDepth;

    /** 中心点深度 (mm) */
    public final float centerDepth;

    /** X方向每像素对应的物理尺寸 (mm/像素) */
    public final float pixelSizeX;

    /** Y方向每像素对应的物理尺寸 (mm/像素) */
    public final float pixelSizeY;

    /** 采样帧数 */
    public final int frameCount;

    /** 时间戳 */
    public final long timestamp;

    public BaselineResult(float[][] baselineDepth, float centerDepth,
                          float pixelSizeX, float pixelSizeY,
                          int frameCount, long timestamp) {
        this.baselineDepth = baselineDepth;
        this.centerDepth = centerDepth;
        this.pixelSizeX = pixelSizeX;
        this.pixelSizeY = pixelSizeY;
        this.frameCount = frameCount;
        this.timestamp = timestamp;
    }

    /**
     * 获取视野宽度 (mm)
     */
    public float getViewWidthMm() {
        return 100 * pixelSizeX;
    }

    /**
     * 获取视野高度 (mm)
     */
    public float getViewHeightMm() {
        return 100 * pixelSizeY;
    }

    /**
     * 获取视野宽度 (cm)
     */
    public float getViewWidthCm() {
        return getViewWidthMm() / 10f;
    }

    /**
     * 获取视野高度 (cm)
     */
    public float getViewHeightCm() {
        return getViewHeightMm() / 10f;
    }

    @Override
    public String toString() {
        return String.format(
                "BaselineResult{centerDepth=%.0fmm, pixelSize=%.2f×%.2fmm, view=%.1f×%.1fcm}",
                centerDepth, pixelSizeX, pixelSizeY, getViewWidthCm(), getViewHeightCm()
        );
    }
}
