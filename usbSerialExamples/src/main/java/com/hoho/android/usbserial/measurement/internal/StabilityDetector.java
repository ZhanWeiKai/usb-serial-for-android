package com.hoho.android.usbserial.measurement.internal;

import java.util.ArrayList;
import java.util.List;

/**
 * 稳定性检测器
 *
 * 检测连续帧之间的尺寸变化是否在允许范围内
 */
public class StabilityDetector {

    private static final String TAG = "StabilityDetector";

    // 参数
    private final float dimensionThreshold;  // 尺寸偏差阈值 (mm)
    private final int requiredStableCount;  // 需要的稳定帧数

    // 累积数据
    private final List<Float> widths = new ArrayList<>();
    private final List<Float> lengths = new ArrayList<>();
    private final List<Float> heights = new ArrayList<>();

    private float lastWidth = 0;
    private float lastLength = 0;
    private float lastHeight = 0;
    private int stableCount = 0;

    public StabilityDetector(float dimensionThreshold, int requiredStableCount) {
        this.dimensionThreshold = dimensionThreshold;
        this.requiredStableCount = requiredStableCount;
    }

    /**
     * 重置状态
     */
    public void reset() {
        widths.clear();
        lengths.clear();
        heights.clear();
        lastWidth = 0;
        lastLength = 0;
        lastHeight = 0;
        stableCount = 0;
    }

    /**
     * 添加一个测量结果并检测稳定性
     *
     * @param width  宽度 (mm)
     * @param length 长度 (mm)
     * @param height 高度 (mm)
     * @return 是否稳定
     */
    public boolean addAndCheck(float width, float length, float height) {
        float diffW = Math.abs(width - lastWidth);
        float diffL = Math.abs(length - lastLength);
        float diffH = Math.abs(height - lastHeight);
        float maxDiff = Math.max(diffW, Math.max(diffL, diffH));

        if (maxDiff < dimensionThreshold) {
            // 稳定
            stableCount++;
            lastWidth = width;
            lastLength = length;
            lastHeight = height;

            widths.add(width);
            lengths.add(length);
            heights.add(height);

            return true;
        } else {
            // 不稳定，重新开始
            stableCount = 1;
            lastWidth = width;
            lastLength = length;
            lastHeight = height;

            widths.clear();
            lengths.clear();
            heights.clear();
            widths.add(width);
            lengths.add(length);
            heights.add(height);

            return false;
        }
    }

    /**
     * 获取当前稳定帧数
     */
    public int getStableCount() {
        return stableCount;
    }

    /**
     * 是否已达到稳定要求
     */
    public boolean isStableEnough() {
        return stableCount >= requiredStableCount;
    }

    /**
     * 获取平均结果（仅在稳定后调用）
     */
    public float getAverageWidth() {
        return average(widths);
    }

    public float getAverageLength() {
        return average(lengths);
    }

    public float getAverageHeight() {
        return average(heights);
    }

    private float average(List<Float> list) {
        if (list.isEmpty()) return 0;
        float sum = 0;
        for (float v : list) {
            sum += v;
        }
        return sum / list.size();
    }
}
