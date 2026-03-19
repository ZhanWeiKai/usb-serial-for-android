package com.hoho.android.usbserial.measurement.internal;

import android.util.Log;

/**
 * 动态阈值管理器
 *
 * 根据边界稳定性动态调整 mask 阈值
 */
public class DynamicThresholdManager {

    private static final String TAG = "DynamicThreshold";

    // 默认参数
    private static final float DEFAULT_THRESHOLD = 50.0f;   // mm
    private static final float MAX_THRESHOLD = 100.0f;    // mm
    private static final float THRESHOLD_STEP = 5.0f;      // 每次增加 5mm
    private static final int PIXEL_CHANGE_THRESHOLD = 3;     // 边界变化阈值
    private static final int UNSTABLE_COUNT_TO_ADJUST = 3; // 累计3次不稳定才调整
    private static final int STABLE_COUNT_TO_LOCK = 10;    // 连续10次稳定才锁定

    // 状态
    private float currentThreshold;
    private int unstableCount;
    private int stableCount;
    private boolean thresholdLocked;
    private int lastXSpan;
    private int lastYSpan;

    public DynamicThresholdManager() {
        reset();
    }

    /**
     * 重置状态
     */
    public void reset() {
        currentThreshold = DEFAULT_THRESHOLD;
        unstableCount = 00;
        stableCount = 00;
        thresholdLocked = false;
        lastXSpan = 00;
        lastYSpan = 00;
    }

    /**
     * 更新动态阈值
     *
     * @param xSpan X方向像素跨度
     * @param ySpan Y方向像素跨度
     * @return 状态描述
     */
    public ThresholdUpdateResult update(int xSpan, int ySpan) {
        int xChange = Math.abs(xSpan - lastXSpan);
        int yChange = Math.abs(ySpan - lastYSpan);
        int pixelChange = Math.max(xChange, yChange);

        ThresholdUpdateResult result = new ThresholdUpdateResult();
        result.currentThreshold = currentThreshold;
        result.thresholdLocked = thresholdLocked;
        result.unstableCount = unstableCount;
        result.stableCount = stableCount;

        if (pixelChange >= PIXEL_CHANGE_THRESHOLD) {
            // 边界有跳动 → 不稳定
            unstableCount++;
            stableCount = 0;

            if (unstableCount >= UNSTABLE_COUNT_TO_ADJUST && !thresholdLocked) {
                // 累计3次不稳定，收紧阈值
                currentThreshold += THRESHOLD_STEP;
                if (currentThreshold > MAX_THRESHOLD) {
                    currentThreshold = MAX_THRESHOLD;
                }

                // 阈值调整后，重置计数
                unstableCount = 0;
                stableCount = 0;

                result.adjusted = true;
                Log.d(TAG, String.format("动态阈值收紧至: %.0fmm (像素变化=%d)",
                        currentThreshold, pixelChange));
            }
        } else {
            // 边界稳定
            if (!thresholdLocked) {
                stableCount++;
                if (stableCount >= STABLE_COUNT_TO_LOCK) {
                    // 连续10次稳定 → 锁定阈值
                    thresholdLocked = true;
                    result.locked = true;
                    Log.d(TAG, String.format("动态阈值锁定在: %.0fmm", currentThreshold));
                }
            }
        }

        // 更新上一帧数据
        lastXSpan = xSpan;
        lastYSpan = ySpan;

        // 生成状态描述
        if (thresholdLocked) {
            result.message = String.format("阈值:%.0fmm(已锁定)", currentThreshold);
        } else {
            result.message = String.format("阈值:%.0fmm 不稳:%d 稳:%d/10",
                    currentThreshold, unstableCount, stableCount);
        }

        return result;
    }

    /**
     * 获取当前阈值
     */
    public float getCurrentThreshold() {
        return currentThreshold;
    }

    /**
     * 阈值是否已锁定
     */
    public boolean isThresholdLocked() {
        return thresholdLocked;
    }

    /**
     * 阈值更新结果
     */
    public static class ThresholdUpdateResult {
        public float currentThreshold;
        public boolean thresholdLocked;
        public int unstableCount;
        public int stableCount;
        public boolean adjusted;  // 是否刚调整过
        public boolean locked;     // 是否刚锁定
        public String message;
    }
}
