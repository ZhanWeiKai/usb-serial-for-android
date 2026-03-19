package com.hoho.android.usbserial.measurement.model;

/**
 * 测量状态描述
 */
public class MeasureStatus {

    /** 当前阈值 (mm) */
    public final float currentThreshold;

    /** 阈值是否已锁定 */
    public final boolean thresholdLocked;

    /** 不稳定计数 */
    public final int unstableCount;

    /** 稳定计数 */
    public final int stableCount;

    /** 状态描述 */
    public final String message;

    public MeasureStatus(float currentThreshold, boolean thresholdLocked,
                         int unstableCount, int stableCount) {
        this.currentThreshold = currentThreshold;
        this.thresholdLocked = thresholdLocked;
        this.unstableCount = unstableCount;
        this.stableCount = stableCount;

        // 自动生成状态描述
        if (thresholdLocked) {
            this.message = String.format("阈值:%.0fmm(已锁定)", currentThreshold);
        } else {
            this.message = String.format("阈值:%.0fmm 不稳:%d 稳:%d/10",
                    currentThreshold, unstableCount, stableCount);
        }
    }

    /**
     * 创建初始状态
     */
    public static MeasureStatus initial(float threshold) {
        return new MeasureStatus(threshold, false, 0, 0);
    }

    @Override
    public String toString() {
        return message;
    }
}
