package com.hoho.android.usbserial.examples;

import android.util.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * 多帧缓冲器 - 用于中值滤波降噪
 *
 * 在采集窗口内累积多帧colCount/rowCount，
 * 取中值后用于梯度检测，提高边界检测稳定性
 */
public class FrameBuffer {

    private static final String TAG = "FrameBuffer";

    // 目标帧数
    public static final int TARGET_FRAMES = 5;

    // 最少帧数（少于此用单帧）
    public static final int MIN_FRAMES = 3;

    // 帧历史
    private final List<int[]> colCountsHistory = new ArrayList<>();
    private final List<int[]> rowCountsHistory = new ArrayList<>();

    // 数组长度
    private final int arrayLength;

    public FrameBuffer(int arrayLength) {
        this.arrayLength = arrayLength;
    }

    /**
     * 添加一帧数据
     */
    public synchronized void addFrame(int[] colCount, int[] rowCount) {
        if (colCount == null || rowCount == null) return;

        // 验证数组长度
        if (colCount.length != arrayLength || rowCount.length != arrayLength) {
            Log.w(TAG, String.format("Array length mismatch: expected %d, got col=%d, row=%d",
                    arrayLength, colCount.length, rowCount.length));
            return;
        }

        // 复制数组（避免外部修改影响）
        colCountsHistory.add(Arrays.copyOf(colCount, colCount.length));
        rowCountsHistory.add(Arrays.copyOf(rowCount, rowCount.length));

        // 保持目标帧数
        while (colCountsHistory.size() > TARGET_FRAMES) {
            colCountsHistory.remove(0);
            rowCountsHistory.remove(0);
        }
    }

    /**
     * 是否有足够的帧数
     */
    public synchronized boolean hasEnoughFrames() {
        return colCountsHistory.size() >= MIN_FRAMES;
    }

    /**
     * 获取当前帧数
     */
    public synchronized int getFrameCount() {
        return colCountsHistory.size();
    }

    /**
     * 获取colCount的中值数组
     */
    public synchronized int[] getMedianColCount() {
        if (colCountsHistory.isEmpty()) return null;

        int[] median = new int[arrayLength];
        int[] values = new int[colCountsHistory.size()];

        for (int i = 0; i < arrayLength; i++) {
            // 收集所有帧在该位置的值
            for (int j = 0; j < colCountsHistory.size(); j++) {
                values[j] = colCountsHistory.get(j)[i];
            }
            // 排序取中值
            Arrays.sort(values, 0, colCountsHistory.size());
            median[i] = values[colCountsHistory.size() / 2];
        }

        return median;
    }

    /**
     * 获取rowCount的中值数组
     */
    public synchronized int[] getMedianRowCount() {
        if (rowCountsHistory.isEmpty()) return null;

        int[] median = new int[arrayLength];
        int[] values = new int[rowCountsHistory.size()];

        for (int i = 0; i < arrayLength; i++) {
                for (int j = 0; j < rowCountsHistory.size(); j++) {
                    values[j] = rowCountsHistory.get(j)[i];
                }
                Arrays.sort(values, 0, rowCountsHistory.size());
                median[i] = values[rowCountsHistory.size() / 2];
        }

        return median;
    }

    /**
     * 清空缓冲
     */
    public synchronized void clear() {
        colCountsHistory.clear();
        rowCountsHistory.clear();
    }
}
