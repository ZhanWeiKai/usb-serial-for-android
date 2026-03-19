package com.hoho.android.usbserial.measurement.internal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * 中值滤波器
 *
 * 对深度数据进行中值降噪处理
 */
public class MedianFilter {

    private static final String TAG = "MedianFilter";

    /**
     * 对多帧深度数据取中值（逐像素）
     *
     * @param frames 帧列表，每帧为 [100][100]
     * @return 中值结果 [100][100]
     */
    public static float[][] medianFrames(List<float[][]> frames) {
        if (frames == null || frames.isEmpty()) return null;

        int rows = frames.get(0).length;
        int cols = frames.get(0)[0].length;
        float[][] result = new float[rows][cols];
        int frameCount = frames.size();

        // 临时数组用于排序
        float[] values = new float[frameCount];

        for (int x = 0; x < rows; x++) {
            for (int y = 0; y < cols; y++) {
                // 收集所有帧在该像素位置的深度值
                for (int f = 0; f < frameCount; f++) {
                    values[f] = frames.get(f)[x][y];
                }

                // 排序
                Arrays.sort(values);

                // 取中值
                if (frameCount % 2 == 1) {
                    result[x][y] = values[frameCount / 2];
                } else {
                    // 偶数帧取中间两个的平均
                    result[x][y] = (values[frameCount / 2 - 1] + values[frameCount / 2]) / 2.0f;
                }
            }
        }

        return result;
    }

    /**
     * 从多帧中选择 maxDepthDiff 中位数对应的那一帧
     *
     * @param frames 帧列表
     * @param maxDepthDiffs 每帧对应的 maxDepthDiff 列表
     * @return 选中的帧
     */
    public static float[][] selectMedianMaxDiffFrame(List<float[][]> frames, List<Float> maxDepthDiffs) {
        if (maxDepthDiffs.isEmpty() || frames.isEmpty()) {
            return frames.isEmpty() ? null : frames.get(0);
        }

        // 复制并排序，找中位数
        List<Float> sorted = new ArrayList<>(maxDepthDiffs);
        Collections.sort(sorted);
        int midIndex = sorted.size() / 2;
        float medianValue = sorted.get(midIndex);

        // 找到最接近中位数的帧索引
        int bestFrameIndex = 0;
        float minDiff = Float.MAX_VALUE;
        for (int i = 0; i < maxDepthDiffs.size(); i++) {
            float diff = Math.abs(maxDepthDiffs.get(i) - medianValue);
            if (diff < minDiff) {
                minDiff = diff;
                bestFrameIndex = i;
            }
        }

        return frames.get(bestFrameIndex);
    }

    /**
     * 计算帧的平均差异
     *
     * @param frame1 帧1
     * @param frame2 帧2
     * @return 平均差异 (mm)
     */
    public static float calculateAverageDifference(float[][] frame1, float[][] frame2) {
        float totalDiff = 0;
        int count = 0;

        for (int x = 0; x < frame1.length; x++) {
            for (int y = 0; y < frame1[x].length; y++) {
                float v1 = frame1[x][y];
                float v2 = frame2[x][y];

                // 忽略无效值和异常值
                if (v1 <= 0 || v2 <= 0) continue;
                if (v1 < 100 || v1 > 1000) continue;
                if (v2 < 100 || v2 > 1000) continue;

                // 过滤单点差异过大的（噪声）
                float diff = Math.abs(v1 - v2);
                if (diff > 500) continue;

                totalDiff += diff;
                count++;
            }
        }

        return count > 0 ? totalDiff / count : 0;
    }
}
