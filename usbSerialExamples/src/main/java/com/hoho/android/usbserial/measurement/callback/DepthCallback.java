package com.hoho.android.usbserial.measurement.callback;

import com.hoho.android.usbserial.measurement.model.MeasureError;

/**
 * 深度数据回调（单次）
 */
public interface DepthCallback {

    /**
     * 收到深度数据
     * @param depthMatrix 深度矩阵 [100][100]，单位 mm
     * @param timestamp 时间戳 (ms)
     */
    void onDepthData(float[][] depthMatrix, long timestamp);

    /**
     * 发生错误
     */
    void onError(MeasureError error);
}
