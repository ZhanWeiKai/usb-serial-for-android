package com.hoho.android.usbserial.measurement.callback;

import com.hoho.android.usbserial.measurement.model.MeasureError;
import com.hoho.android.usbserial.measurement.model.MeasureResult;
import com.hoho.android.usbserial.measurement.model.MeasureStatus;

/**
 * 连续测量回调
 */
public interface ContinuousMeasureCallback {

    /**
     * 测量结果
     * 每次稳定后回调
     */
    void onResult(MeasureResult result);

    /**
     * 状态变化
     * 如阈值调整、锁定等
     */
    void onStatusChanged(MeasureStatus status);

    /**
     * 发生错误
     */
    void onError(MeasureError error);
}
