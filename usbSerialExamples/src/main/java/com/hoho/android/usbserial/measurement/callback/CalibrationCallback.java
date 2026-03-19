package com.hoho.android.usbserial.measurement.callback;

import com.hoho.android.usbserial.measurement.model.BaselineResult;
import com.hoho.android.usbserial.measurement.model.MeasureError;

/**
 * 校准回调
 */
public interface CalibrationCallback {

    /**
     * 进度更新
     * @param current 当前稳定帧数
     * @param total 需要的总帧数
     * @param avgDifference 帧间平均差异 (mm)
     */
    void onProgress(int current, int total, float avgDifference);

    /**
     * 校准成功
     */
    void onSuccess(BaselineResult result);

    /**
     * 校准失败
     */
    void onError(MeasureError error);
}
