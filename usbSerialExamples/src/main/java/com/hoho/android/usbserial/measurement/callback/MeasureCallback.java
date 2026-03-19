package com.hoho.android.usbserial.measurement.callback;

import com.hoho.android.usbserial.measurement.model.MeasureError;
import com.hoho.android.usbserial.measurement.model.MeasureResult;

/**
 * 单次测量回调
 */
public interface MeasureCallback {

    /**
     * 进度更新
     * @param current 当前稳定帧数
     * @param total 需要的总帧数
     * @param intermediate 中间结果
     */
    void onProgress(int current, int total, MeasureResult intermediate);

    /**
     * 测量成功
     */
    void onSuccess(MeasureResult result);

    /**
     * 测量失败
     */
    void onError(MeasureError error);
}
