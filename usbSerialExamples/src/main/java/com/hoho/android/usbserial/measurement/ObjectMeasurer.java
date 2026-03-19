package com.hoho.android.usbserial.measurement;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.examples.ObjectDimensionCalculator;
import com.hoho.android.usbserial.measurement.callback.CalibrationCallback;
import com.hoho.android.usbserial.measurement.callback.ContinuousDepthCallback;
import com.hoho.android.usbserial.measurement.callback.ContinuousMeasureCallback;
import com.hoho.android.usbserial.measurement.callback.DepthCallback;
import com.hoho.android.usbserial.measurement.callback.MeasureCallback;
import com.hoho.android.usbserial.measurement.internal.DynamicThresholdManager;
import com.hoho.android.usbserial.measurement.internal.MedianFilter;
import com.hoho.android.usbserial.measurement.internal.ObjectMeasurerConfig;
import com.hoho.android.usbserial.measurement.internal.StabilityDetector;
import com.hoho.android.usbserial.measurement.model.BaselineResult;
import com.hoho.android.usbserial.measurement.model.MeasureError;
import com.hoho.android.usbserial.measurement.model.MeasureResult;
import com.hoho.android.usbserial.measurement.model.MeasureStatus;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * 物体尺寸测量器
 *
 * 使用流程：
 * 1. 创建实例并配置
 * 2. 调用 start() 开始接收数据
 * 3. 调用 calibrateBaseline() 校准基线
 * 4. 调用 measureOnce() 或 startContinuousMeasure() 测量物体
 * 5. 调用 stop() / release() 释放资源
 */
public class ObjectMeasurer {

    private static final String TAG = "ObjectMeasurer";

    // ========== 配置 ==========
    private final ObjectMeasurerConfig config;
    private final UsbSerialPort usbSerialPort;

    // ========== 内部组件 ==========
    private final DynamicThresholdManager thresholdManager;
    private StabilityDetector calibrateStabilityDetector;
    private StabilityDetector measureStabilityDetector;

    // ========== 状态 ==========
    private State state = State.IDLE;
    private float[][] baselineDepth;
    private float pixelSizeX;
    private float pixelSizeY;

    // ========== 采样相关 ==========
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final Handler samplingHandler = new Handler(Looper.getMainLooper());
    private Runnable samplingRunnable;
    private final ByteArrayOutputStream buffer = new ByteArrayOutputStream();
    private final Object bufferLock = new Object();

    // 窗口采样
    private final List<float[][]> windowFrames = new ArrayList<>();
    private final List<Float> windowMaxDepthDiffs = new ArrayList<>();
    private int windowFrameCount = 0;
    private float[][] latestMedianDepth;

    // 回调
    private CalibrationCallback calibrationCallback;
    private MeasureCallback measureCallback;
    private ContinuousMeasureCallback continuousMeasureCallback;
    private ContinuousDepthCallback continuousDepthCallback;
    private DepthCallback singleDepthCallback;

    // 校准帧累积
    private final List<float[][]> calibrationFrames = new ArrayList<>();
    private int calibrateStableCount = 0;

    // 噪声掩码
    private boolean[][] noiseMask;

    // ========== 运行标志 ==========
    private volatile boolean running = false;
    private volatile byte[] latestDepthData;

    /**
     * 状态枚举
     */
    public enum State {
        IDLE,           // 空闲
        CALIBRATING,    // 校准中
        CALIBRATED,     // 已校准
        MEASURING,      // 测量中
        ERROR           // 错误
    }

    // ==================== Builder ====================

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private UsbSerialPort usbSerialPort;
        private float fovHorizontal = 70f;
        private float fovVertical = 60f;
        private float tiltAngle = 45f;
        private float threshold = 50f;
        private int samplesPerWindow = 10;
        private long sampleIntervalMs = 500;
        private int stableCountCalibrate = 10;
        private int stableCountMeasure = 10;

        public Builder setUsbSerialPort(UsbSerialPort port) {
            this.usbSerialPort = port;
            return this;
        }

        public Builder setFov(float horizontalDeg, float verticalDeg) {
            this.fovHorizontal = horizontalDeg;
            this.fovVertical = verticalDeg;
            return this;
        }

        public Builder setTiltAngle(float angleDeg) {
            this.tiltAngle = angleDeg;
            return this;
        }

        public Builder setThreshold(float thresholdMm) {
            this.threshold = thresholdMm;
            return this;
        }

        public Builder setSampleWindow(int samplesPerWindow, long intervalMs) {
            this.samplesPerWindow = samplesPerWindow;
            this.sampleIntervalMs = intervalMs;
            return this;
        }

        public Builder setStableCount(int count) {
            this.stableCountCalibrate = count;
            this.stableCountMeasure = count;
            return this;
        }

        public ObjectMeasurer build() {
            if (usbSerialPort == null) {
                throw new IllegalStateException("UsbSerialPort is required");
            }
            ObjectMeasurerConfig config = new ObjectMeasurerConfig.Builder()
                    .setFov(fovHorizontal, fovVertical)
                    .setTiltAngle(tiltAngle)
                    .setObjectThreshold(threshold)
                    .setMaxThreshold(100f)
                    .setSamplesPerWindow(samplesPerWindow)
                    .setSampleInterval(sampleIntervalMs)
                    .setStableCountCalibrate(stableCountCalibrate)
                    .setStableCountMeasure(stableCountMeasure)
                    .setBaselineAvgThreshold(14f)
                    .setDimensionThreshold(20f)
                    .setMinConsecutivePixels(8)
                    .build();
            return new ObjectMeasurer(usbSerialPort, config);
        }
    }

    // ==================== 构造函数 ====================

    private ObjectMeasurer(UsbSerialPort usbSerialPort, ObjectMeasurerConfig config) {
        this.usbSerialPort = usbSerialPort;
        this.config = config;
        this.thresholdManager = new DynamicThresholdManager();
    }

    // ==================== 生命周期 ====================

    /**
     * 开始接收数据
     */
    public void start() {
        if (running) return;
        running = true;
        startUsbReadThread();
        Log.d(TAG, "ObjectMeasurer started");
    }

    /**
     * 停止接收数据
     */
    public void stop() {
        running = false;
        stopWindowSampling();
        Log.d(TAG, "ObjectMeasurer stopped");
    }

    /**
     * 释放所有资源
     */
    public void release() {
        stop();
        mainHandler.removeCallbacksAndMessages(null);
        samplingHandler.removeCallbacksAndMessages(null);
        baselineDepth = null;
        calibrationFrames.clear();
        windowFrames.clear();
        Log.d(TAG, "ObjectMeasurer released");
    }

    // ==================== USB 读取线程 ====================

    private void startUsbReadThread() {
        new Thread(() -> {
            byte[] readBuffer = new byte[10240];
            while (running) {
                try {
                    int len = usbSerialPort.read(readBuffer, 100);
                    if (len > 0) {
                        synchronized (bufferLock) {
                            buffer.write(readBuffer, 0, len);
                            processBuffer();
                        }
                    }
                } catch (Exception e) {
                    Log.e(TAG, "USB read error: " + e.getMessage());
                    if (running) {
                        notifyError(MeasureError.of(MeasureError.ErrorCode.USB_READ_FAILED, e.getMessage()));
                    }
                }
            }
        }, "UsbReadThread").start();
    }

    private void processBuffer() {
        // 帧格式: 包头2字节 + 长度2字节 + 元数据16字节 + 深度数据10000字节 + 校验2字节
        // 简化处理：假设每帧是完整的
        byte[] data = buffer.toByteArray();
        if (data.length >= 10192) {  // 最小帧大小
            // 查找包头
            for (int i = 0; i <= data.length - 10192; i++) {
                if (data[i] == 0x00 && data[i + 1] == (byte) 0xFF) {
                    // 提取深度数据（跳过包头2 + 长度2 + 元数据16）
                    int depthOffset = i + 2 + 2 + 16;
                    if (depthOffset + 10000 <= data.length) {
                        byte[] depthData = new byte[10000];
                        System.arraycopy(data, depthOffset, depthData, 0, 10000);
                        latestDepthData = depthData;

                        // 清除已处理的数据
                        int consumed = depthOffset + 10000 + 2;  // +2 for checksum
                        byte[] remaining = new byte[data.length - consumed];
                        System.arraycopy(data, consumed, remaining, 0, remaining.length);
                        buffer.reset();
                        buffer.write(remaining, 0, remaining.length);
                        break;
                    }
                }
            }
        }
    }

    // ==================== 深度数据 ====================

    /**
     * 单次获取深度数据
     */
    public void getDepthData(DepthCallback callback) {
        if (!running) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.USB_NOT_CONNECTED));
            return;
        }
        this.singleDepthCallback = callback;
        // 等待下一帧数据后回调
        mainHandler.postDelayed(() -> {
            if (singleDepthCallback == callback && latestDepthData != null) {
                float[][] depthMatrix = convertToDistanceMatrix(latestDepthData);
                callback.onDepthData(depthMatrix, System.currentTimeMillis());
                singleDepthCallback = null;
            } else if (singleDepthCallback == callback) {
                callback.onError(MeasureError.of(MeasureError.ErrorCode.NO_DEPTH_DATA));
                singleDepthCallback = null;
            }
        }, 200);
    }

    /**
     * 连续获取深度数据
     */
    public void getDepthDataContinuous(ContinuousDepthCallback callback) {
        if (!running) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.USB_NOT_CONNECTED));
            return;
        }
        this.continuousDepthCallback = callback;
        startContinuousDepthDelivery();
    }

    /**
     * 停止连续获取深度数据
     */
    public void stopDepthData() {
        continuousDepthCallback = null;
    }

    private void startContinuousDepthDelivery() {
        mainHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (continuousDepthCallback != null && running) {
                    if (latestDepthData != null) {
                        float[][] depthMatrix = convertToDistanceMatrix(latestDepthData);
                        continuousDepthCallback.onDepthData(depthMatrix, System.currentTimeMillis());
                    }
                    mainHandler.postDelayed(this, 100);  // 10Hz
                }
            }
        }, 100);
    }

    // ==================== 校准 ====================

    /**
     * 开始基线校准
     */
    public void calibrateBaseline(CalibrationCallback callback) {
        if (!running) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.USB_NOT_CONNECTED));
            return;
        }
        if (state == State.CALIBRATING) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.CALIBRATION_IN_PROGRESS));
            return;
        }
        if (state == State.MEASURING) {
            stopMeasure();
        }

        this.calibrationCallback = callback;
        this.state = State.CALIBRATING;
        this.calibrateStabilityDetector = new StabilityDetector(
                config.baselineAvgThreshold, config.stableCountCalibrate);
        calibrationFrames.clear();
        calibrateStableCount = 0;
        baselineDepth = null;

        Log.d(TAG, "开始基线校准");
        startWindowSampling(this::processCalibrationSample);
    }

    private void processCalibrationSample() {
        if (calibrationCallback == null || latestMedianDepth == null) return;

        float[][] currentFrame = copyDepthArray(latestMedianDepth);

        if (baselineDepth == null) {
            baselineDepth = copyDepthArray(currentFrame);
            calibrationFrames.add(copyDepthArray(currentFrame));
            calibrateStableCount = 1;
            notifyCalibrationProgress(1);
        } else {
            float avgDiff = MedianFilter.calculateAverageDifference(baselineDepth, currentFrame);

            if (avgDiff < config.baselineAvgThreshold) {
                calibrateStableCount++;
                calibrationFrames.add(copyDepthArray(currentFrame));
                notifyCalibrationProgress(calibrateStableCount);

                if (calibrateStableCount >= config.stableCountCalibrate) {
                    finishCalibration();
                }
            } else {
                // 不稳定，重新开始
                baselineDepth = copyDepthArray(currentFrame);
                calibrationFrames.clear();
                calibrationFrames.add(copyDepthArray(currentFrame));
                calibrateStableCount = 1;
                notifyCalibrationProgress(1);
            }
        }
    }

    private void finishCalibration() {
        stopWindowSampling();

        // 多帧中值作为最终基线
        baselineDepth = MedianFilter.medianFrames(calibrationFrames);
        int frameCount = calibrationFrames.size();
        calibrationFrames.clear();

        // 计算像素尺寸
        float centerDepth = baselineDepth[50][50];
        double tanFovHHalf = Math.tan(Math.toRadians(config.fovHorizontal / 2.0));
        double tanFovVHalf = Math.tan(Math.toRadians(config.fovVertical / 2.0));
        pixelSizeX = (float) (2.0 * centerDepth * tanFovHHalf / 100.0);
        pixelSizeY = (float) (2.0 * centerDepth * tanFovVHalf / 100.0);

        state = State.CALIBRATED;
        thresholdManager.reset();

        BaselineResult result = new BaselineResult(
                baselineDepth, centerDepth,
                pixelSizeX, pixelSizeY,
                frameCount, System.currentTimeMillis()
        );

        Log.d(TAG, String.format("校准完成: centerDepth=%.0fmm, pixelSize=%.2f×%.2fmm",
                centerDepth, pixelSizeX, pixelSizeY));

        if (calibrationCallback != null) {
            CalibrationCallback cb = calibrationCallback;
            calibrationCallback = null;
            cb.onSuccess(result);
        }
    }

    private void notifyCalibrationProgress(int current) {
        if (calibrationCallback != null) {
            final int c = current;
            final int total = config.stableCountCalibrate;
            mainHandler.post(() -> {
                if (calibrationCallback != null) {
                    calibrationCallback.onProgress(c, total, 0);
                }
            });
        }
    }

    /**
     * 是否已有基线数据
     */
    public boolean hasBaseline() {
        return baselineDepth != null;
    }

    /**
     * 获取当前基线数据
     */
    public BaselineResult getBaseline() {
        if (baselineDepth == null) return null;
        float centerDepth = baselineDepth[50][50];
        return new BaselineResult(
                baselineDepth, centerDepth,
                pixelSizeX, pixelSizeY,
                0, System.currentTimeMillis()
        );
    }

    /**
     * 清除基线数据
     */
    public void clearBaseline() {
        baselineDepth = null;
        state = State.IDLE;
    }

    // ==================== 测量 ====================

    /**
     * 单次测量
     */
    public void measureOnce(MeasureCallback callback) {
        if (!running) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.USB_NOT_CONNECTED));
            return;
        }
        if (!hasBaseline()) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.CALIBRATION_NOT_READY));
            return;
        }
        if (state == State.MEASURING) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.MEASUREMENT_IN_PROGRESS));
            return;
        }

        this.measureCallback = callback;
        this.state = State.MEASURING;
        this.measureStabilityDetector = new StabilityDetector(
                config.dimensionThreshold, config.stableCountMeasure);
        this.continuousMeasureCallback = null;
        thresholdManager.reset();

        Log.d(TAG, "开始单次测量");
        startWindowSampling(this::processMeasurementSample);
    }

    /**
     * 开始连续测量
     */
    public void startContinuousMeasure(ContinuousMeasureCallback callback) {
        if (!running) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.USB_NOT_CONNECTED));
            return;
        }
        if (!hasBaseline()) {
            callback.onError(MeasureError.of(MeasureError.ErrorCode.CALIBRATION_NOT_READY));
            return;
        }

        this.continuousMeasureCallback = callback;
        this.state = State.MEASURING;
        this.measureStabilityDetector = new StabilityDetector(
                config.dimensionThreshold, config.stableCountMeasure);
        this.measureCallback = null;
        thresholdManager.reset();

        Log.d(TAG, "开始连续测量");
        startWindowSampling(this::processMeasurementSample);
    }

    /**
     * 停止测量
     */
    public void stopMeasure() {
        stopWindowSampling();
        measureCallback = null;
        continuousMeasureCallback = null;
        if (state == State.MEASURING) {
            state = hasBaseline() ? State.CALIBRATED : State.IDLE;
        }
        Log.d(TAG, "测量已停止");
    }

    private void processMeasurementSample() {
        if ((measureCallback == null && continuousMeasureCallback == null) ||
                latestMedianDepth == null || baselineDepth == null) {
            return;
        }

        float[][] currentDepth = copyDepthArray(latestMedianDepth);

        // 使用动态阈值创建计算器
        ObjectDimensionCalculator calculator = new ObjectDimensionCalculator(
                baselineDepth, currentDepth,
                config.fovHorizontal, config.fovVertical,
                thresholdManager.getCurrentThreshold(), noiseMask
        );

        // 计算尺寸
        ObjectDimensionCalculator.DimensionResult result =
                calculator.calculateDimensionsByCalibratedRatioWithMedianData(pixelSizeX, pixelSizeY);

        MeasureResult measureResult = convertToMeasureResult(result);

        if (result == null || result.validPixelCount == 0) {
            // 未检测到物体
            notifyMeasureError(MeasureError.of(MeasureError.ErrorCode.OBJECT_NOT_DETECTED));
            return;
        }

        // 更新动态阈值
        int xSpan = result.xMax - result.xMin + 1;
        int ySpan = result.yMax - result.yMin + 1;
        DynamicThresholdManager.ThresholdUpdateResult thresholdResult =
                thresholdManager.update(xSpan, ySpan);

        // 通知状态变化
        if (continuousMeasureCallback != null) {
            MeasureStatus status = new MeasureStatus(
                    thresholdResult.currentThreshold,
                    thresholdResult.thresholdLocked,
                    thresholdResult.unstableCount,
                    thresholdResult.stableCount
            );
            continuousMeasureCallback.onStatusChanged(status);
        }

        // 检测稳定性
        boolean stable = measureStabilityDetector.addAndCheck(
                result.width, result.length, result.height);

        if (measureCallback != null) {
            // 单次测量模式
            measureCallback.onProgress(
                    measureStabilityDetector.getStableCount(),
                    config.stableCountMeasure,
                    measureResult
            );

            if (measureStabilityDetector.isStableEnough()) {
                // 测量完成
                stopWindowSampling();
                state = State.CALIBRATED;

                MeasureResult finalResult = new MeasureResult(
                        measureStabilityDetector.getAverageWidth(),
                        measureStabilityDetector.getAverageLength(),
                        measureStabilityDetector.getAverageHeight(),
                        result.rawPixelCount, result.validPixelCount,
                        result.maxDepthDiff, result.objectSurfaceDepth,
                        result.xMin, result.xMax, result.yMin, result.yMax,
                        true, config.stableCountMeasure,
                        "测量完成"
                );

                Log.d(TAG, "单次测量完成: " + finalResult.toString());
                measureCallback.onSuccess(finalResult);
                measureCallback = null;
            }
        } else if (continuousMeasureCallback != null) {
            // 连续测量模式
            measureResult = new MeasureResult(
                    result.width, result.length, result.height,
                    result.rawPixelCount, result.validPixelCount,
                    result.maxDepthDiff, result.objectSurfaceDepth,
                    result.xMin, result.xMax, result.yMin, result.yMax,
                    stable, measureStabilityDetector.getStableCount(),
                    stable ? "稳定" : "不稳定"
            );

            continuousMeasureCallback.onResult(measureResult);

            if (stable && measureStabilityDetector.isStableEnough()) {
                // 输出平均结果
                MeasureResult avgResult = new MeasureResult(
                        measureStabilityDetector.getAverageWidth(),
                        measureStabilityDetector.getAverageLength(),
                        measureStabilityDetector.getAverageHeight(),
                        result.rawPixelCount, result.validPixelCount,
                        result.maxDepthDiff, result.objectSurfaceDepth,
                        result.xMin, result.xMax, result.yMin, result.yMax,
                        true, measureStabilityDetector.getStableCount(),
                        "测量稳定"
                );
                Log.d(TAG, "连续测量稳定: " + avgResult.toString());
            }
        }
    }

    private MeasureResult convertToMeasureResult(ObjectDimensionCalculator.DimensionResult r) {
        if (r == null) {
            return MeasureResult.empty();
        }
        return new MeasureResult(
                r.width, r.length, r.height,
                r.rawPixelCount, r.validPixelCount,
                r.maxDepthDiff, r.objectSurfaceDepth,
                r.xMin, r.xMax, r.yMin, r.yMax,
                false, 0, r.message
        );
    }

    private void notifyMeasureError(MeasureError error) {
        if (measureCallback != null) {
            measureCallback.onError(error);
        } else if (continuousMeasureCallback != null) {
            // 连续测量模式下，OBJECT_NOT_DETECTED 不算错误
            if (error.code != MeasureError.ErrorCode.OBJECT_NOT_DETECTED) {
                continuousMeasureCallback.onError(error);
            }
        }
    }

    // ==================== 窗口采样 ====================

    private void startWindowSampling(Runnable sampleAction) {
        windowFrames.clear();
        windowMaxDepthDiffs.clear();
        windowFrameCount = 0;

        samplingRunnable = new Runnable() {
            @Override
            public void run() {
                if (!running || samplingRunnable != this) return;

                if (latestDepthData == null) {
                    samplingHandler.postDelayed(this, config.sampleFastIntervalMs);
                    return;
                }

                // 采集一帧
                float[][] frame = convertToDistanceMatrix(latestDepthData);
                windowFrames.add(frame);
                windowFrameCount++;

                // 测量模式下计算 maxDepthDiff
                if (state == State.MEASURING && baselineDepth != null) {
                    float maxDiff = calculateMaxDepthDiff(frame);
                    windowMaxDepthDiffs.add(maxDiff);
                }

                if (windowFrameCount >= config.samplesPerWindow) {
                    // 窗口采集完成，处理中值
                    if (state == State.MEASURING && !windowMaxDepthDiffs.isEmpty()) {
                        // 测量模式：选 maxDepthDiff 中位数对应的帧
                        latestMedianDepth = MedianFilter.selectMedianMaxDiffFrame(
                                windowFrames, windowMaxDepthDiffs);
                    } else {
                        // 校准模式：逐像素中值
                        latestMedianDepth = MedianFilter.medianFrames(windowFrames);
                    }

                    windowFrames.clear();
                    windowMaxDepthDiffs.clear();
                    windowFrameCount = 0;

                    // 执行采样回调
                    if (sampleAction != null) {
                        sampleAction.run();
                    }

                    // 等待下一个窗口
                    samplingHandler.postDelayed(this, config.sampleIntervalMs);
                } else {
                    // 继续窗口内采样
                    samplingHandler.postDelayed(this, config.sampleFastIntervalMs);
                }
            }
        };

        samplingHandler.post(samplingRunnable);
    }

    private void stopWindowSampling() {
        if (samplingRunnable != null) {
            samplingHandler.removeCallbacks(samplingRunnable);
            samplingRunnable = null;
        }
        windowFrames.clear();
        windowMaxDepthDiffs.clear();
        windowFrameCount = 0;
    }

    private float calculateMaxDepthDiff(float[][] frame) {
        float maxDiff = 0;
        for (int x = 0; x < 100; x++) {
            for (int y = 0; y < 100; y++) {
                float bDepth = baselineDepth[x][y];
                float cDepth = frame[x][y];
                if (bDepth > 100 && bDepth < 800 && cDepth > 100 && cDepth < 800) {
                    float diff = bDepth - cDepth;
                    if (diff > maxDiff) {
                        maxDiff = diff;
                    }
                }
            }
        }
        return maxDiff;
    }

    // ==================== 工具方法 ====================

    /**
     * 获取当前状态
     */
    public State getState() {
        return state;
    }

    /**
     * 设置噪声掩码
     */
    public void setNoiseMask(boolean[][] mask) {
        this.noiseMask = mask;
    }

    /**
     * 清除噪声掩码
     */
    public void clearNoiseMask() {
        this.noiseMask = null;
    }

    private float[][] convertToDistanceMatrix(byte[] depthData) {
        float[][] matrix = new float[100][100];
        for (int x = 0; x < 100; x++) {
            for (int y = 0; y < 100; y++) {
                int pixelValue = depthData[x * 100 + y] & 0xFF;
                matrix[x][y] = calculateDistanceMm(pixelValue);
            }
        }
        return matrix;
    }

    private int calculateDistanceMm(int pixelValue) {
        if (pixelValue == 0) return 0;
        double distance = Math.pow(pixelValue / 5.1, 2);
        return (int) Math.round(distance);
    }

    private float[][] copyDepthArray(float[][] source) {
        float[][] copy = new float[source.length][];
        for (int i = 0; i < source.length; i++) {
            copy[i] = source[i].clone();
        }
        return copy;
    }

    private void notifyError(MeasureError error) {
        mainHandler.post(() -> {
            if (calibrationCallback != null) {
                calibrationCallback.onError(error);
            } else if (measureCallback != null) {
                measureCallback.onError(error);
            } else if (continuousMeasureCallback != null) {
                continuousMeasureCallback.onError(error);
            }
        });
    }
}
