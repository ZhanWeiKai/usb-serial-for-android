package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.graphics.Color;
import android.graphics.Typeface;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.ByteArrayOutputStream;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * 物体尺寸测量 Activity
 *
 * 两阶段测量流程:
 * 1. 基线校准: 扫描空桌面， * 2. 物体测量: 计算长宽高
 */
public class ObjectMeasurementActivity extends Activity {

    private static final String TAG = "ObjectMeasurement";

    // 帧格式常量
    private static final int HEADER_SIZE = 2;
    private static final int LENGTH_SIZE = 2;
    private static final int META_SIZE = 16;
    private static final int CHECKSUM_TAIL = 2;
    private static final int HEAD_1 = 0x00;
    private static final int HEAD_2 = 0xFF;

    // 采样参数
    private static final long SAMPLE_INTERVAL_MS = 500;  // 0.5秒采样间隔
    private static final int STABLE_COUNT_REQUIRED = 10; // 连续10次稳定 = 5秒

    // 稳定性阈值
    private static final float BASELINE_AVG_THRESHOLD_MM = 60.0f;  // 基线平均值差异阈值 (mm)
    private static final float POSITION_CHANGE_THRESHOLD_MM = 100.0f;  // 位置变化阈值 (mm)
    private static final float VOLUME_THRESHOLD_CM3 = 50.0f;       // 体积差异阈值 (cm³)

    // ObjectDimensionCalculator 参数
    private static final float PIXEL_SIZE_X_MM = 5.0f;  // 待校准
    private static final float PIXEL_SIZE_Y_MM = 5.0f;  // 待校准
    private static final float OBJECT_THRESHOLD_MM = 20.0f;  // 物体检测阈值

    // 串口
    private UsbSerialPort usbSerialPort;
    private final ByteArrayOutputStream buffer = new ByteArrayOutputStream();
    private final Object bufferLock = new Object();
    private volatile boolean running = true;

    // 数据存储
    private DepthDataStorage depthStorage;

    // 测量状态
    private enum MeasurementPhase {
        IDLE,           // 空闲
        CALIBRATING,    // 基线校准中
        CALIBRATED,     // 校准完成
        MEASURING,      // 测量中
        MEASURED         // 测量完成
    }
    private MeasurementPhase currentPhase = MeasurementPhase.IDLE;

    // 基线数据
    private float[][] baselineDepth;  // [100][100] mm
    private float[][] savedBaseline;  // 已保存的基线（用于位置校验，不会被更新）

    // 测量数据
    private float[][] currentDepth;   // [100][100] mm
    private ObjectDimensionCalculator.DimensionResult lastResult;
    private float lastVolume = 0;
    private int stableCount = 0;

    // UI
    private TextView statusText;
    private TextView frameStatsText;  // 帧校验统计
    private TextView infoText;
    private TextView logView;
    private ScrollView scrollView;
    private Button btnCalibrate;
    private Button btnMeasure;
    private Button btnReset;

    // 定时器
    private Handler mainHandler;
    private Runnable sampleRunnable;
    private byte[] latestDepthData;  // 最新一帧的深度数据

    // 帧校验统计
    private int totalFrameCount = 0;
    private int validFrameCount = 0;
    private int invalidFrameCount = 0;
    private boolean latestFrameValid = false;

    private final SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault());

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        mainHandler = new Handler(Looper.getMainLooper());
        depthStorage = new DepthDataStorage(this);
        createLayout();
        connectUsb();
    }

    private void createLayout() {
        LinearLayout rootLayout = new LinearLayout(this);
        rootLayout.setOrientation(LinearLayout.VERTICAL);
        rootLayout.setBackgroundColor(Color.parseColor("#1E1E1E"));
        rootLayout.setPadding(16, 16, 16, 16);

        // 状态区域
        statusText = new TextView(this);
        statusText.setTextColor(Color.parseColor("#00FF00"));
        statusText.setTextSize(16);
        statusText.setTypeface(Typeface.MONOSPACE);
        statusText.setPadding(0, 0, 0, 8);
        statusText.setText("状态: 等待连接设备...");

        // 帧校验统计区域
        frameStatsText = new TextView(this);
        frameStatsText.setTextColor(Color.parseColor("#888888"));
        frameStatsText.setTextSize(11);
        frameStatsText.setTypeface(Typeface.MONOSPACE);
        frameStatsText.setPadding(0, 0, 0, 16);
        frameStatsText.setText("帧统计: --");

        // 信息显示区域
        infoText = new TextView(this);
        infoText.setTextColor(Color.parseColor("#00FF00"));
        infoText.setBackgroundColor(Color.parseColor("#2D2D2D"));
        infoText.setTextSize(12);
        infoText.setTypeface(Typeface.MONOSPACE);
        infoText.setPadding(16, 16, 16, 16);
        LinearLayout.LayoutParams infoParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1.0f
        );
        infoText.setLayoutParams(infoParams);
        infoText.setText("等待连接...");

        // 日志区域
        logView = new TextView(this);
        logView.setTextColor(Color.parseColor("#AAAAAA"));
        logView.setTextSize(10);
        logView.setTypeface(Typeface.MONOSPACE);
        logView.setPadding(8, 8, 8, 8);

        scrollView = new ScrollView(this);
        LinearLayout.LayoutParams scrollParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                0.8f
        );
        scrollView.setLayoutParams(scrollParams);
        scrollView.setBackgroundColor(Color.parseColor("#1E1E1E"));
        scrollView.addView(logView);

        // 按钮区域
        LinearLayout buttonLayout = new LinearLayout(this);
        buttonLayout.setOrientation(LinearLayout.HORIZONTAL);
        buttonLayout.setPadding(0, 16, 0, 0);

        btnCalibrate = new Button(this);
        btnCalibrate.setText("开始校准");
        btnCalibrate.setOnClickListener(v -> startCalibration());

        btnMeasure = new Button(this);
        btnMeasure.setText("开始测量");
        btnMeasure.setEnabled(false);
        btnMeasure.setOnClickListener(v -> startMeasurement());

        btnReset = new Button(this);
        btnReset.setText("重置");
        btnReset.setOnClickListener(v -> resetMeasurement());

        LinearLayout.LayoutParams btnParams = new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1.0f
        );
        btnCalibrate.setLayoutParams(btnParams);
        btnMeasure.setLayoutParams(btnParams);
        btnReset.setLayoutParams(btnParams);

        buttonLayout.addView(btnCalibrate);
        buttonLayout.addView(btnMeasure);
        buttonLayout.addView(btnReset);

        rootLayout.addView(statusText);
        rootLayout.addView(frameStatsText);
        rootLayout.addView(infoText);
        rootLayout.addView(scrollView);
        rootLayout.addView(buttonLayout);
        setContentView(rootLayout);
    }

    private void connectUsb() {
        new Thread(() -> {
            try {
                UsbManager usbManager = (UsbManager) getSystemService(USB_SERVICE);
                List<UsbSerialPort> ports = UsbSerialProber.getDefaultProber()
                        .findAllDrivers(usbManager)
                        .stream()
                        .flatMap(driver -> driver.getPorts().stream())
                        .toList();

                if (ports.isEmpty()) {
                    runOnUiThread(() -> {
                        statusText.setText("状态: 未找到USB设备");
                        appendLog("错误: 未找到USB串口设备");
                    });
                    return;
                }

                usbSerialPort = ports.get(0);
                UsbDeviceConnection connection = usbManager.openDevice(usbSerialPort.getDriver().getDevice());
                if (connection == null) {
                    runOnUiThread(() -> {
                        statusText.setText("状态: 无法打开USB设备");
                        appendLog("错误: 无法打开USB设备 (权限问题?)");
                    });
                    return;
                }

                usbSerialPort.open(connection);
                usbSerialPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                Log.i(TAG, "串口已打开，波特率: 115200");

                Thread.sleep(500);
                clearBuffer();

                // 配置设备
                sendAtCommand("AT+ISP=0\r\n");
                Thread.sleep(500);
                sendAtCommand("AT+BINN=1\r\n");
                Thread.sleep(200);
                sendAtCommand("AT+UNIT=0\r\n");
                Thread.sleep(200);
                sendAtCommand("AT+FPS=15\r\n");
                Thread.sleep(200);
                sendAtCommand("AT+DISP=6\r\n");
                Thread.sleep(200);
                sendAtCommand("AT+SAVE\r\n");
                Thread.sleep(500);
                sendAtCommand("AT+ISP=1\r\n");
                Thread.sleep(2000);

                runOnUiThread(() -> {
                    statusText.setText("状态: 设备已连接");
                    appendLog("设备连接成功");
                    // 检查是否有已保存的基线
                    if (depthStorage.hasBaseline()) {
                        appendLog("发现已保存的基线数据");
                    }
                });

                // 启动读取线程
                new Thread(this::readLoop).start();

            } catch (Exception e) {
                Log.e(TAG, "USB connection failed", e);
                runOnUiThread(() -> {
                    statusText.setText("状态: 连接失败 - " + e.getMessage());
                    appendLog("连接失败: " + e.getMessage());
                });
            }
        }).start();
    }

    private void sendAtCommand(String cmd) {
        try {
            usbSerialPort.write(cmd.getBytes(), 500);
            Thread.sleep(100);
            byte[] resp = new byte[256];
            int len = usbSerialPort.read(resp, 500);
            if (len > 0) {
                String response = new String(resp, 0, len).trim();
                Log.d(TAG, "AT: " + cmd.trim() + " -> " + response);
            }
        } catch (Exception e) {
            Log.e(TAG, "AT command failed: " + cmd.trim(), e);
        }
    }

    private void clearBuffer() {
        try {
            byte[] tmp = new byte[4096];
            while (usbSerialPort.read(tmp, 50) > 0) {
            }
        } catch (Exception ignored) {}
    }

    private void readLoop() {
        byte[] readBuf = new byte[8192];
        Log.i(TAG, "readLoop started");

        while (running) {
            try {
                int len = usbSerialPort.read(readBuf, 100);
                if (len > 0) {
                    synchronized (bufferLock) {
                        buffer.write(readBuf, 0, len);
                    }
                    parseFrames();
                }
            } catch (Exception e) {
                Log.e(TAG, "Read error", e);
            }
        }
        Log.i(TAG, "readLoop ended");
    }

    private void parseFrames() {
        synchronized (bufferLock) {
            byte[] data = buffer.toByteArray();
            int pos = 0;
            int minHeaderSize = HEADER_SIZE + LENGTH_SIZE;

            while (pos + minHeaderSize <= data.length) {
                // 查找帧头 0x00 0xFF
                if ((data[pos] & 0xFF) != HEAD_1 || (data[pos + 1] & 0xFF) != HEAD_2) {
                    pos++;
                    continue;
                }

                int payloadLen = (data[pos + 2] & 0xFF) | ((data[pos + 3] & 0xFF) << 8);
                int frameSize = HEADER_SIZE + LENGTH_SIZE + payloadLen + CHECKSUM_TAIL;

                if (payloadLen < 10000 || payloadLen > 10050) {
                    pos++;
                    continue;
                }

                if (pos + frameSize > data.length) {
                    break;
                }

                totalFrameCount++;

                // 验证包尾 0xDD
                int tailPos = pos + frameSize - 1;
                int tailByte = data[tailPos] & 0xFF;
                boolean frameValid = (tailByte == 0xDD);

                if (!frameValid) {
                    invalidFrameCount++;
                    latestFrameValid = false;
                    final int failFrameNum = totalFrameCount;
                    final int failTail = tailByte;
                    runOnUiThread(() -> {
                        appendLog(String.format("❌ Frame #%d 尾部验证失败: 0x%02X (期望 0xDD)", failFrameNum, failTail));
                        updateFrameStats();
                    });
                    pos++;
                    continue;
                }

                validFrameCount++;
                latestFrameValid = true;

                // 更新帧统计显示
                updateFrameStats();

                // 提取深度数据
                int pixelStart = pos + HEADER_SIZE + LENGTH_SIZE + META_SIZE;
                byte[] depthData = Arrays.copyOfRange(data, pixelStart, pixelStart + 100 * 100);

                // 保存最新深度数据
                latestDepthData = depthData;

                // 移除已处理的帧
                int newStart = pos + frameSize;
                if (newStart < data.length) {
                    byte[] remaining = Arrays.copyOfRange(data, newStart, data.length);
                    buffer.reset();
                    try {
                        buffer.write(remaining);
                    } catch (Exception ignored) {}
                    data = buffer.toByteArray();
                } else {
                    buffer.reset();
                    data = new byte[0];
                }
                pos = 0;
            }
        }
    }

    // ==================== 阶段1: 基线校准 ====================

    private void startCalibration() {
        // 允许从 IDLE, MEASURED, 或 CALIBRATED（自动重新校准）状态开始
        if (currentPhase == MeasurementPhase.CALIBRATING || currentPhase == MeasurementPhase.MEASURING) {
            return;
        }

        currentPhase = MeasurementPhase.CALIBRATING;
        stableCount = 0;
        baselineDepth = null;
        savedBaseline = null;  // 清除已保存的基线

        runOnUiThread(() -> {
            btnCalibrate.setEnabled(false);
            btnMeasure.setEnabled(false);
            statusText.setText("状态: 基线校准中...");
            infoText.setText("正在采集空桌面数据...\n请保持桌面清洁\n\n稳定进度: 0/10");
            appendLog("开始基线校准");
        });

        // 启动定时采样
        startSampling(this::processCalibrationSample);
    }

    private void processCalibrationSample() {
        if (latestDepthData == null) {
            return;
        }

        // 将原始数据转换为距离矩阵
        float[][] currentFrame = convertToDistanceMatrix(latestDepthData);

        if (baselineDepth == null) {
            // 第一帧，直接保存
            baselineDepth = currentFrame;
            stableCount = 1;
            updateCalibrationUI(1, 0);
        } else {
            // 计算与上一帧的平均值差异
            float avgDiff = calculateAverageDifference(baselineDepth, currentFrame);

            if (avgDiff < BASELINE_AVG_THRESHOLD_MM) {
                // 稳定
                stableCount++;
                updateCalibrationUI(stableCount, avgDiff);

                if (stableCount >= STABLE_COUNT_REQUIRED) {
                    // 校准完成
                    finishCalibration();
                }
            } else {
                // 不稳定，重置计数，更新基线
                baselineDepth = currentFrame;
                stableCount = 1;
                updateCalibrationUI(1, avgDiff);
            }
        }
    }

    private void updateCalibrationUI(int count, float diff) {
        runOnUiThread(() -> {
            StringBuilder sb = new StringBuilder();
            sb.append("正在采集空桌面数据...\n");
            sb.append("请保持桌面清洁\n\n");
            sb.append(String.format("稳定进度: %d/%d\n", count, STABLE_COUNT_REQUIRED));
            sb.append(String.format("帧间差异: %.2f mm (阈值: %.1f mm)\n", diff, BASELINE_AVG_THRESHOLD_MM));
            sb.append("\n当前深度统计:\n");
            sb.append(getDepthStatistics(baselineDepth));
            infoText.setText(sb.toString());
        });
    }

    private void finishCalibration() {
        // 保存已校准的基线（深拷贝，用于位置校验）
        savedBaseline = copyDepthArray(baselineDepth);

        // 计算中心区域(10x10)平均深度
        float centerDepth = calculateCenterDepth(baselineDepth);

        // 保存基线到文件
        boolean saved = depthStorage.saveBaseline(baselineDepth);

        runOnUiThread(() -> {
            currentPhase = MeasurementPhase.CALIBRATED;
            btnMeasure.setEnabled(true);
            btnCalibrate.setEnabled(true);

            statusText.setText("状态: 基线校准完成 ✓ (监控中)");
            StringBuilder sb = new StringBuilder();
            sb.append("══════ 基线校准完成 ══════\n\n");
            sb.append(String.format("中心点深度: %.0f mm\n\n", centerDepth));
            sb.append("基线已保存到本地\n\n");
            sb.append("基线深度统计:\n");
            sb.append(getDepthStatistics(baselineDepth));
            sb.append("\n\n正在监控位置变化...");
            infoText.setText(sb.toString());
            appendLog(String.format("基线校准完成，中心点深度: %.0f mm，开始监控位置变化", centerDepth));
        });

        // 不停止采样，切换到位置监控模式
        stopSampling();
        startSampling(this::monitorPosition);
    }

    /**
     * 深拷贝深度数组
     */
    private float[][] copyDepthArray(float[][] source) {
        if (source == null) return null;
        float[][] copy = new float[source.length][];
        for (int i = 0; i < source.length; i++) {
            copy[i] = source[i].clone();
        }
        return copy;
    }

    /**
     * 监控位置变化
     */
    private void monitorPosition() {
        if (latestDepthData == null || savedBaseline == null) {
            return;
        }

        // 将原始数据转换为距离矩阵
        float[][] currentFrame = convertToDistanceMatrix(latestDepthData);

        // 计算与已保存基线的差异
        float avgDiff = calculateAverageDifference(savedBaseline, currentFrame);

        if (avgDiff >= POSITION_CHANGE_THRESHOLD_MM) {
            // 位置变化超过阈值，自动重新校准
            runOnUiThread(() -> {
                appendLog(String.format("⚠ 检测到位置变化: %.0f mm >= %.0f mm，开始重新校准", avgDiff, POSITION_CHANGE_THRESHOLD_MM));
                statusText.setText("状态: 检测到位置变化，重新校准中...");
            });

            // 停止监控，开始重新校准
            stopSampling();
            startCalibration();
        }
    }

    /**
     * 计算中心10x10区域的平均深度
     */
    private float calculateCenterDepth(float[][] depth) {
        if (depth == null) return 0;

        float sum = 0;
        int count = 0;
        // 中心10x10区域: [45:55, 45:55]
        for (int x = 45; x < 55; x++) {
            for (int y = 45; y < 55; y++) {
                float v = depth[x][y];
                if (v > 0) {
                    sum += v;
                    count++;
                }
            }
        }
        return count > 0 ? sum / count : 0;
    }

    // ==================== 阶段2: 物体测量 ====================

    private void startMeasurement() {
        if (currentPhase != MeasurementPhase.CALIBRATED && currentPhase != MeasurementPhase.MEASURED) {
            return;
        }

        // 停止位置监控
        stopSampling();

        // 加载基线
        if (baselineDepth == null) {
            baselineDepth = depthStorage.loadBaseline();
            if (baselineDepth == null) {
                appendLog("错误: 无法加载基线数据，请重新校准");
                return;
            }
        }

        currentPhase = MeasurementPhase.MEASURING;
        stableCount = 0;
        lastVolume = 0;
        lastResult = null;

        runOnUiThread(() -> {
            btnCalibrate.setEnabled(false);
            btnMeasure.setEnabled(false);
            statusText.setText("状态: 物体测量中...");
            infoText.setText(String.format("正在测量物体尺寸...\n\n请将物体放置在桌面上\n\n稳定进度: 0/%d", STABLE_COUNT_REQUIRED));
            appendLog("开始物体测量");
        });

        // 启动定时采样
        startSampling(this::processMeasurementSample);
    }

    private void processMeasurementSample() {
        if (latestDepthData == null || baselineDepth == null) {
            return;
        }

        // 将原始数据转换为距离矩阵
        currentDepth = convertToDistanceMatrix(latestDepthData);

        // 使用 ObjectDimensionCalculator 计算
        ObjectDimensionCalculator calculator = new ObjectDimensionCalculator(
                baselineDepth, currentDepth,
                PIXEL_SIZE_X_MM, PIXEL_SIZE_Y_MM, OBJECT_THRESHOLD_MM
        );

        ObjectDimensionCalculator.DimensionResult result = calculator.calculate();

        if (result == null || "未检测到物体".equals(result.message)) {
            runOnUiThread(() -> {
                infoText.setText("未检测到物体\n\n请将物体放置在桌面上");
            });
            stableCount = 0;
            return;
        }

        // 计算体积 (mm³ -> cm³)
        float volumeMm3 = result.width * result.length * result.height;
        float volumeCm3 = volumeMm3 / 1000.0f;  // mm³ -> cm³

        // 检查体积稳定性
        float volumeDiff = Math.abs(volumeCm3 - lastVolume);

        if (volumeDiff < VOLUME_THRESHOLD_CM3) {
            stableCount++;
            lastVolume = volumeCm3;  // 更新上一帧体积，用于下次比较
            lastResult = result;

            runOnUiThread(() -> {
                updateMeasurementUI(result, volumeCm3, volumeDiff, stableCount);
            });

            if (stableCount >= STABLE_COUNT_REQUIRED) {
                finishMeasurement(result, volumeCm3);
            }
        } else {
            stableCount = 1;
            lastVolume = volumeCm3;
            lastResult = result;

            runOnUiThread(() -> {
                updateMeasurementUI(result, volumeCm3, volumeDiff, 1);
            });
        }
    }

    private void updateMeasurementUI(ObjectDimensionCalculator.DimensionResult result,
                                     float volumeCm3, float volumeDiff, int count) {
        StringBuilder sb = new StringBuilder();
        sb.append("正在测量物体尺寸...\n\n");
        sb.append(String.format("稳定进度: %d/%d\n", count, STABLE_COUNT_REQUIRED));
        sb.append(String.format("体积变化: %.1f cm³ (阈值: %.1f cm³)\n\n", volumeDiff, VOLUME_THRESHOLD_CM3));
        sb.append("══════ 当前测量值 ══════\n");
        sb.append(String.format("宽(W): %.1f mm  (%.1f cm)\n", result.width, result.width / 10));
        sb.append(String.format("长(L): %.1f mm  (%.1f cm)\n", result.length, result.length / 10));
        sb.append(String.format("高(H): %.1f mm  (%.1f cm)\n", result.height, result.height / 10));
        sb.append(String.format("厚(T): %.1f mm  (%.1f cm)\n", result.thickness, result.thickness / 10));
        sb.append(String.format("体积: %.1f cm³\n", volumeCm3));
        infoText.setText(sb.toString());
    }

    private void finishMeasurement(ObjectDimensionCalculator.DimensionResult result, float volumeCm3) {
        stopSampling();

        runOnUiThread(() -> {
            currentPhase = MeasurementPhase.MEASURED;
            btnCalibrate.setEnabled(true);
            btnMeasure.setEnabled(true);

            statusText.setText("状态: 测量完成 ✓");
            StringBuilder sb = new StringBuilder();
            sb.append("══════ 最终测量结果 ══════\n\n");
            sb.append(String.format("宽(W): %.1f mm  (%.1f cm)\n", result.width, result.width / 10));
            sb.append(String.format("长(L): %.1f mm  (%.1f cm)\n", result.length, result.length / 10));
            sb.append(String.format("高(H): %.1f mm  (%.1f cm)\n", result.height, result.height / 10));
            sb.append(String.format("厚(T): %.1f mm  (%.1f cm)\n", result.thickness, result.thickness / 10));
            sb.append("─────────────────────────\n");
            sb.append(String.format("体积: %.1f cm³\n", volumeCm3));
            sb.append("═════════════════════════\n\n");
            sb.append("点击 [重置] 进行新测量");
            infoText.setText(sb.toString());
            appendLog(String.format("测量完成 - W:%.1fmm L:%.1fmm H:%.1fmm V:%.1fcm³",
                    result.width, result.length, result.height, volumeCm3));
        });
    }

    // ==================== 辅助方法 ====================

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

    private float calculateAverageDifference(float[][] frame1, float[][] frame2) {
        float totalDiff = 0;
        int count = 0;

        for (int x = 0; x < frame1.length; x++) {
            for (int y = 0; y < frame1[x].length; y++) {
                float v1 = frame1[x][y];
                float v2 = frame2[x][y];
                if (v1 > 0 && v2 > 0) {  // 忽略无效值
                    totalDiff += Math.abs(v1 - v2);
                    count++;
                }
            }
        }

        return count > 0 ? totalDiff / count : 0;
    }

    private String getDepthStatistics(float[][] depth) {
        float min = Float.MAX_VALUE;
        float max = 0;
        float sum = 0;
        int count = 0;

        for (int x = 0; x < depth.length; x++) {
            for (int y = 0; y < depth[x].length; y++) {
                float v = depth[x][y];
                if (v > 0) {
                    if (v < min) min = v;
                    if (v > max) max = v;
                    sum += v;
                    count++;
                }
            }
        }

        if (count == 0) return "无有效数据";

        float avg = sum / count;
        return String.format("最小: %.0f mm  最大: %.0f mm  平均: %.0f mm",
                min == Float.MAX_VALUE ? 0 : min, max, avg);
    }

    private void startSampling(Runnable sampleAction) {
        sampleRunnable = new Runnable() {
            @Override
            public void run() {
                if (running) {
                    sampleAction.run();
                    mainHandler.postDelayed(this, SAMPLE_INTERVAL_MS);
                }
            }
        };
        mainHandler.post(sampleRunnable);
    }

    private void stopSampling() {
        if (sampleRunnable != null) {
            mainHandler.removeCallbacks(sampleRunnable);
            sampleRunnable = null;
        }
    }

    private void resetMeasurement() {
        stopSampling();
        currentPhase = MeasurementPhase.IDLE;
        stableCount = 0;
        lastVolume = 0;
        lastResult = null;
        currentDepth = null;
        baselineDepth = null;
        savedBaseline = null;  // 清除已保存的基线

        // 删除保存的基线文件
        boolean deleted = depthStorage.deleteBaseline();

        runOnUiThread(() -> {
            btnCalibrate.setEnabled(true);
            btnMeasure.setEnabled(false);
            statusText.setText("状态: 已重置");
            if (deleted) {
                infoText.setText("已重置，基线数据已清除\n\n点击 [开始校准] 开始新的测量");
                appendLog("已重置测量状态，基线文件已删除");
            } else {
                infoText.setText("已重置\n\n点击 [开始校准] 开始新的测量");
                appendLog("已重置测量状态");
            }
        });
    }

    private void updateFrameStats() {
        runOnUiThread(() -> {
            String statusIcon = (invalidFrameCount == 0) ? "✓" : "⚠";
            String color = (invalidFrameCount == 0) ? "#00FF00" : "#FFAA00";
            frameStatsText.setTextColor(Color.parseColor(color));

            StringBuilder sb = new StringBuilder();
            sb.append(String.format("帧统计: %s 总计:%d 有效:%d 无效:%d",
                    statusIcon, totalFrameCount, validFrameCount, invalidFrameCount));

            if (invalidFrameCount > 0) {
                float validRate = (totalFrameCount > 0) ? (validFrameCount * 100.0f / totalFrameCount) : 0;
                sb.append(String.format(" 有效率:%.1f%%", validRate));
            }

            frameStatsText.setText(sb.toString());
        });
    }

    private void appendLog(String line) {
        runOnUiThread(() -> {
            logView.append(timeFormat.format(new Date()) + " " + line + "\n");
            scrollView.post(() -> scrollView.fullScroll(ScrollView.FOCUS_DOWN));

            // 限制日志行数
            String currentLog = logView.getText().toString();
            String[] lines = currentLog.split("\n");
            if (lines.length > 100) {
                StringBuilder newLog = new StringBuilder();
                for (int i = lines.length - 100; i < lines.length; i++) {
                    newLog.append(lines[i]).append("\n");
                }
                logView.setText(newLog.toString());
            }
        });
    }

    @Override
    protected void onDestroy() {
        running = false;
        stopSampling();
        try {
            if (usbSerialPort != null) {
                usbSerialPort.close();
            }
        } catch (Exception ignored) {}
        super.onDestroy();
    }
}
