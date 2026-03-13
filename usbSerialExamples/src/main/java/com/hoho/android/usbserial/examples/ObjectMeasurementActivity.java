package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.graphics.Bitmap;
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
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.ByteArrayOutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
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
    private static final int STABLE_COUNT_CALIBRATE = 10; // 校准: 连续10次稳定 = 5秒
    private static final int STABLE_COUNT_MEASURE = 10;   // 测量: 连续10次稳定 = 5秒

    // 稳定性阈值
    private static final float BASELINE_AVG_THRESHOLD_MM = 20.0f;  // 基线平均值差异阈值 (mm)
    private static final float POSITION_CHANGE_THRESHOLD_MM = 100.0f;  // 位置变化阈值 (mm)
    private static final float DIMENSION_THRESHOLD_MM = 20.0f;     // 长宽高各维度偏差阈值 (mm)

    // FOV 参数 (A010: 70°×60°)
    private static final double FOV_HORIZONTAL_DEG = 70.0;  // 水平视场角
    private static final double FOV_VERTICAL_DEG = 60.0;    // 垂直视场角
    private static final double TAN_FOV_H_HALF = Math.tan(Math.toRadians(FOV_HORIZONTAL_DEG / 2));  // tan(35°)
    private static final double TAN_FOV_V_HALF = Math.tan(Math.toRadians(FOV_VERTICAL_DEG / 2));    // tan(30°)

    // 动态计算的像素尺寸 (根据校准时的深度)
    private float pixelSizeX;  // 水平方向每像素对应的 mm
    private float pixelSizeY;  // 垂直方向每像素对应的 mm

    private static final float OBJECT_THRESHOLD_MM = 50.0f;  // 物体检测阈值 (考虑噪声余量)

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
    private List<float[][]> calibrationFrames;  // 校准期间累积的稳定帧，用于多帧平均

    // 测量数据
    private float[][] currentDepth;   // [100][100] mm
    private boolean[][] noiseMask;    // 噪声像素黑名单
    private ObjectDimensionCalculator.DimensionResult lastResult;
    private float lastWidth = 0, lastLength = 0, lastHeight = 0;
    private int stableCount = 0;

    // 稳定帧累积 (用于取平均值)
    private List<Float> stableWidths = new ArrayList<>();
    private List<Float> stableLengths = new ArrayList<>();
    private List<Float> stableHeights = new ArrayList<>();

    // UI
    private TextView statusText;
    private TextView frameStatsText;  // 帧校验统计
    private TextView infoText;
    private TextView logView;
    private ScrollView scrollView;
    private Button btnCalibrate;
    private Button btnMeasure;
    private Button btnReset;

    // 差异热力图
    private ImageView imgDiffHeatmap;
    private Bitmap diffBitmap;
    private int[] diffPixels;  // 长度 100*100
    private TextView peakDepthText;      // 显示最高点 baseline/current/diff

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
        // 根布局：水平，左边内容，右边热力图
        LinearLayout rootLayout = new LinearLayout(this);
        rootLayout.setOrientation(LinearLayout.HORIZONTAL);
        rootLayout.setBackgroundColor(Color.parseColor("#1E1E1E"));
        rootLayout.setPadding(16, 16, 16, 16);

        // ===== 左侧面板：原来的内容 =====
        LinearLayout leftPanel = new LinearLayout(this);
        leftPanel.setOrientation(LinearLayout.VERTICAL);
        LinearLayout.LayoutParams leftParams = new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.MATCH_PARENT,
                1.0f
        );
        leftPanel.setLayoutParams(leftParams);

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
        btnCalibrate.setOnClickListener(v -> {
            if (currentPhase == MeasurementPhase.MEASURING) {
                clearCurrentMask();
            } else {
                startCalibration();
            }
        });

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

        leftPanel.addView(statusText);
        leftPanel.addView(frameStatsText);
        leftPanel.addView(infoText);
        leftPanel.addView(scrollView);
        leftPanel.addView(buttonLayout);

        // ===== 右侧面板：热力图 =====
        LinearLayout rightPanel = new LinearLayout(this);
        rightPanel.setOrientation(LinearLayout.VERTICAL);
        rightPanel.setPadding(16, 0, 0, 0);
        LinearLayout.LayoutParams rightParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.MATCH_PARENT
        );
        rightPanel.setLayoutParams(rightParams);

        // 热力图标题
        TextView heatmapTitle = new TextView(this);
        heatmapTitle.setTextColor(Color.parseColor("#888888"));
        heatmapTitle.setTextSize(11);
        heatmapTitle.setTypeface(Typeface.MONOSPACE);
        heatmapTitle.setText("差异热力图");
        heatmapTitle.setPadding(0, 0, 0, 8);

        // 热力图 ImageView
        imgDiffHeatmap = new ImageView(this);
        LinearLayout.LayoutParams heatmapParams = new LinearLayout.LayoutParams(300, 300);
        imgDiffHeatmap.setLayoutParams(heatmapParams);
        imgDiffHeatmap.setScaleType(ImageView.ScaleType.FIT_CENTER);
        imgDiffHeatmap.setBackgroundColor(Color.parseColor("#101010"));

        // 初始化 Bitmap
        diffBitmap = Bitmap.createBitmap(100, 100, Bitmap.Config.ARGB_8888);
        diffPixels = new int[100 * 100];
        imgDiffHeatmap.setImageBitmap(diffBitmap);

        // 最高点深度 TextView（放在热力图下方）
        peakDepthText = new TextView(this);
        peakDepthText.setTextColor(Color.parseColor("#CCCCCC"));
        peakDepthText.setTextSize(10);
        peakDepthText.setTypeface(Typeface.MONOSPACE);
        peakDepthText.setPadding(0, 8, 0, 0);
        peakDepthText.setText("最高点: -");

        // 颜色图例
        TextView legendText = new TextView(this);
        legendText.setTextColor(Color.parseColor("#888888"));
        legendText.setTextSize(9);
        legendText.setTypeface(Typeface.MONOSPACE);
        legendText.setText("蓝=低  →  红=高");
        legendText.setPadding(0, 8, 0, 0);

        rightPanel.addView(heatmapTitle);
        rightPanel.addView(imgDiffHeatmap);
        rightPanel.addView(peakDepthText);
        rightPanel.addView(legendText);

        // 组装根布局
        rootLayout.addView(leftPanel);
        rootLayout.addView(rightPanel);
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
        savedBaseline = null;
        calibrationFrames = new ArrayList<>();

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

        float[][] currentFrame = convertToDistanceMatrix(latestDepthData);

        if (baselineDepth == null) {
            baselineDepth = currentFrame;
            calibrationFrames.add(currentFrame);
            stableCount = 1;
            updateCalibrationUI(1, 0);
        } else {
            float avgDiff = calculateAverageDifference(baselineDepth, currentFrame);

            if (avgDiff < BASELINE_AVG_THRESHOLD_MM) {
                stableCount++;
                calibrationFrames.add(currentFrame);
                updateCalibrationUI(stableCount, avgDiff);

                if (stableCount >= STABLE_COUNT_CALIBRATE) {
                    finishCalibration();
                }
            } else {
                baselineDepth = currentFrame;
                calibrationFrames.clear();
                calibrationFrames.add(currentFrame);
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
            sb.append(String.format("稳定进度: %d/%d\n", count, STABLE_COUNT_CALIBRATE));
            sb.append(String.format("帧间差异: %.2f mm (阈值: %.1f mm)\n", diff, BASELINE_AVG_THRESHOLD_MM));
            sb.append("\n当前深度统计:\n");
            sb.append(getDepthStatistics(baselineDepth));
            infoText.setText(sb.toString());
        });
    }

    private void finishCalibration() {
        // 多帧平均作为最终基线，消除单帧噪声
        baselineDepth = averageFrames(calibrationFrames);
        int frameCount = calibrationFrames.size();
        calibrationFrames = null;

        savedBaseline = copyDepthArray(baselineDepth);

        float centerDepth = calculateCenterDepth(baselineDepth);

        // 根据当前校准深度动态计算像素尺寸
        // 像素宽度 = 2 × depth × tan(FOV_H/2) / 100
        // 像素高度 = 2 × depth × tan(FOV_V/2) / 100
        double depthMm = centerDepth;
        pixelSizeX = (float) (2.0 * depthMm * TAN_FOV_H_HALF / 100.0);
        pixelSizeY = (float) (2.0 * depthMm * TAN_FOV_V_HALF / 100.0);

        boolean saved = depthStorage.saveBaseline(baselineDepth);

        runOnUiThread(() -> {
            currentPhase = MeasurementPhase.CALIBRATED;
            btnMeasure.setEnabled(true);
            btnCalibrate.setEnabled(true);

            statusText.setText("状态: 基线校准完成 ✓ (监控中)");
            StringBuilder sb = new StringBuilder();
            sb.append("══════ 基线校准完成 ══════\n\n");
            sb.append(String.format("中心点深度: %.0f mm\n", centerDepth));
            sb.append(String.format("像素尺寸: X=%.2f mm, Y=%.2f mm\n", pixelSizeX, pixelSizeY));
            sb.append(String.format("基线帧数: %d 帧平均\n\n", frameCount));
            sb.append("基线已保存到本地\n\n");
            sb.append("基线深度统计:\n");
            sb.append(getDepthStatistics(baselineDepth));
            sb.append("\n\n正在监控位置变化...");
            infoText.setText(sb.toString());
            appendLog(String.format("基线校准完成 (%d帧平均)，中心点深度: %.0f mm，像素尺寸: X=%.2f Y=%.2f mm",
                    frameCount, centerDepth, pixelSizeX, pixelSizeY));
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
     * 计算列表中数值的平均值
     */
    // ==================== 差异热力图 ====================

    /**
     * 将深度差映射为颜色
     * - diff < threshold → 深灰背景
     * - diff ∈ [threshold, threshold+150] → 蓝→红渐变
     * - diff > threshold+150 → 饱和红
     */
    private int mapDiffToColor(float diff, float threshold) {
        if (diff <= threshold) {
            // 背景：深灰
            return 0xFF202020;
        }
        float maxExtra = 150f; // 超过 150mm 的都按 150 算
        float extra = Math.min(diff - threshold, maxExtra);
        float t = extra / maxExtra; // 0..1

        // 从蓝(0,0,255) → 红(255,0,0) 线性插值
        int r = (int) (255 * t);
        int g = 0;
        int b = (int) (255 * (1 - t));

        return 0xFF000000 | (r << 16) | (g << 8) | b;
    }

    /**
     * 更新差异热力图 - 只显示有效像素
     */
    private void updateDiffHeatmap(ObjectDimensionCalculator calculator) {
        int width = 100;
        int height = 100;

        int[][] mask = calculator.getMask();
        int[] colCount = calculator.getColCount();
        int[] rowCount = calculator.getRowCount();
        int minProjCount = calculator.getMinProjectionCount();

        // 最高点坐标（最大深度差点）
        int peakX = calculator.getMaxDiffX();
        int peakY = calculator.getMaxDiffY();

        // 注意：传感器内部使用的是 [x][y] = [列][行]，
        // 而 Bitmap 使用的是 (x=水平, y=垂直)。
        // 这里显式以 Bitmap 的 (x,y) 为主坐标，按行主序填充 diffPixels。
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // 将 Bitmap 坐标 (x,y) 映射到传感器数据坐标 (dataX,dataY)。
                // 如需旋转方向，只在这里调整映射关系，其他逻辑不受影响。
                int dataX = x;
                int dataY = y;

                int idx = y * width + x;

                // 只显示有效像素：mask=1 且行列投影都满足阈值
                if (mask[dataX][dataY] == 1 &&
                    colCount[dataX] >= minProjCount &&
                    rowCount[dataY] >= minProjCount) {
                    float diff = calculator.getDepthDiffAt(dataX, dataY);
                    diffPixels[idx] = mapDiffToColor(diff, OBJECT_THRESHOLD_MM);
                } else {
                    // 背景像素：深灰
                    diffPixels[idx] = 0xFF101010;
                }
            }
        }

        // 在最高点位置画一个“X”形标记（使用亮紫色），方便肉眼识别
        if (peakX >= 0 && peakY >= 0 &&
                peakX < width && peakY < height) {
            int crossColor = 0xFFFF00FF; // 亮紫色
            int arm = 2; // 叉叉臂长（像素）
            for (int dy = -arm; dy <= arm; dy++) {
                for (int dx = -arm; dx <= arm; dx++) {
                    // 画对角线: abs(dx) == abs(dy)
                    if (Math.abs(dx) != Math.abs(dy)) continue;
                    int xx = peakX + dx;
                    int yy = peakY + dy;
                    if (xx < 0 || xx >= width || yy < 0 || yy >= height) continue;
                    int idx = yy * width + xx;
                    diffPixels[idx] = crossColor;
                }
            }
        }

        // 把像素数组刷进 Bitmap
        diffBitmap.setPixels(diffPixels, 0, width, 0, 0, width, height);
        // 请求重绘
        runOnUiThread(() -> imgDiffHeatmap.invalidate());
    }

    private float average(List<Float> values) {
        if (values == null || values.isEmpty()) return 0;
        float sum = 0;
        for (float v : values) {
            sum += v;
        }
        return sum / values.size();
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
        lastWidth = 0; lastLength = 0; lastHeight = 0;
        lastResult = null;
        noiseMask = null;
        stableWidths.clear();
        stableLengths.clear();
        stableHeights.clear();

        runOnUiThread(() -> {
            btnCalibrate.setText("清除Mask");
            btnCalibrate.setEnabled(true);
            btnMeasure.setEnabled(false);
            statusText.setText("状态: 物体测量中...");
            infoText.setText(String.format("正在测量物体尺寸...\n\n请先点 [清除Mask] 消除噪声，再放置物体\n\n稳定进度: 0/%d", STABLE_COUNT_MEASURE));
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

        ObjectDimensionCalculator calculator = new ObjectDimensionCalculator(
                baselineDepth, currentDepth,
                pixelSizeX, pixelSizeY, OBJECT_THRESHOLD_MM, noiseMask
        );

        ObjectDimensionCalculator.DimensionResult result = calculator.calculate();

        // 获取最高点的深度信息，方便在界面上展示
        int peakX = calculator.getMaxDiffX();
        int peakY = calculator.getMaxDiffY();
        float peakBaseline = calculator.getMaxDiffBaselineDepth();
        float peakCurrent = calculator.getMaxDiffCurrentDepth();
        float peakDiff = peakBaseline - peakCurrent;

        String maskStats = calculator.getMaskDepthDiffStats();
        if (peakX >= 0 && peakY >= 0) {
            maskStats += String.format(
                    "\n最高点: (x=%d,y=%d) baseline=%.0f mm, current=%.0f mm, diff=%.0f mm\n",
                    peakX, peakY, peakBaseline, peakCurrent, peakDiff);
        }
        final String maskStatsFinal = maskStats;

        // 更新右侧最高点 TextView
        if (peakX >= 0 && peakY >= 0) {
            final String peakText = String.format(
                    "最高点: (x=%d,y=%d)\nbaseline=%.0f mm\ncurrent=%.0f mm\n差值=%.0f mm",
                    peakX, peakY, peakBaseline, peakCurrent, peakDiff);
            runOnUiThread(() -> peakDepthText.setText(peakText));
        } else {
            runOnUiThread(() -> peakDepthText.setText("最高点: -"));
        }

        // 更新差异热力图
        updateDiffHeatmap(calculator);

        if (result == null || "未检测到物体".equals(result.message)) {
            runOnUiThread(() -> {
                StringBuilder sb = new StringBuilder();
                sb.append("未检测到物体\n\n请将物体放置在桌面上\n\n");
                sb.append(String.format("有效像素: %d (原始Mask: %d)\n\n",
                        result.validPixelCount, result.rawPixelCount));
                sb.append("───── Mask 诊断 ─────\n");
                sb.append(maskStatsFinal);
                infoText.setText(sb.toString());
            });
            stableCount = 0;
            return;
        }

        float diffW = Math.abs(result.width - lastWidth);
        float diffL = Math.abs(result.length - lastLength);
        float diffH = Math.abs(result.height - lastHeight);
        float maxDimDiff = Math.max(diffW, Math.max(diffL, diffH));

        if (maxDimDiff < DIMENSION_THRESHOLD_MM) {
            stableCount++;
            lastWidth = result.width;
            lastLength = result.length;
            lastHeight = result.height;
            lastResult = result;

            // 累积稳定帧数据
            stableWidths.add(result.width);
            stableLengths.add(result.length);
            stableHeights.add(result.height);

            runOnUiThread(() -> {
                updateMeasurementUI(result, diffW, diffL, diffH, stableCount, maskStatsFinal);
            });

            if (stableCount >= STABLE_COUNT_MEASURE) {
                // 计算稳定帧的平均值
                float avgWidth = average(stableWidths);
                float avgLength = average(stableLengths);
                float avgHeight = average(stableHeights);
                float volumeMm3 = avgWidth * avgLength * avgHeight;
                float volumeCm3 = volumeMm3 / 1000.0f;

                // 创建平均结果
                ObjectDimensionCalculator.DimensionResult avgResult =
                    new ObjectDimensionCalculator.DimensionResult(
                        avgWidth, avgLength, avgHeight,
                        result.rawPixelCount, result.validPixelCount,
                        "测量完成 (" + STABLE_COUNT_MEASURE + "帧平均)"
                    );
                finishMeasurement(avgResult, volumeCm3);
            }
        } else {
            stableCount = 1;
            lastWidth = result.width;
            lastLength = result.length;
            lastHeight = result.height;
            lastResult = result;

            // 重新开始累积
            stableWidths.clear();
            stableLengths.clear();
            stableHeights.clear();
            stableWidths.add(result.width);
            stableLengths.add(result.length);
            stableHeights.add(result.height);

            runOnUiThread(() -> {
                updateMeasurementUI(result, diffW, diffL, diffH, 1, maskStatsFinal);
            });
        }
    }

    private void updateMeasurementUI(ObjectDimensionCalculator.DimensionResult result,
                                     float diffW, float diffL, float diffH,
                                     int count, String maskStats) {
        StringBuilder sb = new StringBuilder();
        sb.append("正在测量物体尺寸...\n\n");
        sb.append(String.format("稳定进度: %d/%d (阈值: %.0fmm)\n", count, STABLE_COUNT_MEASURE, DIMENSION_THRESHOLD_MM));
        sb.append(String.format("偏差: W:%.1f L:%.1f H:%.1fmm\n\n", diffW, diffL, diffH));
        sb.append("══════ 当前测量值 ══════\n");
        sb.append(String.format("宽(W): %.1f mm  (%.1f cm)\n", result.width, result.width / 10));
        sb.append(String.format("长(L): %.1f mm  (%.1f cm)\n", result.length, result.length / 10));
        sb.append(String.format("高(H): %.1f mm  (%.1f cm)\n", result.height, result.height / 10));
        float volumeCm3 = result.width * result.length * result.height / 1000.0f;
        sb.append(String.format("体积: %.1f cm³\n", volumeCm3));
        sb.append(String.format("有效像素: %d (原始Mask: %d)\n\n", result.validPixelCount, result.rawPixelCount));
        sb.append("───── Mask 诊断 ─────\n");
        sb.append(maskStats);
        infoText.setText(sb.toString());
    }

    private void finishMeasurement(ObjectDimensionCalculator.DimensionResult result, float volumeCm3) {
        stopSampling();

        runOnUiThread(() -> {
            currentPhase = MeasurementPhase.MEASURED;
            btnCalibrate.setText("开始校准");
            btnCalibrate.setEnabled(true);
            btnMeasure.setEnabled(true);

            statusText.setText("状态: 测量完成 ✓");
            StringBuilder sb = new StringBuilder();
            sb.append("══════ 最终测量结果 ══════\n\n");
            sb.append(String.format("宽(W): %.1f mm  (%.1f cm)\n", result.width, result.width / 10));
            sb.append(String.format("长(L): %.1f mm  (%.1f cm)\n", result.length, result.length / 10));
            sb.append(String.format("高(H): %.1f mm  (%.1f cm)\n", result.height, result.height / 10));
            sb.append("─────────────────────────\n");
            sb.append(String.format("体积: %.1f cm³\n", volumeCm3));
            sb.append(String.format("有效像素: %d (原始Mask: %d)\n", result.validPixelCount, result.rawPixelCount));
            sb.append("═════════════════════════\n\n");
            sb.append("点击 [重置] 进行新测量");
            infoText.setText(sb.toString());
            appendLog(String.format("测量完成 - W:%.1fmm L:%.1fmm H:%.1fmm V:%.1fcm³",
                    result.width, result.length, result.height, volumeCm3));
        });
    }

    /**
     * 清除当前 Mask：用当前帧刷新 baselineDepth，消除噪声像素
     */
    private void clearCurrentMask() {
        if (currentPhase != MeasurementPhase.MEASURING || latestDepthData == null || baselineDepth == null) {
            return;
        }

        float[][] currentFrame = convertToDistanceMatrix(latestDepthData);
        noiseMask = new boolean[100][100];
        int noiseCount = 0;

        for (int x = 0; x < 100; x++) {
            for (int y = 0; y < 100; y++) {
                float bDepth = baselineDepth[x][y];
                float cDepth = currentFrame[x][y];
                if (bDepth <= 0 || cDepth <= 0) {
                    noiseMask[x][y] = true;
                    noiseCount++;
                } else if (bDepth - cDepth > OBJECT_THRESHOLD_MM) {
                    noiseMask[x][y] = true;
                    noiseCount++;
                }
            }
        }

        stableCount = 0;
        lastWidth = 0; lastLength = 0; lastHeight = 0;

        final int blocked = noiseCount;
        runOnUiThread(() -> {
            statusText.setText("状态: Mask已清除，请放置物体");
            infoText.setText(String.format("已屏蔽 %d 个噪声像素\n\n现在请将物体放置在桌面上\n\n稳定进度: 0/%d", blocked, STABLE_COUNT_MEASURE));
            appendLog(String.format("已清除Mask，屏蔽 %d 个噪声像素", blocked));
        });
    }

    // ==================== 辅助方法 ====================

    /**
     * 将多帧深度数据取平均，消除单帧噪声
     */
    private float[][] averageFrames(List<float[][]> frames) {
        if (frames == null || frames.isEmpty()) return null;
        int rows = frames.get(0).length;
        int cols = frames.get(0)[0].length;
        float[][] result = new float[rows][cols];
        int frameCount = frames.size();

        for (int x = 0; x < rows; x++) {
            for (int y = 0; y < cols; y++) {
                float sum = 0;
                for (float[][] frame : frames) {
                    sum += frame[x][y];
                }
                result[x][y] = sum / frameCount;
            }
        }
        return result;
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

    private float calculateAverageDifference(float[][] frame1, float[][] frame2) {
        float totalDiff = 0;
        int count = 0;

        for (int x = 0; x < frame1.length; x++) {
            for (int y = 0; y < frame1[x].length; y++) {
                float v1 = frame1[x][y];
                float v2 = frame2[x][y];

                // 忽略无效值和异常值
                if (v1 <= 0 || v2 <= 0) continue;
                if (v1 < 100 || v1 > 1000) continue;  // 过滤异常值
                if (v2 < 100 || v2 > 1000) continue;  // 过滤异常值

                // 过滤单点差异过大的（噪声像素通常差异很大）
                float diff = Math.abs(v1 - v2);
                if (diff > 500) continue;  // 单点差异超过 500mm 视为噪声

                totalDiff += diff;
                count++;
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
        stopSampling();
        Runnable newRunnable = new Runnable() {
            @Override
            public void run() {
                if (running && sampleRunnable == this) {
                    sampleAction.run();
                    if (sampleRunnable == this) {
                        mainHandler.postDelayed(this, SAMPLE_INTERVAL_MS);
                    }
                }
            }
        };
        sampleRunnable = newRunnable;
        mainHandler.post(newRunnable);
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
        lastWidth = 0; lastLength = 0; lastHeight = 0;
        lastResult = null;
        currentDepth = null;
        baselineDepth = null;
        savedBaseline = null;
        noiseMask = null;
        stableWidths.clear();
        stableLengths.clear();
        stableHeights.clear();

        // 删除保存的基线文件
        boolean deleted = depthStorage.deleteBaseline();

        runOnUiThread(() -> {
            btnCalibrate.setText("开始校准");
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
