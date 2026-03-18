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
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import com.hoho.android.usbserial.examples.FrameBuffer;

// 多帧中值滤波 (用于梯度检测降噪)

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
    private static final long SAMPLE_INTERVAL_MS = 500;  // 0.5秒采样间隔 (输出刷新周期)
    private static final int SAMPLES_PER_WINDOW = 10;    // 每个窗口内采样10帧
    private static final long SAMPLE_FAST_INTERVAL_MS = 50;  // 窗口内快速采样间隔 (500/10=50 ≈ 2帧)

    private static final int STABLE_COUNT_CALIBRATE = 10; // 校准: 连续10次稳定 = 5秒
    private static final int STABLE_COUNT_MEASURE = 10;   // 测量: 连续10次稳定 = 5秒

    // 窗口中值滤波参数
    private Handler windowSamplingHandler = new Handler(Looper.getMainLooper());
    private Runnable windowSamplingRunnable;
    private int windowFrameCount = 0;
    private Runnable pendingSampleAction;
    private List<float[][]> windowFrames = new ArrayList<>();  // 窗口帧缓存
    private float[][] latestMedianDepth;  // 中值降噪后的深度图

    // 测量时：取maxDepthDiff中位数对应的帧
    private List<Float> windowMaxDepthDiffs = new ArrayList<>();  // 每帧的maxDepthDiff
    private float stableMaxDepthDiff;      // 选中帧的maxDepthDiff
    private int stableMaxDiffX, stableMaxDiffY;  // 选中帧maxDepthDiff的位置
    private int stableXMin, stableXMax, stableYMin, stableYMax;  // 选中帧的边界范围

    // 稳定性阈值
    private static final float BASELINE_AVG_THRESHOLD_MM = 14.0f;  // 基线平均值差异阈值 (mm) - 改为14mm
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

    private static final float OBJECT_THRESHOLD_MM = 50.0f;  // 物体检测阈值 (mm)

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

    // 多帧中值缓冲器 (用于梯度检测降噪)
    private FrameBuffer frameBuffer = new FrameBuffer(100);
    private long frameStartTime = 0; // 当前采集窗口开始时间

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
        // 根布局：可滚动的垂直布局，内容在上方，热力图在下方（原 logView 位置）
        ScrollView rootScroll = new ScrollView(this);
        rootScroll.setFillViewport(true);

        LinearLayout rootLayout = new LinearLayout(this);
        rootLayout.setOrientation(LinearLayout.VERTICAL);
        rootLayout.setBackgroundColor(Color.parseColor("#1E1E1E"));
        rootLayout.setPadding(16, 16, 16, 16);

        // ===== 主内容面板：状态 + 文本 + 热力图 + 按钮 =====
        LinearLayout leftPanel = new LinearLayout(this);
        leftPanel.setOrientation(LinearLayout.VERTICAL);
        LinearLayout.LayoutParams leftParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
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
                LinearLayout.LayoutParams.WRAP_CONTENT
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

        // 热力图放在原 logView 位置（infoText 下方，按钮上方）
        // 热力图标题
        TextView heatmapTitle = new TextView(this);
        heatmapTitle.setTextColor(Color.parseColor("#888888"));
        heatmapTitle.setTextSize(11);
        heatmapTitle.setTypeface(Typeface.MONOSPACE);
        heatmapTitle.setText("差异热力图");
        heatmapTitle.setPadding(0, 0, 0, 8);

        // 热力图 ImageView（进一步放大，便于观察细节）
        imgDiffHeatmap = new ImageView(this);
        LinearLayout.LayoutParams heatmapParams = new LinearLayout.LayoutParams(700, 700);
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

        // 把热力图区域加到左侧面板（infoText 下面）
        leftPanel.addView(heatmapTitle);
        leftPanel.addView(imgDiffHeatmap);
        leftPanel.addView(peakDepthText);
        leftPanel.addView(legendText);

        // 按钮放在最底部
        leftPanel.addView(buttonLayout);

        // 根布局只包含一个垂直面板，外面包一层 ScrollView 方便整体滚动
        rootLayout.addView(leftPanel);
        rootScroll.addView(rootLayout);
        setContentView(rootScroll);
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

        // 启动窗口采样模式 (每500ms采集10帧取中值)
        startWindowSampling(this::processCalibrationSample);
    }

    private void processCalibrationSample() {
        // 使用中值降噪后的深度图， 而非原始数据
        if (latestMedianDepth == null) {
            return;
        }

        float[][] currentFrame = latestMedianDepth;  // 直接使用中值降噪后的深度图

        if (baselineDepth == null) {
            baselineDepth = copyDepthArray(currentFrame);
            calibrationFrames.add(copyDepthArray(currentFrame));
            stableCount = 1;
            updateCalibrationUI(1, 0);
        } else {
            float avgDiff = calculateAverageDifference(baselineDepth, currentFrame);

            if (avgDiff < BASELINE_AVG_THRESHOLD_MM) {
                stableCount++;
                calibrationFrames.add(copyDepthArray(currentFrame));
                updateCalibrationUI(stableCount, avgDiff);

                if (stableCount >= STABLE_COUNT_CALIBRATE) {
                    finishCalibration();
                }
            } else {
                baselineDepth = copyDepthArray(currentFrame);
                calibrationFrames.clear();
                calibrationFrames.add(copyDepthArray(currentFrame));
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
        // 多帧中值作为最终基线，比平均值更抗噪声
        baselineDepth = medianFrames(calibrationFrames);
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
            sb.append(String.format("中心点深度: %.0f mm (%.1f cm)\n", centerDepth, centerDepth / 10.0));
            sb.append(String.format("像素尺寸: X=%.2f mm, Y=%.2f mm\n", pixelSizeX, pixelSizeY));
            sb.append(String.format("基线帧数: %d 帧平均\n\n", frameCount));

            // 显示空桌面的像素范围和物理尺寸
            int imageWidth = 100;  // 图像宽度像素
            int imageHeight = 100; // 图像高度像素
            float physicalWidthMm = imageWidth * pixelSizeX;  // X方向物理尺寸
            float physicalHeightMm = imageHeight * pixelSizeY; // Y方向物理尺寸
            sb.append("══════ 空桌面视野范围 ══════\n");
            sb.append(String.format("像素范围: %d × %d\n", imageWidth, imageHeight));
            sb.append(String.format("物理尺寸: %.1f × %.1f cm\n", physicalWidthMm / 10.0, physicalHeightMm / 10.0));
            sb.append(String.format("(宽 × 长)\n\n", pixelSizeX, pixelSizeY));

            sb.append("基线已保存到本地\n\n");
            sb.append("基线深度统计:\n");
            sb.append(getDepthStatistics(baselineDepth));
            sb.append("\n\n正在监控位置变化...");
            infoText.setText(sb.toString());
            appendLog(String.format("基线校准完成 (%d帧平均)，中心点深度: %.0f mm，像素尺寸: X=%.2f Y=%.2f mm，视野范围: %.1f×%.1f cm",
                    frameCount, centerDepth, pixelSizeX, pixelSizeY,
                    100 * pixelSizeX / 10.0, 100 * pixelSizeY / 10.0));
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

        // 防御性检查：如果投影数据为空，跳过热力图更新
        if (colCount == null || rowCount == null) {
            return;
        }

        // 获取有效像素的边界范围，用于画叉叉标记
        int bxMin = calculator.getXMin();
        int bxMax = calculator.getXMax();
        int byMin = calculator.getYMin();
        int byMax = calculator.getYMax();

        Log.d(TAG, "═══ updateDiffHeatmap 开始 ═══");
        Log.d(TAG, String.format("有效像素范围: x[%d-%d] y[%d-%d]", bxMin, bxMax, byMin, byMax));

        // 计算中心深度（边界中心点或有效像素平均深度）
        float centerDepth = calculator.getCenterDepth();
        float depthMin = centerDepth - 200;  // 深度下限
        float depthMax = centerDepth + 200;  // 深度上限

        Log.d(TAG, String.format("中心深度: %.1fmm, 有效范围: %.1f~%.1fmm", centerDepth, depthMin, depthMax));

        // 收集异常深度信息和边缘像素深度
        List<String> anomalyInfo = new ArrayList<>();
        List<String> edgeDepthInfo = new ArrayList<>();
        int anomalyCount = 0;

        // 统计有效像素的深度分布
        int validPixelStats = 0;
        float minDepthFound = Float.MAX_VALUE;
        float maxDepthFound = Float.MIN_VALUE;
        float sumDepth = 0;

        // 注意：传感器内部使用的是 [x][y] = [列][行]，
        // 而 Bitmap 使用的是 (x=水平, y=垂直)。
        // 这里显式以 Bitmap 的 (x,y) 为主坐标，按行主序填充 diffPixels。
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // 将 Bitmap 坐标 (x,y) 映射到传感器数据坐标 (dataX,dataY)。
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

        // 在有效像素范围的四个角画紫色叉叉标记
        if (bxMin >= 0 && bxMax >= 0 && byMin >= 0 && byMax >= 0) {
            int crossColor = 0xFFFF00FF; // 亮紫色
            int arm = 2; // 叉叉臂长（像素）

            // 四个角的位置
            int[][] corners = {
                {bxMin, byMin},  // 左上
                {bxMax, byMin},  // 右上
                {bxMin, byMax},  // 左下
                {bxMax, byMax}   // 右下
            };

            for (int[] corner : corners) {
                int cx = corner[0];
                int cy = corner[1];
                if (cx < 0 || cx >= width || cy < 0 || cy >= height) continue;

                for (int dy = -arm; dy <= arm; dy++) {
                    for (int dx = -arm; dx <= arm; dx++) {
                        // 画对角线: abs(dx) == abs(dy)
                        if (Math.abs(dx) != Math.abs(dy)) continue;
                        int xx = cx + dx;
                        int yy = cy + dy;
                        if (xx < 0 || xx >= width || yy < 0 || yy >= height) continue;
                        int idx = yy * width + xx;
                        diffPixels[idx] = crossColor;
                    }
                }
            }

            // 收集边缘像素深度信息
            // 四条边：上、下、左、右
            StringBuilder edgeSb = new StringBuilder();
            edgeSb.append("边缘像素深度:\n");

            // 上边缘 (y = byMin)
            edgeSb.append(String.format("上边缘(y=%d): ", byMin));
            for (int x = bxMin; x <= bxMax; x += 3) {
                float depth = currentDepth[x][byMin];
                boolean isAnomaly = depth < depthMin || depth > depthMax;
                edgeSb.append(isAnomaly ? "*" : "");
                edgeSb.append(String.format("%d=%.0f ", x, depth));
            }
            edgeSb.append("\n");

            // 下边缘 (y = byMax)
            edgeSb.append(String.format("下边缘(y=%d): ", byMax));
            for (int x = bxMin; x <= bxMax; x += 3) {
                float depth = currentDepth[x][byMax];
                boolean isAnomaly = depth < depthMin || depth > depthMax;
                edgeSb.append(isAnomaly ? "*" : "");
                edgeSb.append(String.format("%d=%.0f ", x, depth));
            }
            edgeSb.append("\n");

            // 左边缘 (x = bxMin)
            edgeSb.append(String.format("左边缘(x=%d): ", bxMin));
            for (int y = byMin; y <= byMax; y += 3) {
                float depth = currentDepth[bxMin][y];
                boolean isAnomaly = depth < depthMin || depth > depthMax;
                edgeSb.append(isAnomaly ? "*" : "");
                edgeSb.append(String.format("%d=%.0f ", y, depth));
            }
            edgeSb.append("\n");

            // 右边缘 (x = bxMax)
            edgeSb.append(String.format("右边缘(x=%d): ", bxMax));
            for (int y = byMin; y <= byMax; y += 3) {
                float depth = currentDepth[bxMax][y];
                boolean isAnomaly = depth < depthMin || depth > depthMax;
                edgeSb.append(isAnomaly ? "*" : "");
                edgeSb.append(String.format("%d=%.0f ", y, depth));
            }
            edgeSb.append("\n");

            edgeDepthInfo.add(edgeSb.toString());
        }

        // 用红色圈圈标记异常深度的像素
        if (bxMin >= 0 && bxMax >= 0 && byMin >= 0 && byMax >= 0 && currentDepth != null) {
            int circleColor = 0xFFFF0000; // 红色（更明显）

            Log.d(TAG, String.format("开始检测异常深度, currentDepth!=null: %b", currentDepth != null));

            for (int x = bxMin; x <= bxMax; x++) {
                if (colCount[x] < minProjCount) continue;
                for (int y = byMin; y <= byMax; y++) {
                    if (rowCount[y] < minProjCount) continue;
                    if (mask[x][y] != 1) continue;

                    float depth = currentDepth[x][y];

                    // 统计有效像素深度分布
                    validPixelStats++;
                    sumDepth += depth;
                    if (depth < minDepthFound) minDepthFound = depth;
                    if (depth > maxDepthFound) maxDepthFound = depth;

                    if (depth < depthMin || depth > depthMax) {
                        // 异常深度，画圈圈
                        anomalyCount++;
                        drawCircle(diffPixels, width, height, x, y, circleColor);

                        // 记录异常信息（只记录前20个）
                        if (anomalyCount <= 20) {
                            anomalyInfo.add(String.format("(%d,%d)=%.0fmm", x, y, depth));
                            Log.d(TAG, String.format("异常深度 #%d: (%d,%d)=%.1fmm (范围:%.1f~%.1f)",
                                    anomalyCount, x, y, depth, depthMin, depthMax));
                        }
                    }
                }
            }

            // 详细统计日志
            if (validPixelStats > 0) {
                float avgDepth = sumDepth / validPixelStats;
                Log.d(TAG, String.format("有效像素深度统计: count=%d, min=%.1f, max=%.1f, avg=%.1f",
                        validPixelStats, minDepthFound, maxDepthFound, avgDepth));
            }
            Log.d(TAG, String.format("异常检测结果: centerDepth=%.0f, range=%.0f~%.0f, anomaly=%d",
                    centerDepth, depthMin, depthMax, anomalyCount));
        } else {
            Log.d(TAG, String.format("跳过异常检测: bxMin=%d, bxMax=%d, byMin=%d, byMax=%d, currentDepth=%s",
                    bxMin, bxMax, byMin, byMax, currentDepth == null ? "null" : "ok"));
        }

        // 把像素数组刷进 Bitmap
        diffBitmap.setPixels(diffPixels, 0, width, 0, 0, width, height);

        // 更新异常深度信息显示
        final int finalAnomalyCount = anomalyCount;
        final String anomalyText = anomalyInfo.isEmpty() ? "" :
            String.format("异常深度(黄圈): %d个\n%s", finalAnomalyCount,
                String.join(", ", anomalyInfo.subList(0, Math.min(10, anomalyInfo.size()))));
        final String edgeText = edgeDepthInfo.isEmpty() ? "" : edgeDepthInfo.get(0);
        final float finalCenterDepth = centerDepth;
        final float finalDepthMin = depthMin;
        final float finalDepthMax = depthMax;

        runOnUiThread(() -> {
            imgDiffHeatmap.invalidate();
            // 更新异常信息显示
            if (peakDepthText != null) {
                StringBuilder sb = new StringBuilder();
                sb.append(String.format("中心深度: %.0fmm\n", finalCenterDepth));
                sb.append(String.format("有效范围: %.0f~%.0fmm\n", finalDepthMin, finalDepthMax));
                sb.append("───────────────────\n");
                sb.append(anomalyText);
                if (!edgeText.isEmpty()) {
                    sb.append("\n───────────────────\n");
                    sb.append(edgeText);
                    sb.append("(* 表示异常深度)\n");
                }
                peakDepthText.setText(sb.toString());
            }
        });
    }

    /**
     * 在像素数组上画一个圈圈（更大的圈）
     */
    private void drawCircle(int[] pixels, int width, int height, int cx, int cy, int color) {
        // 画一个更大的圈圈，半径为3像素
        for (int dy = -3; dy <= 3; dy++) {
            for (int dx = -3; dx <= 3; dx++) {
                // 只画圈圈边缘，不是实心圆
                int distSq = dx * dx + dy * dy;
                if (distSq < 4 || distSq > 9) continue;  // 只画半径2-3的环

                int px = cx + dx;
                int py = cy + dy;
                if (px >= 0 && px < width && py >= 0 && py < height) {
                    int idx = py * width + px;
                    pixels[idx] = color;
                }
            }
        }
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
        frameBuffer.clear();  // 清空帧缓冲器
        frameStartTime = 0;   // 重置采集窗口时间
        stableWidths.clear();
        stableLengths.clear();
        stableHeights.clear();
        windowMaxDepthDiffs.clear();  // 清空maxDepthDiff缓存
        stableMaxDepthDiff = 0;
        stableMaxDiffX = stableMaxDiffY = -1;
        stableXMin = stableYMin = 100;
        stableXMax = stableYMax = -1;

        runOnUiThread(() -> {
            btnCalibrate.setText("清除Mask");
            btnCalibrate.setEnabled(true);
            btnMeasure.setEnabled(false);
            statusText.setText("状态: 物体测量中...");
            infoText.setText(String.format("正在测量物体尺寸...\n\n请先点 [清除Mask] 消除噪声，再放置物体\n\n稳定进度: 0/%d", STABLE_COUNT_MEASURE));
            appendLog("开始物体测量");
        });

        // 启动窗口采样模式 (每500ms采集10帧取中值)
        startWindowSampling(this::processMeasurementSample);
    }

    private void processMeasurementSample() {
        // 使用中值降噪后的深度图（每500ms采集10帧取中值）
        if (latestMedianDepth == null || baselineDepth == null) {
            return;
        }

        // 直接使用中值降噪后的深度图
        currentDepth = copyDepthArray(latestMedianDepth);

        // 创建计算器，处理当前帧
        ObjectDimensionCalculator calculator = new ObjectDimensionCalculator(
                baselineDepth, currentDepth,
                (float)FOV_HORIZONTAL_DEG, (float)FOV_VERTICAL_DEG, OBJECT_THRESHOLD_MM, noiseMask
        );

        // 直接计算尺寸（使用新的直接连续像素法）
        // 深度图已经做了中值滤波，不需要再做投影数据的FrameBuffer
        ObjectDimensionCalculator.DimensionResult result = calculator
                .calculateDimensionsByCalibratedRatioWithMedianData(pixelSizeX, pixelSizeY);

        String maskStats = calculator.getMaskDepthDiffStats();

        // 添加调试信息：有效像素的像素坐标范围
        int pxMin = calculator.getXMin();
        int pxMax = calculator.getXMax();
        int pyMin = calculator.getYMin();
        int pyMax = calculator.getYMax();
        maskStats += String.format(
                "\n边界像素范围: x[%d-%d] y[%d-%d] (%d×%d)\n",
                pxMin, pxMax, pyMin, pyMax,
                (pxMax - pxMin + 1), (pyMax - pyMin + 1));
        final String maskStatsFinal = maskStats;

        // 更新差异热力图
        updateDiffHeatmap(calculator);

        if (result == null || "未检测到物体".equals(result.message) || result.validPixelCount == 0) {
            runOnUiThread(() -> {
                StringBuilder sb = new StringBuilder();
                sb.append("未检测到物体\n\n请将物体放置在桌面上\n");
                sb.append("(物体需至少占8个连续像素)\n\n");
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
                        "测量完成 (" + STABLE_COUNT_MEASURE + "帧平均)",
                        result.xMin, result.xMax, result.yMin, result.yMax
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
            sb.append(String.format("像素范围: x[%d-%d] y[%d-%d]\n", result.xMin, result.xMax, result.yMin, result.yMax));
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

    /**
     * 将多帧深度数据取中值，比平均值更抗噪声
     * 对脉冲噪声（如个别异常深度值)更鲁棒
     */
    private float[][] medianFrames(List<float[][]> frames) {
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

    // ==================== 窗口采样机制 ====================

    /**
     * 启动窗口采样模式
     * 在500ms内快速采集10帧，取中值后执行回调
     */
    private void startWindowSampling(Runnable sampleAction) {
        stopSampling();
        windowFrames.clear();
        windowMaxDepthDiffs.clear();
        windowFrameCount = 0;
        pendingSampleAction = sampleAction;

        windowSamplingRunnable = new Runnable() {
            @Override
            public void run() {
                if (!running || windowSamplingRunnable != this) {
                    return;  // 已停止，退出
                }

                // 如果还没有深度数据，等待后重试
                if (latestDepthData == null) {
                    runOnUiThread(() -> {
                        frameStatsText.setText("等待深度数据...");
                    });
                    windowSamplingHandler.postDelayed(this, SAMPLE_FAST_INTERVAL_MS);
                    return;
                }

                // 快速采集一帧
                float[][] frame = convertToDistanceMatrix(latestDepthData);
                windowFrames.add(frame);
                windowFrameCount++;

                // 测量模式下：计算每帧的maxDepthDiff
                if (currentPhase == MeasurementPhase.MEASURING && baselineDepth != null) {
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
                    windowMaxDepthDiffs.add(maxDiff);
                }

                // 更新UI显示
                final float frameMaxDiff = windowMaxDepthDiffs.isEmpty() ? 0 :
                        windowMaxDepthDiffs.get(windowMaxDepthDiffs.size() - 1);
                runOnUiThread(() -> {
                    frameStatsText.setText(String.format("窗口采样: %d/%d帧 (maxDiff=%.1fmm)",
                            windowFrameCount, SAMPLES_PER_WINDOW, frameMaxDiff));
                });

                // 采集满10帧后取中值
                if (windowFrameCount >= SAMPLES_PER_WINDOW) {
                    // 测量模式：取maxDepthDiff中位数对应的帧
                    if (currentPhase == MeasurementPhase.MEASURING && !windowMaxDepthDiffs.isEmpty()) {
                        selectMedianMaxDiffFrame();
                    } else {
                        // 校准模式：对深度图每个像素取中值
                        latestMedianDepth = medianFrames(windowFrames);
                    }
                    windowFrames.clear();
                    windowMaxDepthDiffs.clear();
                    windowFrameCount = 0;

                    // 执行回调
                    if (pendingSampleAction != null) {
                        pendingSampleAction.run();
                    }

                    // 一个窗口完成，等待500ms后开始下一个窗口
                    windowSamplingHandler.postDelayed(this, SAMPLE_INTERVAL_MS);
                } else {
                    // 窗口内：每50ms采集一帧
                    windowSamplingHandler.postDelayed(this, SAMPLE_FAST_INTERVAL_MS);
                }
            }
        };

        // 启动窗口采样
        windowSamplingHandler.post(windowSamplingRunnable);
    }

    /**
     * 从10帧中选取maxDepthDiff中位数对应的那一帧
     * 并记录该帧的maxDepthDiff、位置和边界范围
     */
    private void selectMedianMaxDiffFrame() {
        if (windowMaxDepthDiffs.isEmpty() || windowFrames.isEmpty()) {
            return;
        }

        // 复制并排序，找中位数
        List<Float> sorted = new ArrayList<>(windowMaxDepthDiffs);
        Collections.sort(sorted);
        int midIndex = sorted.size() / 2;
        float medianValue = sorted.get(midIndex);

        // 找到最接近中位数的帧索引
        int bestFrameIndex = 0;
        float minDiff = Float.MAX_VALUE;
        for (int i = 0; i < windowMaxDepthDiffs.size(); i++) {
            float diff = Math.abs(windowMaxDepthDiffs.get(i) - medianValue);
            if (diff < minDiff) {
                minDiff = diff;
                bestFrameIndex = i;
            }
        }

        // 使用该帧作为输出
        latestMedianDepth = windowFrames.get(bestFrameIndex);
        stableMaxDepthDiff = windowMaxDepthDiffs.get(bestFrameIndex);

        // 找该帧的maxDepthDiff位置和边界
        float maxDiff = 0;
        int maxX = -1, maxY = -1;
        boolean[][] mask = new boolean[100][100];

        for (int x = 0; x < 100; x++) {
            for (int y = 0; y < 100; y++) {
                float bDepth = baselineDepth[x][y];
                float cDepth = latestMedianDepth[x][y];
                if (bDepth > 100 && bDepth < 800 && cDepth > 100 && cDepth < 800) {
                    float diff = bDepth - cDepth;
                    if (diff > OBJECT_THRESHOLD_MM) {
                        mask[x][y] = true;
                    }
                    if (diff > maxDiff) {
                        maxDiff = diff;
                        maxX = x;
                        maxY = y;
                    }
                }
            }
        }

        stableMaxDiffX = maxX;
        stableMaxDiffY = maxY;

        // 计算边界范围（连续像素法）
        stableXMin = 100; stableXMax = -1;
        stableYMin = 100; stableYMax = -1;

        // 行方向：找连续>=8个有效像素的行
        for (int y = 0; y < 100; y++) {
            int consecutive = 0;
            for (int x = 0; x < 100; x++) {
                if (mask[x][y]) {
                    consecutive++;
                } else {
                    if (consecutive >= 8) {
                        if (y < stableYMin) stableYMin = y;
                        if (y > stableYMax) stableYMax = y;
                    }
                    consecutive = 0;
                }
            }
            if (consecutive >= 8) {
                if (y < stableYMin) stableYMin = y;
                if (y > stableYMax) stableYMax = y;
            }
        }

        // 列方向：找连续>=8个有效像素的列
        for (int x = 0; x < 100; x++) {
            int consecutive = 0;
            for (int y = 0; y < 100; y++) {
                if (mask[x][y]) {
                    consecutive++;
                } else {
                    if (consecutive >= 8) {
                        if (x < stableXMin) stableXMin = x;
                        if (x > stableXMax) stableXMax = x;
                    }
                    consecutive = 0;
                }
            }
            if (consecutive >= 8) {
                if (x < stableXMin) stableXMin = x;
                if (x > stableXMax) stableXMax = x;
            }
        }

        Log.d(TAG, String.format("10帧中位数帧: 第%d帧, maxDiff=%.1fmm, 位置(%d,%d), 边界x[%d-%d] y[%d-%d]",
                bestFrameIndex + 1, stableMaxDepthDiff, stableMaxDiffX, stableMaxDiffY,
                stableXMin, stableXMax, stableYMin, stableYMax));
    }

    /**
     * 停止窗口采样
     */
    private void stopWindowSampling() {
        if (windowSamplingHandler != null) {
            windowSamplingHandler.removeCallbacks(windowSamplingRunnable);
            windowSamplingRunnable = null;
        }
    }

    /**
     * 停止采样 (stopWindowSampling的别名)
     */
    private void stopSampling() {
        stopWindowSampling();
    }

    /**
     * 启动采样 (startWindowSampling的别名)
     */
    private void startSampling(Runnable sampleAction) {
        startWindowSampling(sampleAction);
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
        frameBuffer.clear();  // 清空帧缓冲器

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
