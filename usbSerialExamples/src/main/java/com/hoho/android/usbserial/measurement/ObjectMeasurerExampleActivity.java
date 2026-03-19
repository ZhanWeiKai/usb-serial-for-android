package com.hoho.android.usbserial.measurement;

import android.app.Activity;
import android.graphics.Color;
import android.graphics.Typeface;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.measurement.callback.CalibrationCallback;
import com.hoho.android.usbserial.measurement.callback.ContinuousMeasureCallback;
import com.hoho.android.usbserial.measurement.callback.DepthCallback;
import com.hoho.android.usbserial.measurement.callback.MeasureCallback;
import com.hoho.android.usbserial.measurement.model.BaselineResult;
import com.hoho.android.usbserial.measurement.model.MeasureError;
import com.hoho.android.usbserial.measurement.model.MeasureResult;
import com.hoho.android.usbserial.measurement.model.MeasureStatus;

import java.util.List;

/**
 * ObjectMeasurer SDK 使用示例 Activity
 *
 * 展示如何使用 ObjectMeasurer 进行：
 * 1. 获取深度数据
 * 2. 基线校准
 * 3. 单次测量
 * 4. 连续测量
 */
public class ObjectMeasurerExampleActivity extends Activity {

    private static final String TAG = "MeasurementExample";

    // ========== SDK 实例 ==========
    private ObjectMeasurer objectMeasurer;
    private UsbSerialPort usbSerialPort;

    // ========== UI 组件 ==========
    private TextView textStatus;
    private TextView textResult;
    private TextView textLog;
    private ProgressBar progressBar;
    private ScrollView scrollView;

    private Button btnConnect;
    private Button btnGetDepth;
    private Button btnCalibrate;
    private Button btnMeasureOnce;
    private Button btnMeasureContinuous;
    private Button btnStopMeasure;
    private Button btnDisconnect;

    // ========== 状态 ==========
    private boolean isContinuousMeasuring = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        createLayout();
    }

    /**
     * 创建 UI 布局
     */
    private void createLayout() {
        // 根布局
        LinearLayout rootLayout = new LinearLayout(this);
        rootLayout.setOrientation(LinearLayout.VERTICAL);
        rootLayout.setBackgroundColor(Color.parseColor("#1E1E1E"));
        rootLayout.setPadding(16, 16, 16, 16);

        // ===== 状态区域 =====
        textStatus = new TextView(this);
        textStatus.setTextColor(Color.parseColor("#00FF00"));
        textStatus.setTextSize(16);
        textStatus.setTypeface(Typeface.MONOSPACE);
        textStatus.setText("状态: 请连接USB设备");
        rootLayout.addView(textStatus);

        // ===== 进度条 =====
        progressBar = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        progressBar.setMax(100);
        progressBar.setProgress(0);
        LinearLayout.LayoutParams progressParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
        );
        progressParams.setMargins(0, 16, 0, 16);
        progressBar.setLayoutParams(progressParams);
        rootLayout.addView(progressBar);

        // ===== 按钮区域 1: 连接/断开 =====
        LinearLayout buttonRow1 = new LinearLayout(this);
        buttonRow1.setOrientation(LinearLayout.HORIZONTAL);
        buttonRow1.setPadding(0, 8, 0, 8);

        btnConnect = createButton("连接USB", v -> connectUsb());
        btnDisconnect = createButton("断开", v -> disconnectUsb());
        btnDisconnect.setEnabled(false);
        buttonRow1.addView(btnConnect);
        buttonRow1.addView(btnDisconnect);
        rootLayout.addView(buttonRow1);

        // ===== 按钮区域 2: 深度数据 =====
        LinearLayout buttonRow2 = new LinearLayout(this);
        buttonRow2.setOrientation(LinearLayout.HORIZONTAL);
        buttonRow2.setPadding(0, 8, 0, 8);

        btnGetDepth = createButton("获取深度", v -> getDepthData());
        btnGetDepth.setEnabled(false);
        buttonRow2.addView(btnGetDepth);
        rootLayout.addView(buttonRow2);

        // ===== 按钮区域 3: 校准 =====
        LinearLayout buttonRow3 = new LinearLayout(this);
        buttonRow3.setOrientation(LinearLayout.HORIZONTAL);
        buttonRow3.setPadding(0, 8, 0, 8);

        btnCalibrate = createButton("开始校准", v -> startCalibration());
        btnCalibrate.setEnabled(false);
        buttonRow3.addView(btnCalibrate);
        rootLayout.addView(buttonRow3);

        // ===== 按钮区域 4: 测量 =====
        LinearLayout buttonRow4 = new LinearLayout(this);
        buttonRow4.setOrientation(LinearLayout.HORIZONTAL);
        buttonRow4.setPadding(0, 8, 0, 8);

        btnMeasureOnce = createButton("单次测量", v -> measureOnce());
        btnMeasureOnce.setEnabled(false);
        btnMeasureContinuous = createButton("连续测量", v -> startContinuousMeasure());
        btnMeasureContinuous.setEnabled(false);
        btnStopMeasure = createButton("停止", v -> stopMeasure());
        btnStopMeasure.setEnabled(false);
        buttonRow4.addView(btnMeasureOnce);
        buttonRow4.addView(btnMeasureContinuous);
        buttonRow4.addView(btnStopMeasure);
        rootLayout.addView(buttonRow4);

        // ===== 结果显示区域 =====
        textResult = new TextView(this);
        textResult.setTextColor(Color.parseColor("#00FF00"));
        textResult.setBackgroundColor(Color.parseColor("#2D2D2D"));
        textResult.setTextSize(12);
        textResult.setTypeface(Typeface.MONOSPACE);
        textResult.setPadding(16, 16, 16, 16);
        textResult.setText("等待操作...");
        LinearLayout.LayoutParams resultParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
        );
        resultParams.setMargins(0, 16, 0, 0);
        textResult.setLayoutParams(resultParams);
        rootLayout.addView(textResult);

        // ===== 日志区域 =====
        TextView logTitle = new TextView(this);
        logTitle.setTextColor(Color.parseColor("#888888"));
        logTitle.setTextSize(11);
        logTitle.setText("日志:");
        logTitle.setPadding(0, 16, 0, 4);
        rootLayout.addView(logTitle);

        scrollView = new ScrollView(this);
        LinearLayout.LayoutParams scrollParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1.0f
        );
        scrollView.setLayoutParams(scrollParams);

        textLog = new TextView(this);
        textLog.setTextColor(Color.parseColor("#AAAAAA"));
        textLog.setTextSize(10);
        textLog.setTypeface(Typeface.MONOSPACE);
        textLog.setPadding(8, 8, 8, 8);
        scrollView.addView(textLog);
        rootLayout.addView(scrollView);

        setContentView(rootLayout);
    }

    /**
     * 创建按钮
     */
    private Button createButton(String text, View.OnClickListener listener) {
        Button button = new Button(this);
        button.setText(text);
        button.setOnClickListener(listener);
        LinearLayout.LayoutParams btnParams = new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1.0f
        );
        btnParams.setMargins(4, 0, 4, 0);
        button.setLayoutParams(btnParams);
        return button;
    }

    // ==================== USB 连接 ====================

    /**
     * 连接 USB 设备
     */
    private void connectUsb() {
        appendLog("正在连接USB设备...");

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
                        appendLog("错误: 未找到USB串口设备");
                        Toast.makeText(this, "未找到USB设备", Toast.LENGTH_SHORT).show();
                    });
                    return;
                }

                usbSerialPort = ports.get(0);
                UsbDeviceConnection connection = usbManager.openDevice(usbSerialPort.getDriver().getDevice());

                if (connection == null) {
                    runOnUiThread(() -> {
                        appendLog("错误: 无法打开USB设备，可能需要USB权限");
                        Toast.makeText(this, "无法打开USB设备", Toast.LENGTH_SHORT).show();
                    });
                    return;
                }

                usbSerialPort.open(connection);
                usbSerialPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

                // 配置 A010 传感器
                Thread.sleep(500);
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

                // ========== 创建 ObjectMeasurer 实例 ==========
                objectMeasurer = ObjectMeasurer.builder()
                        .setUsbSerialPort(usbSerialPort)
                        .setFov(70f, 60f)           // A010: 70°×60°
                        .setTiltAngle(45f)          // 俯视角度
                        .setThreshold(50f)          // 物体检测阈值 (mm)
                        .setStableCount(10)         // 稳定帧数要求
                        .build();

                // 启动
                objectMeasurer.start();

                // 更新 UI
                runOnUiThread(() -> {
                    btnConnect.setEnabled(false);
                    btnDisconnect.setEnabled(true);
                    btnGetDepth.setEnabled(true);
                    btnCalibrate.setEnabled(true);

                    textStatus.setText("状态: 已连接");
                    textResult.setText("已连接，请先进行基线校准");
                    appendLog("USB设备连接成功");
                });

                // 检查是否已有基线
                if (objectMeasurer.hasBaseline()) {
                    runOnUiThread(() -> {
                        appendLog("已加载保存的基线数据");
                        enableMeasureButtons(true);
                        textResult.setText("已加载基线，可以直接测量");
                    });
                }

            } catch (Exception e) {
                Log.e(TAG, "USB connection failed", e);
                runOnUiThread(() -> {
                    appendLog("连接失败: " + e.getMessage());
                    Toast.makeText(this, "连接失败: " + e.getMessage(), Toast.LENGTH_SHORT).show();
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
                String response = new String(resp, 0, len);
                Log.d(TAG, "AT命令响应: " + response.trim());
            }
        } catch (Exception e) {
            Log.e(TAG, "发送AT命令失败: " + e.getMessage());
        }
    }

    /**
     * 断开 USB 设备
     */
    private void disconnectUsb() {
        appendLog("正在断开连接...");

        if (objectMeasurer != null) {
            objectMeasurer.stop();
            objectMeasurer.release();
            objectMeasurer = null;
        }

        if (usbSerialPort != null) {
            try {
                usbSerialPort.close();
            } catch (Exception e) {
                Log.e(TAG, "关闭串口失败: " + e.getMessage());
            }
            usbSerialPort = null;
        }

        // 更新 UI
        btnConnect.setEnabled(true);
        btnDisconnect.setEnabled(false);
        btnGetDepth.setEnabled(false);
        btnCalibrate.setEnabled(false);
        enableMeasureButtons(false);

        textStatus.setText("状态: 已断开");
        textResult.setText("等待连接...");
        appendLog("已断开连接");
    }

    // ==================== 深度数据 ====================

    /**
     * 获取单次深度数据
     */
    private void getDepthData() {
        appendLog("获取深度数据...");

        objectMeasurer.getDepthData(new DepthCallback() {
            @Override
            public void onDepthData(float[][] depthMatrix, long timestamp) {
                // 计算统计信息
                float min = Float.MAX_VALUE, max = 0, sum = 0;
                int count = 0;
                for (int x = 0; x < 100; x++) {
                    for (int y = 0; y < 100; y++) {
                        float v = depthMatrix[x][y];
                        if (v > 0) {
                            if (v < min) min = v;
                            if (v > max) max = v;
                            sum += v;
                            count++;
                        }
                    }
                }
                float avg = count > 0 ? sum / count : 0;
                float centerDepth = depthMatrix[50][50];

                final String result = String.format(
                        "深度数据:\n" +
                        "中心深度: %.0f mm\n" +
                        "最小: %.0f mm\n" +
                        "最大: %.0f mm\n" +
                        "平均: %.0f mm\n" +
                        "有效像素: %d",
                        centerDepth, min == Float.MAX_VALUE ? 0 : min, max, avg, count
                );

                runOnUiThread(() -> {
                    textResult.setText(result);
                    appendLog("深度数据获取成功: 中心=" + (int)centerDepth + "mm");
                });
            }

            @Override
            public void onError(MeasureError error) {
                runOnUiThread(() -> {
                    appendLog("错误: " + error.getLocalizedMessage());
                    Toast.makeText(ObjectMeasurerExampleActivity.this,
                            error.getLocalizedMessage(), Toast.LENGTH_SHORT).show();
                });
            }
        });
    }

    // ==================== 校准 ====================

    /**
     * 开始基线校准
     */
    private void startCalibration() {
        appendLog("开始基线校准，请保持桌面清洁...");
        textStatus.setText("状态: 校准中...");
        textResult.setText("正在采集空桌面数据...\n请保持桌面清洁");

        btnCalibrate.setEnabled(false);
        enableMeasureButtons(false);
        progressBar.setProgress(0);

        objectMeasurer.calibrateBaseline(new CalibrationCallback() {
            @Override
            public void onProgress(int current, int total, float avgDifference) {
                runOnUiThread(() -> {
                    progressBar.setMax(total);
                    progressBar.setProgress(current);
                    textResult.setText(String.format(
                            "校准进度: %d/%d\n" +
                            "帧间差异: %.1f mm\n\n" +
                            "请保持桌面清洁...",
                            current, total, avgDifference
                    ));
                });
            }

            @Override
            public void onSuccess(BaselineResult result) {
                runOnUiThread(() -> {
                    appendLog("校准完成!");
                    textStatus.setText("状态: 已校准");
                    btnCalibrate.setEnabled(true);
                    enableMeasureButtons(true);

                    textResult.setText(String.format(
                            "══════ 基线校准完成 ══════\n\n" +
                            "中心深度: %.0f mm (%.1f cm)\n" +
                            "像素尺寸: X=%.2f mm, Y=%.2f mm\n" +
                            "视野范围: %.1f × %.1f cm\n\n" +
                            "现在可以开始测量物体",
                            result.centerDepth, result.centerDepth / 10,
                            result.pixelSizeX, result.pixelSizeY,
                            result.getViewWidthCm(), result.getViewHeightCm()
                    ));
                });
            }

            @Override
            public void onError(MeasureError error) {
                runOnUiThread(() -> {
                    appendLog("校准失败: " + error.getLocalizedMessage());
                    textStatus.setText("状态: 校准失败");
                    btnCalibrate.setEnabled(true);
                    textResult.setText("校准失败: " + error.getLocalizedMessage());
                });
            }
        });
    }

    // ==================== 测量 ====================

    /**
     * 单次测量
     */
    private void measureOnce() {
        appendLog("开始单次测量...");
        textStatus.setText("状态: 测量中...");
        textResult.setText("正在测量，请保持物体稳定...");

        btnMeasureOnce.setEnabled(false);
        btnMeasureContinuous.setEnabled(false);
        progressBar.setProgress(0);

        objectMeasurer.measureOnce(new MeasureCallback() {
            @Override
            public void onProgress(int current, int total, MeasureResult intermediate) {
                runOnUiThread(() -> {
                    progressBar.setMax(total);
                    progressBar.setProgress(current);
                    textResult.setText(String.format(
                            "稳定进度: %d/%d\n\n" +
                            "当前测量:\n" +
                            "W=%.1f mm, L=%.1f mm, H=%.1f mm\n\n" +
                            "请保持物体稳定...",
                            current, total,
                            intermediate.width, intermediate.length, intermediate.height
                    ));
                });
            }

            @Override
            public void onSuccess(MeasureResult result) {
                runOnUiThread(() -> {
                    appendLog(String.format("测量完成: W=%.1f, L=%.1f, H=%.1f mm, V=%.1f cm³",
                            result.width, result.length, result.height, result.volumeCm3));
                    textStatus.setText("状态: 测量完成");
                    btnMeasureOnce.setEnabled(true);
                    btnMeasureContinuous.setEnabled(true);

                    textResult.setText(result.getFormattedResult());
                });
            }

            @Override
            public void onError(MeasureError error) {
                runOnUiThread(() -> {
                    if (error.code == MeasureError.ErrorCode.OBJECT_NOT_DETECTED) {
                        appendLog("未检测到物体");
                        textResult.setText("未检测到物体\n\n请将物体放置在桌面上");
                    } else {
                        appendLog("测量错误: " + error.getLocalizedMessage());
                        textResult.setText("测量失败: " + error.getLocalizedMessage());
                    }
                    textStatus.setText("状态: 测量失败");
                    btnMeasureOnce.setEnabled(true);
                    btnMeasureContinuous.setEnabled(true);
                });
            }
        });
    }

    /**
     * 开始连续测量
     */
    private void startContinuousMeasure() {
        appendLog("开始连续测量...");
        textStatus.setText("状态: 连续测量中");
        textResult.setText("连续测量中，请放置物体...");

        isContinuousMeasuring = true;
        btnMeasureOnce.setEnabled(false);
        btnMeasureContinuous.setEnabled(false);
        btnStopMeasure.setEnabled(true);

        objectMeasurer.startContinuousMeasure(new ContinuousMeasureCallback() {
            @Override
            public void onResult(MeasureResult result) {
                runOnUiThread(() -> {
                    textResult.setText(String.format(
                            "══════ 实时测量 ══════\n\n" +
                            "宽(W): %.1f mm  (%.1f cm)\n" +
                            "长(L): %.1f mm  (%.1f cm)\n" +
                            "高(H): %.1f mm  (%.1f cm)\n" +
                            "─────────────────────\n" +
                            "体积: %.1f cm³\n" +
                            "有效像素: %d\n" +
                            "稳定: %s (%d/10)\n" +
                            "═════════════════════",
                            result.width, result.getWidthCm(),
                            result.length, result.getLengthCm(),
                            result.height, result.getHeightCm(),
                            result.volumeCm3,
                            result.validPixelCount,
                            result.isStable ? "是" : "否",
                            result.stableCount
                    ));

                    if (result.isStable) {
                        appendLog(String.format("稳定结果: W=%.1f, L=%.1f, H=%.1f mm",
                                result.width, result.length, result.height));
                    }
                });
            }

            @Override
            public void onStatusChanged(MeasureStatus status) {
                runOnUiThread(() -> {
                    textStatus.setText("状态: " + status.message);
                });
            }

            @Override
            public void onError(MeasureError error) {
                runOnUiThread(() -> {
                    if (error.code != MeasureError.ErrorCode.OBJECT_NOT_DETECTED) {
                        appendLog("错误: " + error.getLocalizedMessage());
                    }
                });
            }
        });
    }

    /**
     * 停止测量
     */
    private void stopMeasure() {
        appendLog("停止测量");
        objectMeasurer.stopMeasure();

        isContinuousMeasuring = false;
        btnMeasureOnce.setEnabled(true);
        btnMeasureContinuous.setEnabled(true);
        btnStopMeasure.setEnabled(false);

        textStatus.setText("状态: 已停止");
        textResult.setText("测量已停止\n\n点击 [单次测量] 或 [连续测量] 开始新的测量");
    }

    // ==================== 工具方法 ====================

    /**
     * 启用/禁用测量按钮
     */
    private void enableMeasureButtons(boolean enabled) {
        btnMeasureOnce.setEnabled(enabled);
        btnMeasureContinuous.setEnabled(enabled);
        if (!enabled) {
            btnStopMeasure.setEnabled(false);
        }
    }

    /**
     * 添加日志
     */
    private void appendLog(String message) {
        final String timestamp = new java.text.SimpleDateFormat("HH:mm:ss", java.util.Locale.getDefault())
                .format(new java.util.Date());
        final String logLine = String.format("[%s] %s", timestamp, message);

        runOnUiThread(() -> {
            textLog.append(logLine + "\n");
            // 滚动到底部
            scrollView.post(() -> scrollView.fullScroll(View.FOCUS_DOWN));
        });

        Log.d(TAG, message);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (objectMeasurer != null) {
            objectMeasurer.stop();
            objectMeasurer.release();
        }
        if (usbSerialPort != null) {
            try {
                usbSerialPort.close();
            } catch (Exception e) {
                Log.e(TAG, "关闭串口失败: " + e.getMessage());
            }
        }
    }
}
