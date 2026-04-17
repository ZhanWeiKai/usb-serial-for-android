package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.graphics.Color;
import android.graphics.Typeface;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * A010 串口原始数据读取 - 支持 USB 直连和 CP2102 UART
 * 点击按钮连接，显示最原始的串口数据 (HEX + ASCII)
 */
public class A010Cp2102Activity extends Activity {

    private static final String TAG = "A010_SERIAL";

    private static final int BAUD_RATE = 115200;

    private UsbSerialPort usbSerialPort;
    private volatile boolean running = false;
    private long totalBytesRead = 0;

    // UI
    private Button connectBtn;
    private TextView headerView;
    private TextView dataView;
    private ScrollView scrollView;
    private final SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault());
    private final StringBuilder logBuffer = new StringBuilder();
    private static final int MAX_LOG_SIZE = 50000;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        createLayout();
    }

    private void createLayout() {
        LinearLayout rootLayout = new LinearLayout(this);
        rootLayout.setOrientation(LinearLayout.VERTICAL);
        rootLayout.setBackgroundColor(Color.parseColor("#1A1A1A"));
        rootLayout.setGravity(Gravity.CENTER_HORIZONTAL);

        // 状态栏
        headerView = new TextView(this);
        headerView.setTextColor(Color.parseColor("#00FF00"));
        headerView.setBackgroundColor(Color.parseColor("#2D2D2D"));
        headerView.setTextSize(12);
        headerView.setTypeface(Typeface.MONOSPACE);
        headerView.setPadding(12, 8, 12, 8);
        LinearLayout.LayoutParams headerParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
        );
        headerView.setLayoutParams(headerParams);
        headerView.setText("点击下方按钮连接串口设备");

        // 中间按钮
        connectBtn = new Button(this);
        connectBtn.setText("连 接 设 备");
        connectBtn.setTextColor(Color.WHITE);
        connectBtn.setBackgroundColor(Color.parseColor("#00796B"));
        connectBtn.setTextSize(20);
        connectBtn.setAllCaps(false);
        connectBtn.setGravity(Gravity.CENTER);
        LinearLayout.LayoutParams btnParams = new LinearLayout.LayoutParams(
                400,
                150
        );
        btnParams.gravity = Gravity.CENTER_HORIZONTAL;
        btnParams.topMargin = 80;
        btnParams.bottomMargin = 80;
        connectBtn.setLayoutParams(btnParams);
        connectBtn.setOnClickListener(v -> {
            if (running) {
                // 断开
                running = false;
                try { if (usbSerialPort != null) usbSerialPort.close(); } catch (Exception ignored) {}
                connectBtn.setText("连 接 设 备");
                connectBtn.setBackgroundColor(Color.parseColor("#00796B"));
                appendData("已断开连接");
            } else {
                connectBtn.setEnabled(false);
                connectBtn.setText("连接中...");
                connectBtn.setBackgroundColor(Color.parseColor("#555555"));
                new Thread(this::connectUsb).start();
            }
        });

        // 原始数据滚动区
        dataView = new TextView(this);
        dataView.setTextColor(Color.parseColor("#CCCCCC"));
        dataView.setTextSize(10);
        dataView.setTypeface(Typeface.MONOSPACE);
        dataView.setPadding(8, 4, 8, 8);

        scrollView = new ScrollView(this);
        scrollView.setBackgroundColor(Color.parseColor("#222222"));
        LinearLayout.LayoutParams scrollParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1.0f
        );
        scrollView.setLayoutParams(scrollParams);
        scrollView.addView(dataView);

        rootLayout.addView(headerView);
        rootLayout.addView(connectBtn);
        rootLayout.addView(scrollView);
        setContentView(rootLayout);
    }

    private void connectUsb() {
        UsbManager usbManager = (UsbManager) getSystemService(USB_SERVICE);

        // 和 A010RawDataActivity 一样的连接方式：查找所有串口，取第一个
        List<UsbSerialPort> ports = UsbSerialProber.getDefaultProber()
                .findAllDrivers(usbManager)
                .stream()
                .flatMap(driver -> driver.getPorts().stream())
                .toList();

        if (ports.isEmpty()) {
            runOnUiThread(() -> {
                updateHeader("错误: 未找到 USB 串口设备");
                resetButton();
            });
            return;
        }

        usbSerialPort = ports.get(0);
        runOnUiThread(() -> appendData("找到设备: " + usbSerialPort.getDriver().getClass().getSimpleName()));

        UsbDeviceConnection connection = usbManager.openDevice(usbSerialPort.getDriver().getDevice());
        if (connection == null) {
            runOnUiThread(() -> {
                updateHeader("错误: 无法打开 USB 设备 (权限?)");
                resetButton();
            });
            return;
        }

        try {
            usbSerialPort.open(connection);
            usbSerialPort.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            Log.i(TAG, "Serial opened, baud=" + BAUD_RATE);
            runOnUiThread(() -> appendData("串口已打开, 波特率: " + BAUD_RATE));

            Thread.sleep(500);
            clearBuffer();

            // 和 A010RawDataActivity 完全一样的 AT 配置流程
            sendAt("AT+ISP=0\r\n");
            Thread.sleep(500);

            sendAt("AT+BINN=1\r\n");
            Thread.sleep(200);
            sendAt("AT+UNIT=0\r\n");
            Thread.sleep(200);
            sendAt("AT+FPS=15\r\n");
            Thread.sleep(200);
            sendAt("AT+DISP=6\r\n");
            Thread.sleep(200);
            sendAt("AT+SAVE\r\n");
            Thread.sleep(500);

            sendAt("AT+ISP=1\r\n");
            Thread.sleep(2000);

            // 查询状态
            sendAt("AT+ISP?\r");
            Thread.sleep(200);
            sendAt("AT+BINN?\r");
            Thread.sleep(200);
            sendAt("AT+FPS?\r");
            Thread.sleep(200);
            sendAt("AT+DISP?\r");
            Thread.sleep(200);

            clearBuffer();

            running = true;
            runOnUiThread(() -> {
                connectBtn.setText("断 开 连 接");
                connectBtn.setBackgroundColor(Color.parseColor("#C62828"));
                connectBtn.setEnabled(true);
                updateHeader("已连接 | 波特率: " + BAUD_RATE + " | DISP=6\n正在读取原始数据...");
            });

            new Thread(this::readLoop).start();

        } catch (Exception e) {
            Log.e(TAG, "Setup failed", e);
            runOnUiThread(() -> {
                updateHeader("连接失败: " + e.getMessage());
                resetButton();
            });
        }
    }

    private void resetButton() {
        connectBtn.setEnabled(true);
        connectBtn.setText("连 接 设 备");
        connectBtn.setBackgroundColor(Color.parseColor("#00796B"));
    }

    private void clearBuffer() {
        try {
            byte[] tmp = new byte[4096];
            while (usbSerialPort.read(tmp, 50) > 0) {}
        } catch (Exception ignored) {}
    }

    private void sendAt(String cmd) {
        try {
            Log.i(TAG, ">>> " + cmd.trim());
            usbSerialPort.write(cmd.getBytes(), 500);
            Thread.sleep(100);
            byte[] resp = new byte[256];
            int len = usbSerialPort.read(resp, 500);
            if (len > 0) {
                String response = new String(resp, 0, len).trim();
                Log.i(TAG, "<<< " + response);
                runOnUiThread(() -> appendData(">>> " + cmd.trim() + "  ->  " + response));
            } else {
                runOnUiThread(() -> appendData(">>> " + cmd.trim() + "  ->  (无响应)"));
            }
        } catch (Exception e) {
            Log.e(TAG, "AT error: " + cmd.trim());
        }
    }

    private void readLoop() {
        byte[] readBuf = new byte[8192];
        Log.i(TAG, "readLoop started");

        while (running) {
            try {
                int len = usbSerialPort.read(readBuf, 100);
                if (len > 0) {
                    totalBytesRead += len;

                    StringBuilder hex = new StringBuilder();
                    StringBuilder ascii = new StringBuilder();
                    for (int i = 0; i < len; i++) {
                        int b = readBuf[i] & 0xFF;
                        hex.append(String.format("%02X ", b));
                        ascii.append(b >= 32 && b < 127 ? (char) b : '.');
                    }

                    String time = timeFormat.format(new Date());
                    String line = String.format("[%s] %d bytes\nHEX: %s\nASC: %s",
                            time, len, hex.toString().trim(), ascii.toString());

                    final String displayLine = line;
                    final long total = totalBytesRead;
                    runOnUiThread(() -> {
                        appendData(displayLine);
                        updateHeader("已连接 | 波特率: " + BAUD_RATE +
                                " | DISP=6\n已读取: " + total + " bytes");
                    });
                }
            } catch (Exception e) {
                Log.e(TAG, "Read error", e);
            }
        }
        Log.i(TAG, "readLoop ended");
    }

    private void updateHeader(String text) {
        headerView.setText(text);
    }

    private void appendData(String line) {
        logBuffer.append(line).append("\n\n");
        if (logBuffer.length() > MAX_LOG_SIZE) {
            logBuffer.delete(0, logBuffer.length() - MAX_LOG_SIZE / 2);
        }
        dataView.setText(logBuffer.toString());
        scrollView.post(() -> scrollView.fullScroll(ScrollView.FOCUS_DOWN));
    }

    @Override
    protected void onDestroy() {
        running = false;
        try {
            if (usbSerialPort != null) {
                usbSerialPort.close();
            }
        } catch (Exception ignored) {}
        super.onDestroy();
    }
}
