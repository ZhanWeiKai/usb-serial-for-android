package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.graphics.Color;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.graphics.Typeface;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * A010 原始数据显示 Activity
 *
 * 上方固定区域：显示最新帧的详细信息
 * 下方滚动区域：显示历史帧的简要日志
 */
public class A010RawDataActivity extends Activity {

    private static final String TAG = "A010RawData222";

    // 帧格式常量
    private static final int HEADER_SIZE = 2;       // 0x00 0xFF
    private static final int LENGTH_SIZE = 2;       // 包长度字段
    private static final int META_SIZE = 16;        // 元数据
    private static final int PIXEL_COUNT = 10000;   // 100x100
    private static final int CHECKSUM_TAIL = 2;     // 校验 + 包尾

    private static final int HEAD_1 = 0x00;
    private static final int HEAD_2 = 0xFF;

    private UsbSerialPort usbSerialPort;
    private final ByteArrayOutputStream buffer = new ByteArrayOutputStream();
    private final Object bufferLock = new Object();  // 添加锁，解决并发问题
    private int frameCounter = 0;
    private volatile boolean running = true;

    // 帧验证统计
    private int validFrameCount = 0;
    private int invalidFrameCount = 0;

    // UI
    private TextView fixedInfo;      // 上方固定信息
    private TextView logView;        // 下方日志
    private ScrollView scrollView;   // 滚动容器
    private final SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault());

    // 设备配置状态
    private int currentFps = 0;
    private int currentUnit = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // 创建布局
        createLayout();

        Log.i(TAG, "A010RawDataActivity started");
        connectUsb();
    }

    private void createLayout() {
        LinearLayout rootLayout = new LinearLayout(this);
        rootLayout.setOrientation(LinearLayout.VERTICAL);
        rootLayout.setBackgroundColor(Color.parseColor("#1E1E1E"));

        // 上方固定信息区域
        fixedInfo = new TextView(this);
        fixedInfo.setTextColor(Color.parseColor("#00FF00"));
        fixedInfo.setBackgroundColor(Color.parseColor("#2D2D2D"));
        fixedInfo.setTextSize(12);
        fixedInfo.setTypeface(Typeface.MONOSPACE);
        fixedInfo.setPadding(16, 16, 16, 16);
        LinearLayout.LayoutParams fixedParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
        );
        fixedInfo.setLayoutParams(fixedParams);
        fixedInfo.setText("等待连接设备...\n\n帧格式: header(2) + len(2) + meta(16) + pixels(10000) + checksum(1) + tail(1)");

        // 下方滚动日志区域
        logView = new TextView(this);
        logView.setTextColor(Color.parseColor("#AAAAAA"));
        logView.setTextSize(11);
        logView.setTypeface(Typeface.MONOSPACE);
        logView.setPadding(16, 8, 16, 16);

        scrollView = new ScrollView(this);
        scrollView.setBackgroundColor(Color.parseColor("#1E1E1E"));
        LinearLayout.LayoutParams scrollParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1.0f
        );
        scrollView.setLayoutParams(scrollParams);
        scrollView.addView(logView);
        scrollView.setSmoothScrollingEnabled(true);

        rootLayout.addView(fixedInfo);
        rootLayout.addView(scrollView);
        setContentView(rootLayout);
    }

    private void connectUsb() {
        UsbManager usbManager = (UsbManager) getSystemService(USB_SERVICE);
        List<UsbSerialPort> ports = UsbSerialProber.getDefaultProber()
                .findAllDrivers(usbManager)
                .stream()
                .flatMap(driver -> driver.getPorts().stream())
                .toList();

        if (ports.isEmpty()) {
            updateFixedInfo("错误: 未找到USB串口设备");
            return;
        }

        usbSerialPort = ports.get(0);
        UsbDeviceConnection connection = usbManager.openDevice(usbSerialPort.getDriver().getDevice());
        if (connection == null) {
            updateFixedInfo("错误: 无法打开USB设备 (权限问题?)");
            return;
        }

        try {
            usbSerialPort.open(connection);

            // Step 1: 先用默认波特率115200通信
            usbSerialPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            Log.i(TAG, "串口已打开，波特率: 115200");

            Thread.sleep(500);  // 增加等待时间，让设备稳定
            clearBuffer();

            // Step 2: 关闭 ISP，确保没有帧数据干扰 AT 命令
            sendAtWithResponse("AT+ISP=0\r\n", "关闭ISP");
            Thread.sleep(500);  // 等待 ISP 完全关闭

            // Step 3: 在 ISP 关闭状态下配置所有参数（此时无帧数据干扰）
            sendAtWithResponse("AT+BINN=1\r\n", "分辨率100x100");
            Thread.sleep(200);
            sendAtWithResponse("AT+UNIT=0\r\n", "自动量化");
            Thread.sleep(200);
            sendAtWithResponse("AT+FPS=15\r\n", "帧率15fps");
            Thread.sleep(200);
            sendAtWithResponse("AT+DISP=6\r\n", "USB+UART输出");  // 6 = USB+UART
            Thread.sleep(200);
            sendAtWithResponse("AT+SAVE\r\n", "保存配置");
            Thread.sleep(500);

            // Step 4: 最后启动 ISP（开始输出帧数据）
            sendAtWithResponse("AT+ISP=1\r\n", "启动ISP");
            Log.i(TAG, "ISP 已启动，等待稳定...");
            Thread.sleep(2000);  // ISP 启动需要 1-2 秒

            // Step 5: 查询设备状态
            String deviceStatus = queryDeviceStatus();
            Log.i(TAG, "设备状态:\n" + deviceStatus);

            Thread.sleep(200);
            clearBuffer();
            synchronized (bufferLock) {
                buffer.reset();
            }

            updateFixedInfo("═══ 设备状态 ═══\n" +
                    deviceStatus + "\n" +
                    "────────────────\n" +
                    "波特率: 2000000\n" +
                    "等待帧数据...");

            new Thread(this::readLoop).start();

        } catch (Exception e) {
            Log.e(TAG, "Setup failed", e);
            updateFixedInfo("错误: " + e.getMessage());
        }
    }

    private void clearBuffer() {
        try {
            byte[] tmp = new byte[4096];
            while (usbSerialPort.read(tmp, 50) > 0) {
                // 清空
            }
        } catch (Exception ignored) {}
    }

    private String sendAtWithResponse(String cmd, String description) {
        try {
            String cmdStr = ">>> " + cmd.trim();
            Log.i(TAG, cmdStr + " (" + description + ")");

            // 显示发送的命令
            runOnUiThread(() -> appendLog(cmdStr));

            usbSerialPort.write(cmd.getBytes(), 500);
            Thread.sleep(100);  // 增加等待时间

            // 读取AT响应，避免响应数据混入帧buffer
            byte[] resp = new byte[256];
            int len = usbSerialPort.read(resp, 500);  // 增加超时时间到500ms
            if (len > 0) {
                String response = new String(resp, 0, len).trim();
                Log.i(TAG, "<<< " + response);

                // 显示收到的响应
                String respStr = "<<< " + response;
                runOnUiThread(() -> appendLog(respStr));

                return response;
            } else {
                runOnUiThread(() -> appendLog("<<< (无响应)"));
            }
        } catch (Exception e) {
            Log.e(TAG, "AT error: " + cmd.trim() + " - " + e.getMessage());
            runOnUiThread(() -> appendLog("ERR: " + e.getMessage()));
        }
        return "";
    }

    // 查询设备状态并返回状态字符串
    private String queryDeviceStatus() {
        StringBuilder status = new StringBuilder();

        // ISP 状态
        String isp = sendAtWithResponse("AT+ISP?\r", "查询ISP状态");
        status.append("ISP:    ").append(parseIspResponse(isp)).append("\n");

        // BINN 分辨率
        String binn = sendAtWithResponse("AT+BINN?\r", "查询BINN状态");
        status.append("BINN:   ").append(parseBinnResponse(binn)).append("\n");

        // BAUD 波特率
        String baud = sendAtWithResponse("AT+BAUD?\r", "查询BAUD状态");
        status.append("BAUD:   ").append(parseBaudResponse(baud)).append("\n");

        // UNIT 量化单位
        String unit = sendAtWithResponse("AT+UNIT?\r", "查询UNIT状态");
        status.append("UNIT:   ").append(parseUnitResponse(unit)).append("\n");

        // FPS 帧率
        String fps = sendAtWithResponse("AT+FPS?\r", "查询FPS状态");
        status.append("FPS:    ").append(parseFpsResponse(fps)).append("\n");

        // DISP 显示输出
        String disp = sendAtWithResponse("AT+DISP?\r", "查询DISP状态");
        status.append("DISP:   ").append(parseDispResponse(disp));

        return status.toString();
    }

    // 解析 ISP 响应 - 格式: +ISP=1 或 ISP=1
    private String parseIspResponse(String response) {
        Log.d(TAG, "parseIspResponse raw: [" + response + "]");
        if (response.contains("+ISP=1") || response.contains("ISP=1")) return "1 (开启)";
        if (response.contains("+ISP=0") || response.contains("ISP=0")) return "0 (关闭)";
        return response.isEmpty() ? "(无响应)" : response;
    }

    // 解析 FPS 响应 - 格式: +FPS=15
    private String parseFpsResponse(String response) {
        Log.d(TAG, "parseFpsResponse raw: [" + response + "]");
        // 尝试提取数字
        for (int i = 1; i <= 30; i++) {
            if (response.contains("=" + i) || response.contains("+FPS=" + i)) {
                return i + " fps";
            }
        }
        return response.isEmpty() ? "(无响应)" : response;
    }

    // 解析 BINN 响应 - 格式: +BINN=1
    private String parseBinnResponse(String response) {
        Log.d(TAG, "parseBinnResponse raw: [" + response + "]");
        if (response.contains("+BINN=1") || response.endsWith("=1")) return "1 (100x100)";
        if (response.contains("+BINN=2") || response.endsWith("=2")) return "2 (50x50)";
        if (response.contains("+BINN=4") || response.endsWith("=4")) return "4 (25x25)";
        return response.isEmpty() ? "(无响应)" : response;
    }

    // 解析 BAUD 响应 - 格式: +BAUD=7
    private String parseBaudResponse(String response) {
        Log.d(TAG, "parseBaudResponse raw: [" + response + "]");
        String[] baudRates = {"9600", "57600", "115200", "230400", "460800", "921600", "1000000", "2000000", "3000000"};
        for (int i = 0; i < baudRates.length; i++) {
            if (response.contains("=" + i) || response.contains("+BAUD=" + i)) {
                return i + " (" + baudRates[i] + ")";
            }
        }
        return response.isEmpty() ? "(无响应)" : response;
    }

    // 解析 UNIT 响应 - 格式: +UNIT=1
    private String parseUnitResponse(String response) {
        Log.d(TAG, "parseUnitResponse raw: [" + response + "]");
        if (response.contains("+UNIT=0") || response.endsWith("=0")) return "0 (自动量化)";
        for (int i = 1; i <= 9; i++) {
            if (response.contains("+UNIT=" + i) || response.endsWith("=" + i)) {
                return i + " (" + i + "mm/像素)";
            }
        }
        return response.isEmpty() ? "(无响应)" : response;
    }

    // 解析 DISP 响应 - 格式: +DISP=6
    private String parseDispResponse(String response) {
        Log.d(TAG, "parseDispResponse raw: [" + response + "]");
        String[] dispModes = {
            "0 (全部关闭)", "1 (LCD)", "2 (USB)", "3 (LCD+USB)",
            "4 (UART)", "5 (LCD+UART)", "6 (USB+UART)", "7 (全部开启)"
        };
        for (int i = 0; i < dispModes.length; i++) {
            if (response.contains("+DISP=" + i) || response.endsWith("=" + i)) {
                return dispModes[i];
            }
        }
        return response.isEmpty() ? "(无响应)" : response;
    }

    private void readLoop() {
        byte[] readBuf = new byte[8192];
        int totalBytesRead = 0;

        Log.i(TAG, "readLoop started");

        while (running) {
            try {
                int len = usbSerialPort.read(readBuf, 100);  // 缩短超时以更快响应
                if (len > 0) {
                    totalBytesRead += len;
                    synchronized (bufferLock) {
                        buffer.write(readBuf, 0, len);
                    }
                    // 每1MB打印一次统计
                    if (totalBytesRead % 1000000 < len) {
                        Log.i(TAG, "Total bytes read: " + totalBytesRead);
                    }
                    parseFramesLocked();
                }
            } catch (Exception e) {
                Log.e(TAG, "Read error", e);
            }
        }
        Log.i(TAG, "readLoop ended");
    }

    // 使用锁保护整个解析过程
    private void parseFramesLocked() {
        synchronized (bufferLock) {
            byte[] data = buffer.toByteArray();
            int pos = 0;
            int minHeaderSize = HEADER_SIZE + LENGTH_SIZE;
            boolean foundFrame = false;

            while (pos + minHeaderSize <= data.length) {
                // 查找帧头 0x00 0xFF
                if ((data[pos] & 0xFF) != HEAD_1 || (data[pos + 1] & 0xFF) != HEAD_2) {
                    pos++;
                    continue;
                }

                // 读取长度字段 (小端)
                // payloadLen = 元数据(16) + 像素数据，不包含校验和包尾
                int payloadLen = (data[pos + 2] & 0xFF) | ((data[pos + 3] & 0xFF) << 8);
                int frameSize = HEADER_SIZE + LENGTH_SIZE + payloadLen + CHECKSUM_TAIL;  // +2 for checksum + tail

                // 验证长度合理性 (预期 payloadLen = 10016 = 16 + 10000)
                if (payloadLen < 10000 || payloadLen > 10050) {
                    Log.w(TAG, "Invalid payloadLen: " + payloadLen + " at pos=" + pos);
                    pos++;
                    continue;
                }

                // 检查是否有完整帧
                if (pos + frameSize > data.length) {
                    break;  // 等待更多数据
                }

                // 验证包尾 0xDD
                int tailPos = pos + frameSize - 1;
                int tailByte = data[tailPos] & 0xFF;
                boolean frameValid = (tailByte == 0xDD);

                if (!frameValid) {
                    invalidFrameCount++;
                    Log.w(TAG, String.format("Invalid tail: 0x%02X (expected 0xDD) at frame #%d", tailByte, frameCounter + 1));
                    // 显示失败信息到UI
                    final int failFrameNum = frameCounter + 1;
                    final int failTail = tailByte;
                    runOnUiThread(() -> {
                        appendLog(String.format("❌ Frame #%d 尾部验证失败: 0x%02X (期望 0xDD)", failFrameNum, failTail));
                    });
                    pos++;
                    continue;
                }

                validFrameCount++;

                // 提取数据 - 从帧头开始计算偏移
                int metaStart = pos + HEADER_SIZE + LENGTH_SIZE;  // pos + 4
                int pixelStart = metaStart + META_SIZE;            // pos + 20

                byte[] metaData = Arrays.copyOfRange(data, metaStart, metaStart + META_SIZE);
                byte[] firstRow = Arrays.copyOfRange(data, pixelStart, pixelStart + 100);

                // 第50行50列附近的像素 (中心区域) - 偏移 = 50*100+50 = 5050
                int centerOffset = 50 * 100 + 50;
                byte[] centerPixels = Arrays.copyOfRange(data, pixelStart + centerOffset, pixelStart + centerOffset + 10);

                frameCounter++;
                foundFrame = true;
                String time = timeFormat.format(new Date());

                // 详细调试：打印帧位置信息和像素值
                Log.i(TAG, String.format("Frame #%d @ buffer_pos=%d, frameSize=%d, tail=0x%02X ✓", frameCounter, pos, frameSize, tailByte));
                Log.i(TAG, String.format("  firstRow[0-9]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                        firstRow[0] & 0xFF, firstRow[1] & 0xFF, firstRow[2] & 0xFF,
                        firstRow[3] & 0xFF, firstRow[4] & 0xFF, firstRow[5] & 0xFF,
                        firstRow[6] & 0xFF, firstRow[7] & 0xFF, firstRow[8] & 0xFF,
                        firstRow[9] & 0xFF));
                Log.i(TAG, String.format("  center[5050-5059]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                        centerPixels[0] & 0xFF, centerPixels[1] & 0xFF, centerPixels[2] & 0xFF,
                        centerPixels[3] & 0xFF, centerPixels[4] & 0xFF, centerPixels[5] & 0xFF,
                        centerPixels[6] & 0xFF, centerPixels[7] & 0xFF, centerPixels[8] & 0xFF,
                        centerPixels[9] & 0xFF));

                // 更新UI
                final int frameNum = frameCounter;
                final String frameTime = time;
                final byte[] meta = metaData;
                final byte[] pixels = firstRow.clone();
                final byte[] center = centerPixels.clone();
                final int validCount = validFrameCount;
                final int invalidCount = invalidFrameCount;

                runOnUiThread(() -> {
                    updateFixedInfo(formatFixedInfo(frameNum, frameTime, payloadLen, meta, pixels, center, validCount, invalidCount));
                });

                // 移除已处理的帧 - 关键：正确更新buffer
                int newStart = pos + frameSize;
                if (newStart < data.length) {
                    byte[] remaining = Arrays.copyOfRange(data, newStart, data.length);
                    buffer.reset();
                    try {
                        buffer.write(remaining);
                    } catch (Exception ignored) {}
                    // 重要：必须重新获取data，因为buffer内容已改变
                    data = buffer.toByteArray();
                } else {
                    buffer.reset();
                    data = new byte[0];
                }
                pos = 0;  // 从头开始扫描剩余数据
            }

            // 防止buffer无限增长
            if (!foundFrame && buffer.size() > 30000) {
                Log.w(TAG, "Buffer overflow, dropping old data");
                byte[] remaining = Arrays.copyOfRange(buffer.toByteArray(), 10000, buffer.size());
                buffer.reset();
                try {
                    buffer.write(remaining);
                } catch (Exception ignored) {}
            }
        }
    }

    private String formatFixedInfo(int frameNum, String time, int payloadLen, byte[] meta, byte[] pixels, byte[] center, int validCount, int invalidCount) {
        StringBuilder sb = new StringBuilder();
        sb.append("═══════════════════════════════════════\n");
        sb.append("帧: #").append(frameNum).append("  ").append(time);
        sb.append(" [").append(frameNum % 10).append("]");
        sb.append(" ✓有效:").append(validCount).append(" ✗无效:").append(invalidCount).append("\n");
        sb.append("───────────────────────────────────────\n");
        // 第一行像素 (原始值)
        sb.append("像素[0-9]:   ");
        for (int i = 0; i < 10 && i < pixels.length; i++) {
            sb.append(String.format("%3d ", pixels[i] & 0xFF));
        }
        sb.append("\n");
        // 第一行像素 (距离 cm) - UNIT=0: 距离(mm) = (p/5.1)²
        sb.append("距离[0-9]:   ");
        for (int i = 0; i < 10 && i < pixels.length; i++) {
            int distMm = calculateDistanceMm(pixels[i] & 0xFF);
            sb.append(String.format("%3d ", distMm / 10));  // 转换为 cm
        }
        sb.append(" cm\n");
        // 中心区域像素 (原始值)
        sb.append("中心像素:   ");
        for (int i = 0; i < 10 && i < center.length; i++) {
            sb.append(String.format("%3d ", center[i] & 0xFF));
        }
        sb.append("\n");
        // 中心区域像素 (距离 cm)
        sb.append("中心距离:   ");
        for (int i = 0; i < 10 && i < center.length; i++) {
            int distMm = calculateDistanceMm(center[i] & 0xFF);
            sb.append(String.format("%3d ", distMm / 10));  // 转换为 cm
        }
        sb.append(" cm\n");
        sb.append("───────────────────────────────────────\n");
        sb.append("元数据(16字节):\n");
        sb.append("  ").append(bytesToHex(meta, 0, 8)).append("\n");
        sb.append("  ").append(bytesToHex(meta, 8, 16)).append("\n");
        sb.append("───────────────────────────────────────\n");
        sb.append("第一行像素(100字节 HEX):\n");
        sb.append("  ").append(bytesToHex(pixels, 0, 50)).append("\n");
        sb.append("  ").append(bytesToHex(pixels, 50, 100)).append("\n");
        sb.append("═══════════════════════════════════════");
        return sb.toString();
    }

    /**
     * 计算像素值对应的距离 (UNIT=0 模式)
     * 公式: 距离(mm) = (像素值 / 5.1)²
     * @param pixelValue 像素值 (0-255)
     * @return 距离 (mm)
     */
    private int calculateDistanceMm(int pixelValue) {
        if (pixelValue == 0) return 0;
        double distance = Math.pow(pixelValue / 5.1, 2);
        return (int) Math.round(distance);
    }

    private String bytesToHex(byte[] data, int count) {
        return bytesToHex(data, 0, count);
    }

    private String bytesToHex(byte[] data, int start, int end) {
        StringBuilder sb = new StringBuilder();
        StringBuilder ascii = new StringBuilder();
        for (int i = start; i < end && i < data.length; i++) {
            sb.append(String.format("%02X ", data[i] & 0xFF));
            char c = (char) (data[i] & 0xFF);
            ascii.append(c >= 32 && c < 127 ? c : '.');
        }
        // 对齐
        while (sb.length() < 50) {
            sb.append("   ");
        }
        return sb.toString() + " | " + ascii;
    }

    private void updateFixedInfo(String text) {
        fixedInfo.setText(text);
    }

    private void appendLog(String line) {
        logView.append(line + "\n");

        // 限制日志行数，保持最后200行
        String currentLog = logView.getText().toString();
        String[] lines = currentLog.split("\n");
        if (lines.length > 200) {
            StringBuilder newLog = new StringBuilder();
            for (int i = lines.length - 200; i < lines.length; i++) {
                newLog.append(lines[i]).append("\n");
            }
            logView.setText(newLog.toString());
        }

        // 滚动到底部
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
