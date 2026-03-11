package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

/**
 * MaixSense-A010 TOF 传感器测试 Activity
 *
 * 帧格式 (参考开发手册):
 * - 帧头: 2 字节 (0x00, 0xFF)
 * - 包长度: 2 字节 (小端, 剩余数据长度 = meta + pixels + checksum + tail)
 * - 元数据: 16 字节
 * - 图像帧: 10000 字节 (100x100 像素)
 * - 校验: 1 字节 (忽略)
 * - 包尾: 1 字节 (忽略)
 * - 总计: 2 + 2 + 16 + 10000 + 1 + 1 = 10022 字节
 */
public class A010AtQuickTestActivity extends Activity {

    private static final String TAG = "A010_TEST123";

    // 帧格式常量
    private static final int WIDTH = 100;
    private static final int HEIGHT = 100;
    private static final int HEADER_SIZE = 2;       // 0x00 0xFF
    private static final int LENGTH_SIZE = 2;       // 包长度字段
    private static final int META_SIZE = 16;        // 元数据
    private static final int PIXEL_COUNT = WIDTH * HEIGHT;  // 10000
    private static final int CHECKSUM_TAIL = 2;     // 校验 + 包尾 (忽略)
    private static final int FRAME_SIZE = HEADER_SIZE + LENGTH_SIZE + META_SIZE + PIXEL_COUNT + CHECKSUM_TAIL;  // 10022

    // 帧头
    private static final int HEAD_1 = 0x00;
    private static final int HEAD_2 = 0xFF;

    private UsbSerialPort usbSerialPort;
    private final ByteArrayOutputStream buffer = new ByteArrayOutputStream();
    private int frameCounter = 0;
    private volatile boolean running = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.i(TAG, "========== A010 Test Start ==========");
        Log.i(TAG, "Frame format: header(2) + len(2) + meta(16) + pixels(10000) + checksum(1) + tail(1) = 10022 bytes");

        UsbManager usbManager = (UsbManager) getSystemService(USB_SERVICE);
        List<UsbSerialPort> ports = UsbSerialProber.getDefaultProber()
                .findAllDrivers(usbManager)
                .stream()
                .flatMap(driver -> driver.getPorts().stream())
                .toList();

        if (ports.isEmpty()) {
            Log.e(TAG, "No USB serial device found");
            return;
        }

        usbSerialPort = ports.get(0);
        UsbDeviceConnection connection = usbManager.openDevice(usbSerialPort.getDriver().getDevice());
        if (connection == null) {
            Log.e(TAG, "Failed to open USB device (permission?)");
            return;
        }

        try {
            // 打开串口 - 使用更高波特率以支持 15fps @ 10018 bytes/frame
            usbSerialPort.open(connection);
            usbSerialPort.setParameters(2000000, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            Log.i(TAG, "Serial opened: 2000000, 8N1");

            // 等待设备稳定
            Thread.sleep(500);

            // 清空旧数据
            clearBuffer();

            // 配置设备 - 设置高波特率以支持大数据传输
            sendAt("AT+ISP=0\r\n");    // 先关闭 ISP
            sendAt("AT+BAUD=7\r\n");   // 2000000 波特率
            sendAt("AT+SAVE\r\n");     // 保存波特率设置
            Thread.sleep(100);

            // 重新设置串口波特率
            usbSerialPort.setParameters(2000000, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            Log.i(TAG, "Baud rate changed to 2000000");

            sendAt("AT+ISP=1\r\n");    // 启用 ISP
            sendAt("AT+BINN=1\r\n");   // 1x1 = 100x100
            sendAt("AT+FPS=15\r\n");   // 帧率 15
            sendAt("AT+DISP=6\r\n");   // USB+UART 输出
            sendAt("AT+SAVE\r\n");     // 保存

            Thread.sleep(500);

            // 再次清空
            clearBuffer();
            buffer.reset();

            Log.i(TAG, "========== Starting read loop ==========");
            new Thread(this::readLoop).start();

        } catch (Exception e) {
            Log.e(TAG, "Setup failed", e);
        }
    }

    private void clearBuffer() {
        try {
            byte[] tmp = new byte[4096];
            int total = 0;
            int len;
            while ((len = usbSerialPort.read(tmp, 50)) > 0) {
                total += len;
            }
            if (total > 0) {
                Log.i(TAG, "Cleared " + total + " bytes");
            }
        } catch (Exception e) {
            Log.w(TAG, "clearBuffer error");
        }
    }

    private void sendAt(String cmd) {
        try {
            Log.i(TAG, ">>> " + cmd.trim());
            usbSerialPort.write(cmd.getBytes(), 500);
            Thread.sleep(50);
            byte[] resp = new byte[256];
            int len = usbSerialPort.read(resp, 200);
            if (len > 0) {
                Log.i(TAG, "<<< " + bytesToHex(resp, Math.min(len, 32)));
            }
        } catch (Exception e) {
            Log.e(TAG, "AT error: " + cmd.trim());
        }
    }

    private void readLoop() {
        byte[] readBuf = new byte[8192];  // 增大读取缓冲区

        while (running) {
            try {
                int len = usbSerialPort.read(readBuf, 200);  // 缩短超时，更快响应
                if (len > 0) {
                    buffer.write(readBuf, 0, len);
                    parseFrames();
                }
            } catch (Exception e) {
                Log.e(TAG, "Read error", e);
            }
        }
    }

    /**
     * 解析帧 - 使用帧头和长度字段
     */
    private void parseFrames() throws IOException {
        byte[] data = buffer.toByteArray();
        int pos = 0;
        int minHeaderSize = HEADER_SIZE + LENGTH_SIZE;  // 至少需要 4 字节来读取长度

        while (pos + minHeaderSize <= data.length) {
            // 查找帧头 0x00 0xFF
            if ((data[pos] & 0xFF) != HEAD_1 || (data[pos + 1] & 0xFF) != HEAD_2) {
                pos++;
                continue;
            }

            // 读取长度字段 (小端)
            int payloadLen = (data[pos + 2] & 0xFF) | ((data[pos + 3] & 0xFF) << 8);
            int frameSize = HEADER_SIZE + LENGTH_SIZE + payloadLen;

            // 验证长度合理性 (应该在 10018-10020 范围内)
            if (payloadLen < 10000 || payloadLen > 10050) {
                Log.w(TAG, "Invalid length " + payloadLen + " at pos " + pos + ", skip");
                pos++;
                continue;
            }

            // 检查是否有完整帧
            if (pos + frameSize > data.length) {
                Log.d(TAG, "Incomplete frame: need " + frameSize + ", have " + (data.length - pos));
                break;
            }

            Log.i(TAG, "Found header at pos " + pos + ", len=" + payloadLen + ", buffer=" + data.length);

            // 提取像素数据 (跳过 header + length + metadata)
            int pixelStart = pos + HEADER_SIZE + LENGTH_SIZE + META_SIZE;
            byte[] pixels = Arrays.copyOfRange(data, pixelStart, pixelStart + PIXEL_COUNT);

            // 统计
            long sum = 0;
            int min = 255, max = 0;
            for (byte b : pixels) {
                int v = b & 0xFF;
                sum += v;
                if (v < min) min = v;
                if (v > max) max = v;
            }
            int avg = (int) (sum / pixels.length);
            int centerIdx = (HEIGHT / 2) * WIDTH + (WIDTH / 2);
            int center = pixels[centerIdx] & 0xFF;

            frameCounter++;
            Log.i(TAG, "✓✓✓ Frame " + frameCounter +
                    " -> avg=" + avg +
                    " min=" + min +
                    " max=" + max +
                    " center=" + center);

            // 移除已处理的帧
            int newStart = pos + frameSize;
            if (newStart < data.length) {
                byte[] remaining = Arrays.copyOfRange(data, newStart, data.length);
                buffer.reset();
                buffer.write(remaining);
            } else {
                buffer.reset();
            }

            // 重新扫描
            data = buffer.toByteArray();
            pos = 0;
        }

        // 防止 buffer 无限增长
        if (buffer.size() > FRAME_SIZE * 3) {
            int drop = buffer.size() - FRAME_SIZE * 2;
            Log.w(TAG, "Buffer overflow, dropping " + drop + " bytes");
            byte[] remaining = Arrays.copyOfRange(buffer.toByteArray(), drop, buffer.size());
            buffer.reset();
            try { buffer.write(remaining); } catch (Exception ignored) {}
        }
    }

    private String bytesToHex(byte[] data, int len) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < len; i++) {
            sb.append(String.format("%02X ", data[i] & 0xFF));
        }
        return sb.toString();
    }

    @Override
    protected void onDestroy() {
        running = false;
        super.onDestroy();
    }
}
