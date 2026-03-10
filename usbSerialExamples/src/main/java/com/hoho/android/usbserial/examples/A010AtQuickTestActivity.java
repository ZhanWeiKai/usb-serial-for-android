package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.ByteArrayOutputStream;
import java.util.Arrays;
import java.util.List;

public class A010AtQuickTestActivity extends Activity {

    private static final String TAG = "A010AtQuickTest222";
    private UsbSerialPort usbSerialPort;

    private final ByteArrayOutputStream buffer = new ByteArrayOutputStream();
    private int frameCounter = 0;

    private static final int WIDTH = 100;
    private static final int HEIGHT = 100;
    private static final int FRAME_META_LEN = 16;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.i(TAG, "onCreate");

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
            usbSerialPort.open(connection);
            usbSerialPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            Log.i(TAG, "USB serial opened");

            // 发送验证命令
            sendAt("AT+ISP=1\r\n");     // Turn on ISP
            sendAt("AT+BINN=1\r\n");    // 100x100 depth
            //sendAt("AT+UNIT=1\r\n");    // mm unit
            sendAt("AT+FPS=10\r\n");    // frame rate
            sendAt("AT+DISP=6\r\n");    // USB+UART binary depth
            sendAt("AT+SAVE\r\n");      // optional store

        } catch (Exception e) {
            Log.e(TAG, "Serial open failed", e);
        }

        new Thread(this::readLoop).start();
    }

    private void readLoop() {
        byte[] buf = new byte[4096];
        while (true) {
            try {
                int len = usbSerialPort.read(buf, 500);
                if (len > 0) {

                    // ✅ 打印原始收到的字符流，保留 OK 等响应
                    String s = new String(buf, 0, len).trim();
                    if (!s.isEmpty()) Log.i(TAG, "RECV -> " + s);

                    // 写入 buffer 用于解析 frame
                    buffer.write(buf, 0, len);

                    // 尝试解析 frame
                    parseFrames();
                }
            } catch (Exception e) {
                Log.e(TAG, "Read error", e);
                break;
            }
        }
    }

    private void parseFrames() {
        byte[] all = buffer.toByteArray();
        int i = 0;

        while (i <= all.length - 4) {
            // 查找帧头 0x00 0xFF
            if ((all[i] & 0xFF) == 0x00 && (all[i + 1] & 0xFF) == 0xFF) {

                int len = (all[i + 2] & 0xFF) | ((all[i + 3] & 0xFF) << 8); // 剩余数据字节数
                int fullFrameSize = 4 + len; // 文档中 len 已经包含 payload+checksum+tail

                if (all.length >= i + fullFrameSize) {
                    byte[] payload = Arrays.copyOfRange(all, i + 4, i + 4 + len);

                    // 长度检查
                    if (payload.length >= FRAME_META_LEN + WIDTH * HEIGHT + 2) {
                        // 校验和 & 包尾
                        int sum = 0;
                        for (int k = 0; k < payload.length - 2; k++) {
                            sum += payload[k] & 0xFF;
                        }
                        sum &= 0xFF;
                        int checksum = payload[payload.length - 2] & 0xFF;
                        int tail = payload[payload.length - 1] & 0xFF;

                        if (sum != checksum || tail != 0xDD) {
                            Log.w(TAG, "Frame checksum/tail mismatch: checksum=" + checksum + " calc=" + sum + " tail=" + tail);
                            // 丢弃此帧
                            i += fullFrameSize;
                            continue;
                        }

                        // frameId 从 payload 前16字节第0-3字节或其他字段获取（按文档）
                        int frameId = ((payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8));

                        // 取像素数据
                        byte[] pixels = Arrays.copyOfRange(payload, FRAME_META_LEN, FRAME_META_LEN + WIDTH * HEIGHT);

                        // 统计
                        long total = 0;
                        int min = 255, max = 0;
                        for (byte b : pixels) {
                            int v = b & 0xFF;
                            total += v;
                            if (v < min) min = v;
                            if (v > max) max = v;
                        }
                        int avg = (int) (total / pixels.length);
                        int center = pixels[(HEIGHT / 2) * WIDTH + (WIDTH / 2)] & 0xFF;

                        Log.i(TAG, "Frame " + frameId + " -> avg=" + avg + " min=" + min + " max=" + max + " center=" + center);

                        // 随机打印几个像素调试
                        for (int j = 0; j < 10; j++) {
                            int idx = (int) (Math.random() * pixels.length);
                            Log.d(TAG, "Pixel[" + idx + "]=" + (pixels[idx] & 0xFF));
                        }
                    }

                    // 移除已解析数据
                    byte[] remaining = Arrays.copyOfRange(all, i + fullFrameSize, all.length);
                    buffer.reset();
                    try { buffer.write(remaining); } catch (Exception ignored) {}
                    all = buffer.toByteArray();
                    i = -1; // 重头扫描
                } else {
                    break; // 帧不完整，等待更多数据
                }
            }
            i++;
        }
    }

    private void sendAt(String cmd) {
        try {
            usbSerialPort.write(cmd.getBytes(), 500);
            Log.i(TAG, "Sent AT: " + cmd.trim());
        } catch (Exception e) {
            Log.e(TAG, "AT write failed: " + cmd.trim(), e);
        }
    }
}