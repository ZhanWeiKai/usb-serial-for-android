package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import java.io.ByteArrayOutputStream;
import java.util.List;

public class MaixSenseDisplayActivity extends Activity implements SerialInputOutputManager.Listener {

    private static final String TAG = "MaixSense";

    private UsbSerialPort usbSerialPort;
    private SerialInputOutputManager usbIoManager;
    private ByteArrayOutputStream bufferStream = new ByteArrayOutputStream();
    private ImageView imageView;

    private static final int WIDTH = 100;
    private static final int HEIGHT = 100;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        imageView = new ImageView(this);
        setContentView(imageView);

        Log.e(TAG, "onCreate MaixSenseDisplayActivity");

        connectUsbSerial();
    }

    private void connectUsbSerial() {
        UsbManager usbManager = (UsbManager) getSystemService(USB_SERVICE);
        List<UsbSerialPort> ports = UsbSerialProber.getDefaultProber()
                .findAllDrivers(usbManager)
                .stream()
                .flatMap(driver -> driver.getPorts().stream())
                .toList();

        if (ports.isEmpty()) {
            Log.e(TAG, "未找到 USB 设备");
            return;
        }

        usbSerialPort = ports.get(0);
        UsbDevice device = usbSerialPort.getDriver().getDevice();
        UsbDeviceConnection connection = usbManager.openDevice(device);
        if (connection == null) {
            Log.e(TAG, "无法打开 USB 权限");
            return;
        }

        try {
            usbSerialPort.open(connection);
            usbSerialPort.setParameters(
                    115200,
                    8,
                    UsbSerialPort.STOPBITS_1,
                    UsbSerialPort.PARITY_NONE
            );
            Log.i(TAG, "串口打开成功");

            usbIoManager = new SerialInputOutputManager(usbSerialPort, this);
            usbIoManager.start();

            // 发 AT 命令保证连续输出
            sendAtCommand("AT+BINN=1\r\n"); // 100x100
            sendAtCommand("AT+UNIT=1\r\n"); // mm
            sendAtCommand("AT+FPS=10\r\n"); // 连续输出帧率
            sendAtCommand("AT+SAVE\r\n");   // 保存配置

        } catch (Exception e) {
            e.printStackTrace();
            Log.e(TAG, "串口打开失败: " + e.getMessage());
        }
    }

    private void sendAtCommand(String cmd) {
        try {
            usbSerialPort.write(cmd.getBytes(), 1000);
            Log.i(TAG, "发送 AT 命令: " + cmd.trim());
        } catch (Exception e) {
            e.printStackTrace();
            Log.e(TAG, "发送 AT 命令失败: " + cmd.trim());
        }
    }

    @Override
    public void onNewData(byte[] data) {
        Log.d(TAG, "收到数据长度: " + data.length);
        try {
            bufferStream.write(data);
            parseFrames();
        } catch (Exception e) {
            Log.e(TAG, "写入缓存失败: " + e.getMessage());
        }
    }

    @Override
    public void onRunError(Exception e) {
        Log.e(TAG, "SerialInputOutputManager 错误: " + e.getMessage());
    }

    private void parseFrames() {
        byte[] data = bufferStream.toByteArray();
        int i = 0;

        while (i < data.length - 5) {
            if (data[i] == 0x00 && data[i + 1] == (byte) 0xFF) {
                int payloadLen = ((data[i + 2] & 0xFF) << 8) | (data[i + 3] & 0xFF);
                int frameEnd = i + 4 + payloadLen + 2;

                Log.d(TAG, "找到帧头 i=" + i + ", Payload长度=" + payloadLen + ", 缓存长度=" + data.length);

                if (data.length >= frameEnd) {
                    byte[] payload = new byte[payloadLen];
                    System.arraycopy(data, i + 4, payload, 0, payloadLen);

                    Log.i(TAG, "解析到完整帧 Payload长度=" + payloadLen);

                    processPayload(payload);

                    // 移除已解析数据
                    int remainingLen = data.length - frameEnd;
                    byte[] remaining = new byte[remainingLen];
                    System.arraycopy(data, frameEnd, remaining, 0, remainingLen);
                    bufferStream.reset();
                    try { bufferStream.write(remaining); } catch (Exception ignored) {}
                    data = bufferStream.toByteArray();
                    i = -1;
                } else {
                    Log.d(TAG, "帧不完整，等待更多数据");
                    break;
                }
            }
            i++;
        }
    }

    private void processPayload(byte[] payload) {
        if (payload.length != WIDTH * HEIGHT) {
            Log.w(TAG, "Payload 长度不对: " + payload.length);
            return;
        }

        Bitmap bmp = Bitmap.createBitmap(WIDTH, HEIGHT, Bitmap.Config.ARGB_8888);

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                int v = payload[y * WIDTH + x] & 0xFF;
                int color = Color.rgb(v, v, v); // 灰度显示
                bmp.setPixel(x, y, color);
            }
        }

        Log.d(TAG, "显示一帧 depth map, 中心像素值=" + (payload[50 * WIDTH + 50] & 0xFF));

        runOnUiThread(() -> imageView.setImageBitmap(bmp));
    }
}
