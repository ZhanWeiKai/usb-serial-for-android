package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.ByteArrayOutputStream;
import java.util.Arrays;
import java.util.List;

public class MaixSenseActivity extends Activity implements SerialInputOutputManager.Listener {

    private static final String TAG = "MaixSense111";

    private UsbSerialPort usbSerialPort;
    private SerialInputOutputManager usbIoManager;
    private ByteArrayOutputStream bufferStream = new ByteArrayOutputStream();

    private TextView textView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.e(TAG, "on Create MaixSense111");
        super.onCreate(savedInstanceState);
        textView = new TextView(this);
        setContentView(textView);

        connectUsbSerial();
    }

    private void connectUsbSerial() {
        UsbManager usbManager = (UsbManager) getSystemService(USB_SERVICE);
        List<UsbSerialPort> ports = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager).stream()
                .flatMap(driver -> driver.getPorts().stream())
                .toList();

        if (ports.isEmpty()) {
            textView.setText("未找到 USB 串口设备");
            Log.e(TAG, "未找到 USB 串口设备");
            return;
        }

        usbSerialPort = ports.get(0); // 假设只有一个 A010
        UsbDevice device = usbSerialPort.getDriver().getDevice();
        UsbDeviceConnection connection = usbManager.openDevice(device);
        if (connection == null) {
            textView.setText("无法打开 USB 设备权限");
            Log.e(TAG, "无法打开 USB 设备权限");
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

            // 启动循环读取
            usbIoManager = new SerialInputOutputManager(usbSerialPort, this);
            usbIoManager.start();

            // 可选：发送 AT 命令配置 A010
            sendAtCommand("AT+BINN=1\r\n"); // 100x100
            sendAtCommand("AT+UNIT=1\r\n"); // 每像素 1mm
            sendAtCommand("AT+FPS=10\r\n"); // 降低丢帧
            sendAtCommand("AT+DISP=6\r\n");   // USB + UART 输出深度
            sendAtCommand("AT+SAVE\r\n");   // 保存配置

        } catch (Exception e) {
            e.printStackTrace();
            textView.setText("打开串口失败: " + e.getMessage());
            Log.e(TAG, "打开串口失败: " + e.getMessage());
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
        // 收到数据回调
        Log.d(TAG, "收到数据长度: " + data.length);
        byte[] first16 = Arrays.copyOf(data, Math.min(16, data.length));
        Log.d(TAG, "缓存前16字节: " + Arrays.toString(first16));
        try {
            bufferStream.write(data);
            parseFrames();
        } catch (Exception e) {
            e.printStackTrace();
            Log.e(TAG, "数据写入缓存失败: " + e.getMessage());
        }
    }

    @Override
    public void onRunError(Exception e) {
        e.printStackTrace();
        Log.e(TAG, "SerialInputOutputManager 运行出错: " + e.getMessage());
    }

    private void parseFrames() {
        byte[] data = bufferStream.toByteArray();
        int i = 0;
        //Log.d(TAG, "parseFrames..., 缓存长度=" + data.length);

        while (i < data.length - 5) { // 至少 Header(2) + Length(2) + Checksum + Tail
            if (data[i] == 0x00 && data[i + 1] == (byte) 0xFF) {
                // 读取 Length
                int payloadLen = ((data[i + 2] & 0xFF) << 8) | (data[i + 3] & 0xFF);
                int frameEnd = i + 4 + payloadLen + 2; // +2 for checksum + tail

                Log.d(TAG, "找到帧头 i=" + i + ", Payload长度=" + payloadLen + ", 缓存总长=" + data.length);

                // 判断缓存是否有完整帧
                if (data.length >= frameEnd) {
                    byte[] payload = new byte[payloadLen];
                    System.arraycopy(data, i + 4, payload, 0, payloadLen);

                    Log.i(TAG, "解析到完整帧 Payload长度=" + payloadLen);

                    processPayload(payload);

                    // 移除已解析的数据
                    int remainingLen = data.length - frameEnd;
                    byte[] remaining = new byte[remainingLen];
                    System.arraycopy(data, frameEnd, remaining, 0, remainingLen);

                    bufferStream.reset();
                    try {
                        bufferStream.write(remaining);
                    } catch (Exception e) {
                        Log.e(TAG, "写入剩余缓存失败: " + e.getMessage());
                    }

                    Log.d(TAG, "解析后剩余缓存长度=" + bufferStream.size());

                    // 重置索引从头开始寻找下一帧
                    data = bufferStream.toByteArray();
                    i = -1;
                } else {
                    // 帧不完整，等待下一次 onNewData 拼接
                    Log.d(TAG, "帧不完整，等待更多数据");
                    break;
                }
            }
            i++;
        }
    }

    private void processPayload(byte[] payload) {
        int width = 100;
        int height = 100;
        if (payload.length != width * height) {
            Log.w(TAG, "Payload 长度不正确: " + payload.length);
            return;
        }

        int[][] depth = new int[height][width];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                depth[y][x] = payload[y * width + x] & 0xFF;
            }
        }

        Log.d(TAG, "处理一帧 depth map，中心像素值: " + depth[50][50]);
        runOnUiThread(() -> textView.setText("收到一帧深度图，中心像素值: " + depth[50][50]));
    }
}