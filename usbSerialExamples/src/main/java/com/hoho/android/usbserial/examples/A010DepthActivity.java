package com.hoho.android.usbserial.examples;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import java.io.ByteArrayOutputStream;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Executors;

public class A010DepthActivity extends Activity implements SerialInputOutputManager.Listener {

    private static final String TAG = "A010Depth";

    private UsbSerialPort usbSerialPort;
    private SerialInputOutputManager usbIoManager;
    private final ByteArrayOutputStream bufferStream = new ByteArrayOutputStream();

    private ImageView depthView;

    private static final int WIDTH = 100;
    private static final int HEIGHT = 100;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        depthView = new ImageView(this);
        LinearLayout layout = new LinearLayout(this);
        layout.addView(depthView);
        setContentView(layout);

        Log.i(TAG, "Activity created");
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
            Log.e(TAG, "No USB serial device found");
            return;
        }

        usbSerialPort = ports.get(0);
        UsbDevice device = usbSerialPort.getDriver().getDevice();
        UsbDeviceConnection connection = usbManager.openDevice(device);

        if (connection == null) {
            Log.e(TAG, "Failed to open USB device (permission?)");
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
            Log.i(TAG, "USB serial opened");

            // Start read loop
            usbIoManager = new SerialInputOutputManager(usbSerialPort, this);
            usbIoManager.start();

            sendInitialCommands();
        } catch (Exception e) {
            Log.e(TAG, "Serial open failed: " + e.getMessage());
        }
    }

    private void sendInitialCommands() {
        // According to official AT doc -> enable output and correct format
        sendAt("AT+ISP=1\r\n");     // Turn on ISP
        sendAt("AT+BINN=1\r\n");    // 100x100 depth
        sendAt("AT+UNIT=1\r\n");    // mm unit
        sendAt("AT+FPS=10\r\n");    // set frame rate
        sendAt("AT+DISP=6\r\n");    // USB+UART output binary depth
        sendAt("AT+SAVE\r\n");      // optional (store config)
    }

    private void sendAt(String cmd) {
        try {
            usbSerialPort.write(cmd.getBytes(), 500);
            Log.i(TAG, "Sent AT: " + cmd.trim());
        } catch (Exception e) {
            Log.e(TAG, "AT write failed: " + cmd.trim());
        }
    }

    @Override
    public void onNewData(byte[] data) {
        Log.d(TAG, "onNewData length=" + data.length);

        try {
            bufferStream.write(data);
            parseFrames();
        } catch (Exception e) {
            Log.e(TAG, "write buffer fail: " + e.getMessage());
        }
    }

    @Override
    public void onRunError(Exception e) {
        Log.e(TAG, "Serial run error: " + e.getMessage());
    }

    private void parseFrames() {

        byte[] all = bufferStream.toByteArray();
        int i = 0;

        while (i <= all.length - 4) {

            // 查找帧头 00 FF
            if ((all[i] & 0xFF) == 0x00 && (all[i + 1] & 0xFF) == 0xFF) {

                int len = ((all[i + 2] & 0xFF) | ((all[i + 3] & 0xFF) << 8));
                Log.d("FrameAvgDepth:", "Found hdr at i=" + i + " len=" + len);

                int fullFrameSize = 4 + len + 2;
                // 4 = header + len
                // len = payload
                // 2 = checksum + tail

                if (all.length >= i + fullFrameSize) {

                    Log.i("FrameAvgDepth:", "Frame complete");

                    // 取 payload
                    byte[] payload = Arrays.copyOfRange(all, i + 4, i + 4 + len);

                    for (int j = 0; j < 16; j++) {
                        Log.d("FrameAvgDepth", j + " = " + (payload[j] & 0xFF));
                    }

                    final int FRAME_META_LEN = 16;
                    final int pixelLen = WIDTH * HEIGHT; // 10000

                    if (payload.length >= FRAME_META_LEN + pixelLen) {

                        // 跳过 16 字节 metadata
                        byte[] pixelData = Arrays.copyOfRange(
                                payload,
                                FRAME_META_LEN,
                                FRAME_META_LEN + pixelLen
                        );

                        Log.d("FrameAvgDepth:", "pixelData length=" + pixelData.length);

                        // ===== 计算平均深度 =====
                        long sum = 0;
                        int min = 255;
                        int max = 0;

                        for (byte b : pixelData) {

                            int v = b & 0xFF;

                            sum += v;

                            if (v < min) min = v;
                            if (v > max) max = v;
                        }

                        int avg = (int) (sum / pixelData.length);

                        Log.d("FrameAvgDepth:",
                                "Depth stats -> avg=" + avg +
                                        " min=" + min +
                                        " max=" + max
                        );
                        // ===== 显示深度图 =====
                        drawDepth(pixelData);
                    } else {

                        Log.w(TAG, "Payload too short: " + payload.length);
                    }

                    // ===== 移除已经解析的数据 =====

                    int nextStart = i + fullFrameSize;

                    byte[] remaining = Arrays.copyOfRange(
                            all,
                            nextStart,
                            all.length
                    );

                    bufferStream.reset();

                    try {
                        bufferStream.write(remaining);
                    } catch (Exception ignored) {}

                    all = bufferStream.toByteArray();

                    i = -1; // 重新扫描

                } else {

                    Log.d(TAG, "Frame incomplete waiting more");
                    break;
                }
            }

            i++;
        }
    }

    private void drawDepth(byte[] payload) {
        Bitmap bmp = Bitmap.createBitmap(WIDTH, HEIGHT, Bitmap.Config.ARGB_8888);

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                int d = payload[y * WIDTH + x] & 0xFF;
                int c = Color.rgb(d, d, d);
                bmp.setPixel(x, y, c);
            }
        }

        Log.d(TAG, "Display depth center=" + (payload[50*WIDTH+50] & 0xFF));

        //runOnUiThread(() -> depthView.setImageBitmap(bmp));
    }
}
