package com.hoho.android.usbserial.examples;

import android.content.Context;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

/**
 * 深度数据存储工具类
 *
 * 用于保存和加载基线深度数据 (BaselineDepth)
 * 使用JSON格式存储到应用内部存储目录
 */
public class DepthDataStorage {

    private static final String TAG = "DepthDataStorage";
    private static final String FILENAME = "baseline_depth.json";

    private final Context context;

    public DepthDataStorage(Context context) {
        this.context = context.getApplicationContext();
    }

    /**
     * 保存基线深度数据到文件
     *
     * @param depth 100x100 的深度数据数组，单位: mm
     * @return 是否保存成功
     */
    public boolean saveBaseline(float[][] depth) {
        if (depth == null || depth.length == 0) {
            Log.e(TAG, "Depth data is null or empty");
            return false;
        }

        try {
            JSONObject root = new JSONObject();
            root.put("version", 1);
            root.put("width", depth.length);
            root.put("height", depth[0].length);
            root.put("timestamp", System.currentTimeMillis());

            // 将二维数组转为一维数组存储
            JSONArray depthArray = new JSONArray();
            for (int x = 0; x < depth.length; x++) {
                for (int y = 0; y < depth[x].length; y++) {
                    depthArray.put(depth[x][y]);
                }
            }
            root.put("data", depthArray);

            // 写入文件
            String jsonStr = root.toString();
            File file = new File(context.getFilesDir(), FILENAME);
            try (FileOutputStream fos = new FileOutputStream(file)) {
                fos.write(jsonStr.getBytes(StandardCharsets.UTF_8));
                fos.flush();
            }

            Log.i(TAG, "Baseline saved successfully, size: " + depth.length + "x" + depth[0].length);
            return true;

        } catch (Exception e) {
            Log.e(TAG, "Failed to save baseline", e);
            return false;
        }
    }

    /**
     * 从文件加载基线深度数据
     *
     * @return 100x100 的深度数据数组，单位: mm；如果加载失败返回 null
     */
    public float[][] loadBaseline() {
        File file = new File(context.getFilesDir(), FILENAME);
        if (!file.exists()) {
            Log.w(TAG, "Baseline file does not exist");
            return null;
        }

        try {
            // 读取文件内容
            StringBuilder sb = new StringBuilder();
            try (FileInputStream fis = new FileInputStream(file)) {
                byte[] buffer = new byte[4096];
                int len;
                while ((len = fis.read(buffer)) > 0) {
                    sb.append(new String(buffer, 0, len, StandardCharsets.UTF_8));
                }
            }

            // 解析JSON
            JSONObject root = new JSONObject(sb.toString());
            int version = root.getInt("version");
            int width = root.getInt("width");
            int height = root.getInt("height");
            JSONArray depthArray = root.getJSONArray("data");

            // 验证数据大小
            if (depthArray.length() != width * height) {
                Log.e(TAG, "Data size mismatch: expected " + (width * height) + ", got " + depthArray.length());
                return null;
            }

            // 转换为二维数组
            float[][] depth = new float[width][height];
            int idx = 0;
            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    depth[x][y] = (float) depthArray.getDouble(idx++);
                }
            }

            Log.i(TAG, "Baseline loaded successfully, size: " + width + "x" + height);
            return depth;

        } catch (Exception e) {
            Log.e(TAG, "Failed to load baseline", e);
            return null;
        }
    }

    /**
     * 检查是否存在已保存的基线数据
     *
     * @return 是否存在基线数据
     */
    public boolean hasBaseline() {
        File file = new File(context.getFilesDir(), FILENAME);
        return file.exists();
    }

    /**
     * 删除已保存的基线数据
     *
     * @return 是否删除成功
     */
    public boolean deleteBaseline() {
        File file = new File(context.getFilesDir(), FILENAME);
        if (file.exists()) {
            boolean deleted = file.delete();
            if (deleted) {
                Log.i(TAG, "Baseline deleted successfully");
            }
            return deleted;
        }
        return true;
    }

    /**
     * 获取基线数据的保存时间
     *
     * @return 保存时间戳，如果不存在返回 -1
     */
    public long getBaselineTimestamp() {
        File file = new File(context.getFilesDir(), FILENAME);
        if (!file.exists()) {
            return -1;
        }

        try {
            StringBuilder sb = new StringBuilder();
            try (FileInputStream fis = new FileInputStream(file)) {
                byte[] buffer = new byte[4096];
                int len;
                while ((len = fis.read(buffer)) > 0) {
                    sb.append(new String(buffer, 0, len, StandardCharsets.UTF_8));
                }
            }

            JSONObject root = new JSONObject(sb.toString());
            return root.getLong("timestamp");

        } catch (Exception e) {
            Log.e(TAG, "Failed to get baseline timestamp", e);
            return -1;
        }
    }
}
