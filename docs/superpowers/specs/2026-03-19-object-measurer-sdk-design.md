# ObjectMeasurer SDK 设计文档

## 1. 概述

### 1.1 背景

当前项目中的物体尺寸测量功能分散在 `ObjectMeasurementActivity` 和 `ObjectDimensionCalculator` 中，代码与UI强耦合，难以复用。

### 1.2 目标

将测量功能封装成独立的、可复用的模块，提供简洁的API接口：
- 获取深度数据
- 校准基线
- 测量物体尺寸（单次/连续）

### 1.3 范围

- **内部模块**：供当前项目内部使用，不发布为独立库
- **异步回调**：采用回调模式返回结果
- **支持两种测量模式**：单次测量和连续测量

---

## 2. 架构设计

### 2.1 类图

```
┌─────────────────────────────────────────────────────────────────┐
│                        ObjectMeasurer                            │
│  (主入口类)                                                      │
├─────────────────────────────────────────────────────────────────┤
│  - Builder                                                      │
│  - start() / stop() / release()                                │
│  - getDepthData() / getDepthDataContinuous()                   │
│  - calibrateBaseline()                                          │
│  - measureOnce() / startContinuousMeasure() / stopMeasure()    │
│  - getState() / hasBaseline() / getBaseline()                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    内部组件 (复用现有代码)                         │
├─────────────────────────────────────────────────────────────────┤
│  ObjectDimensionCalculator  - 尺寸计算核心                       │
│  DepthDataStorage           - 数据持久化                         │
│  FrameBuffer                - 帧缓冲                             │
│  UsbSerialPort              - USB串口通信                        │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 数据流

```
USB串口 ──► 帧解析 ──► 深度矩阵 ──► 中值滤波 ──► 业务处理
                                                          │
                          ┌───────────────────────────────┼───────────────────────────┐
                          │                               │                           │
                          ▼                               ▼                           ▼
                    深度数据回调                    校准流程                      测量流程
                    (DepthCallback)            (CalibrationCallback)       (MeasureCallback)
```

---

## 3. API 设计

### 3.1 ObjectMeasurer 主类

```java
package com.hoho.android.usbserial.measurement;

/**
 * 物体尺寸测量器
 *
 * 使用流程：
 * 1. 创建实例并配置
 * 2. 调用 start() 开始接收数据
 * 3. 调用 calibrateBaseline() 校准基线
 * 4. 调用 measureOnce() 或 startContinuousMeasure() 测量物体
 * 5. 调用 stop() / release() 释放资源
 */
public class ObjectMeasurer {

    // ==================== 构建器 ====================

    /**
     * 创建 Builder
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * 配置构建器
     */
    public static class Builder {
        /**
         * 设置USB串口（必需）
         */
        public Builder setUsbSerialPort(UsbSerialPort port);

        /**
         * 设置视场角（默认 70°×60°，适用于A010）
         */
        public Builder setFov(float horizontalDeg, float verticalDeg);

        /**
         * 设置俯视角度（默认 45°）
         */
        public Builder setTiltAngle(float angleDeg);

        /**
         * 设置物体检测阈值（默认 50mm）
         */
        public Builder setThreshold(float thresholdMm);

        /**
         * 设置采样窗口大小（默认 10帧/500ms）
         */
        public Builder setSampleWindow(int samplesPerWindow, long intervalMs);

        /**
         * 设置稳定帧数要求（默认 10帧）
         */
        public Builder setStableCount(int count);

        /**
         * 构建实例
         */
        public ObjectMeasurer build();
    }

    // ==================== 生命周期 ====================

    /**
     * 开始接收数据
     * 必须在调用其他方法前调用
     */
    public void start();

    /**
     * 停止接收数据
     * 可以重新调用 start() 恢复
     */
    public void stop();

    /**
     * 释放所有资源
     * 调用后实例不可再使用
     */
    public void release();

    // ==================== 深度数据 ====================

    /**
     * 单次获取深度数据
     */
    public void getDepthData(DepthCallback callback);

    /**
     * 连续获取深度数据
     * 需要调用 stopDepthData() 停止
     */
    public void getDepthDataContinuous(ContinuousDepthCallback callback);

    /**
     * 停止连续获取深度数据
     */
    public void stopDepthData();

    // ==================== 校准 ====================

    /**
     * 开始基线校准
     * 校准前请确保桌面清洁、无物体
     */
    public void calibrateBaseline(CalibrationCallback callback);

    /**
     * 是否已有基线数据
     */
    public boolean hasBaseline();

    /**
     * 获取当前基线数据
     * @return 基线结果，如果没有返回 null
     */
    public BaselineResult getBaseline();

    /**
     * 清除基线数据
     */
    public void clearBaseline();

    // ==================== 测量 ====================

    /**
     * 单次测量
     * 连续10帧稳定后返回结果
     */
    public void measureOnce(MeasureCallback callback);

    /**
     * 开始连续测量
     * 每次稳定后都会回调结果
     */
    public void startContinuousMeasure(ContinuousMeasureCallback callback);

    /**
     * 停止测量
     */
    public void stopMeasure();

    // ==================== 状态 ====================

    /**
     * 获取当前状态
     */
    public State getState();

    /**
     * 状态枚举
     */
    public enum State {
        IDLE,           // 空闲
        CALIBRATING,    // 校准中
        CALIBRATED,     // 已校准
        MEASURING,      // 测量中
        ERROR           // 错误
    }
}
```

### 3.2 回调接口

```java
package com.hoho.android.usbserial.measurement;

/**
 * 深度数据回调（单次）
 */
public interface DepthCallback {
    /**
     * 收到深度数据
     * @param depthMatrix 深度矩阵 [100][100]，单位 mm
     * @param timestamp 时间戳
     */
    void onDepthData(float[][] depthMatrix, long timestamp);

    /**
     * 发生错误
     */
    void onError(MeasureError error);
}

/**
 * 深度数据回调（连续）
 */
public interface ContinuousDepthCallback {
    void onDepthData(float[][] depthMatrix, long timestamp);
    void onError(MeasureError error);
}

/**
 * 校准回调
 */
public interface CalibrationCallback {
    /**
     * 进度更新
     * @param current 当前稳定帧数
     * @param total 需要的总帧数
     * @param avgDifference 帧间平均差异 (mm)
     */
    void onProgress(int current, int total, float avgDifference);

    /**
     * 校准成功
     */
    void onSuccess(BaselineResult result);

    /**
     * 校准失败
     */
    void onError(MeasureError error);
}

/**
 * 单次测量回调
 */
public interface MeasureCallback {
    /**
     * 进度更新
     * @param current 当前稳定帧数
     * @param total 需要的总帧数
     * @param intermediate 中间结果
     */
    void onProgress(int current, int total, MeasureResult intermediate);

    /**
     * 测量成功
     */
    void onSuccess(MeasureResult result);

    /**
     * 测量失败
     */
    void onError(MeasureError error);
}

/**
 * 连续测量回调
 */
public interface ContinuousMeasureCallback {
    /**
     * 测量结果
     * 每次稳定后回调
     */
    void onResult(MeasureResult result);

    /**
     * 状态变化
     * 如阈值调整、锁定等
     */
    void onStatusChanged(MeasureStatus status);

    /**
     * 发生错误
     */
    void onError(MeasureError error);
}
```

### 3.3 结果数据类

```java
package com.hoho.android.usbserial.measurement;

/**
 * 基线校准结果
 */
public class BaselineResult {
    /** 基线深度矩阵 [100][100]，单位 mm */
    public final float[][] baselineDepth;

    /** 中心点深度 (mm) */
    public final float centerDepth;

    /** X方向每像素对应的物理尺寸 (mm/像素) */
    public final float pixelSizeX;

    /** Y方向每像素对应的物理尺寸 (mm/像素) */
    public final float pixelSizeY;

    /** 采样帧数 */
    public final int frameCount;

    /** 时间戳 */
    public final long timestamp;

    /**
     * 获取视野宽度 (mm)
     */
    public float getViewWidthMm();

    /**
     * 获取视野高度 (mm)
     */
    public float getViewHeightMm();

    /**
     * 获取视野宽度 (cm)
     */
    public float getViewWidthCm();

    /**
     * 获取视野高度 (cm)
     */
    public float getViewHeightCm();
}

/**
 * 物体测量结果
 */
public class MeasureResult {
    // ========== 核心结果 ==========

    /** 宽度 (mm) */
    public final float width;

    /** 长度 (mm) */
    public final float length;

    /** 高度 (mm) */
    public final float height;

    /** 体积 (cm³) */
    public final float volumeCm3;

    // ========== 质量指标 ==========

    /** 原始 mask 像素数 */
    public final int rawPixelCount;

    /** 有效像素数 */
    public final int validPixelCount;

    /** 最大深度差 (mm) */
    public final float maxDepthDiff;

    /** 物体表面深度 (mm) */
    public final float objectSurfaceDepth;

    // ========== 边界范围 ==========

    /** X方向最小像素索引 */
    public final int xMin;

    /** X方向最大像素索引 */
    public final int xMax;

    /** Y方向最小像素索引 */
    public final int yMin;

    /** Y方向最大像素索引 */
    public final int yMax;

    // ========== 状态信息 ==========

    /** 是否已稳定（连续10帧） */
    public final boolean isStable;

    /** 稳定帧计数 */
    public final int stableCount;

    /** 状态消息 */
    public final String message;

    // ========== 工具方法 ==========

    /** 宽度 (cm) */
    public float getWidthCm();

    /** 长度 (cm) */
    public float getLengthCm();

    /** 高度 (cm) */
    public float getHeightCm();

    /**
     * 格式化输出
     * @return 格式化的测量结果字符串
     */
    public String getFormattedResult();
}

/**
 * 测量状态描述
 */
public class MeasureStatus {
    /** 当前阈值 (mm) */
    public final float currentThreshold;

    /** 阈值是否已锁定 */
    public final boolean thresholdLocked;

    /** 不稳定计数 */
    public final int unstableCount;

    /** 稳定计数 */
    public final int stableCount;

    /** 状态描述 */
    public final String message;
}
```

### 3.4 错误处理

```java
package com.hoho.android.usbserial.measurement;

/**
 * 测量错误
 */
public class MeasureError {
    /** 错误码 */
    public final ErrorCode code;

    /** 错误消息 */
    public final String message;

    /** 原始异常（可选） */
    public final Throwable cause;

    /**
     * 错误码枚举
     */
    public enum ErrorCode {
        // ========== 设备相关 ==========
        /** USB未连接 */
        USB_NOT_CONNECTED,
        /** USB权限被拒绝 */
        USB_PERMISSION_DENIED,
        /** USB读取失败 */
        USB_READ_FAILED,

        // ========== 数据相关 ==========
        /** 无深度数据 */
        NO_DEPTH_DATA,
        /** 无效深度数据 */
        INVALID_DEPTH_DATA,
        /** 深度数据超时 */
        DEPTH_DATA_TIMEOUT,

        // ========== 校准相关 ==========
        /** 尚未校准 */
        CALIBRATION_NOT_READY,
        /** 校准失败 */
        CALIBRATION_FAILED,
        /** 正在校准中 */
        CALIBRATION_IN_PROGRESS,

        // ========== 测量相关 ==========
        /** 未检测到物体 */
        OBJECT_NOT_DETECTED,
        /** 物体太小 */
        OBJECT_TOO_SMALL,
        /** 正在测量中 */
        MEASUREMENT_IN_PROGRESS,

        // ========== 状态相关 ==========
        /** 无效状态 */
        INVALID_STATE,
        /** 内部错误 */
        INTERNAL_ERROR
    }

    /**
     * 是否可恢复
     */
    public boolean isRecoverable();

    /**
     * 获取本地化消息
     */
    public String getLocalizedMessage();

    /**
     * 创建错误实例
     */
    public static MeasureError of(ErrorCode code, String message);

    /**
     * 创建带异常的错误实例
     */
    public static MeasureError of(ErrorCode code, String message, Throwable cause);
}
```

---

## 4. 内部实现

### 4.1 核心组件

| 组件 | 职责 | 复用来源 |
|------|------|----------|
| `ObjectDimensionCalculator` | 尺寸计算核心算法 | 现有代码 |
| `DepthDataStorage` | 基线数据持久化 | 现有代码 |
| `FrameBuffer` | 多帧缓冲 | 现有代码 |
| `DepthFrameParser` | 帧解析（新增） | 从Activity中提取 |
| `MedianFilter` | 中值滤波（新增） | 从Activity中提取 |
| `StabilityDetector` | 稳定性检测（新增） | 从Activity中提取 |
| `DynamicThresholdManager` | 动态阈值管理（新增） | 从Activity中提取 |

### 4.2 类结构

```
com.hoho.android.usbserial.measurement/
├── ObjectMeasurer.java              # 主入口
├── ObjectMeasurerConfig.java        # 配置类
├── callback/
│   ├── DepthCallback.java
│   ├── ContinuousDepthCallback.java
│   ├── CalibrationCallback.java
│   ├── MeasureCallback.java
│   └── ContinuousMeasureCallback.java
├── model/
│   ├── BaselineResult.java
│   ├── MeasureResult.java
│   ├── MeasureStatus.java
│   └── MeasureError.java
└── internal/
    ├── DepthFrameParser.java        # 帧解析
    ├── MedianFilter.java            # 中值滤波
    ├── StabilityDetector.java       # 稳定性检测
    ├── DynamicThresholdManager.java # 动态阈值
    └── MeasurementStateMachine.java # 状态机
```

### 4.3 状态机

```
         ┌──────────────────────────────────────┐
         │                                      │
         ▼                                      │
    ┌─────────┐  calibrate()  ┌────────────┐   │
    │  IDLE   │ ─────────────►│CALIBRATING │   │
    └─────────┘               └─────┬──────┘   │
         ▲                          │          │
         │                    success│         │
         │                          ▼          │
         │                    ┌────────────┐   │
         │                    │ CALIBRATED │   │
         │                    └─────┬──────┘   │
         │                          │          │
         │     stopMeasure()   measure()│      │
         │     ◄──────────────────────┘│      │
         │                          ▼          │
         │                    ┌────────────┐   │
         └────────────────────│ MEASURING  │───┘
           error/clearBaseline └────────────┘
```

---

## 5. 使用示例

### 5.1 初始化

```java
// 创建实例
ObjectMeasurer measurer = ObjectMeasurer.builder()
    .setUsbSerialPort(usbPort)
    .setFov(70f, 60f)
    .setTiltAngle(45f)
    .setThreshold(50f)
    .build();

// 开始接收数据
measurer.start();
```

### 5.2 获取深度数据

```java
// 单次获取
measurer.getDepthData(new DepthCallback() {
    @Override
    public void onDepthData(float[][] depthMatrix, long timestamp) {
        float centerDepth = depthMatrix[50][50];
        Log.d("Depth", "中心深度: " + centerDepth + " mm");
    }

    @Override
    public void onError(MeasureError error) {
        Log.e("Depth", error.message);
    }
});

// 连续获取
measurer.getDepthDataContinuous(new ContinuousDepthCallback() {
    @Override
    public void onDepthData(float[][] depthMatrix, long timestamp) {
        updateVisualization(depthMatrix);
    }

    @Override
    public void onError(MeasureError error) {
        Log.e("Depth", error.message);
    }
});

// 停止连续获取
measurer.stopDepthData();
```

### 5.3 基线校准

```java
measurer.calibrateBaseline(new CalibrationCallback() {
    @Override
    public void onProgress(int current, int total, float avgDifference) {
        progressBar.setProgress(current * 100 / total);
    }

    @Override
    public void onSuccess(BaselineResult result) {
        Log.d("Calibration", String.format(
            "校准完成! 视野: %.1f×%.1f cm, 像素: %.2f×%.2f mm",
            result.getViewWidthCm(), result.getViewHeightCm(),
            result.pixelSizeX, result.pixelSizeY
        ));
    }

    @Override
    public void onError(MeasureError error) {
        Log.e("Calibration", error.getLocalizedMessage());
    }
});
```

### 5.4 单次测量

```java
measurer.measureOnce(new MeasureCallback() {
    @Override
    public void onProgress(int current, int total, MeasureResult intermediate) {
        progressBar.setProgress(current * 100 / total);
    }

    @Override
    public void onSuccess(MeasureResult result) {
        Log.d("Measure", result.getFormattedResult());
    }

    @Override
    public void onError(MeasureError error) {
        if (error.code == MeasureError.ErrorCode.OBJECT_NOT_DETECTED) {
            Log.w("Measure", "未检测到物体");
        } else {
            Log.e("Measure", error.message);
        }
    }
});
```

### 5.5 连续测量

```java
measurer.startContinuousMeasure(new ContinuousMeasureCallback() {
    @Override
    public void onResult(MeasureResult result) {
        textView.setText(result.getFormattedResult());
    }

    @Override
    public void onStatusChanged(MeasureStatus status) {
        statusView.setText(status.message);
    }

    @Override
    public void onError(MeasureError error) {
        // 处理错误
    }
});

// 停止测量
measurer.stopMeasure();
```

### 5.6 完整Activity示例

```java
public class MeasurementActivity extends Activity {

    private ObjectMeasurer measurer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_measurement);

        UsbSerialPort usbPort = getConnectedUsbPort();

        measurer = ObjectMeasurer.builder()
            .setUsbSerialPort(usbPort)
            .setFov(70f, 60f)
            .build();

        measurer.start();

        // 校准
        findViewById(R.id.btnCalibrate).setOnClickListener(v -> {
            measurer.calibrateBaseline(new CalibrationCallback() {
                @Override
                public void onProgress(int current, int total, float avgDifference) {
                    updateProgress(current, total);
                }

                @Override
                public void onSuccess(BaselineResult result) {
                    showResult("校准完成: " + result.getViewWidthCm() + "×" + result.getViewHeightCm() + " cm");
                    enableMeasureButtons(true);
                }

                @Override
                public void onError(MeasureError error) {
                    showError(error.getLocalizedMessage());
                }
            });
        });

        // 单次测量
        findViewById(R.id.btnMeasure).setOnClickListener(v -> {
            measurer.measureOnce(new MeasureCallback() {
                @Override
                public void onProgress(int current, int total, MeasureResult intermediate) {
                    updateProgress(current, total);
                }

                @Override
                public void onSuccess(MeasureResult result) {
                    showResult(result.getFormattedResult());
                }

                @Override
                public void onError(MeasureError error) {
                    showError(error.getLocalizedMessage());
                }
            });
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (measurer != null) {
            measurer.stop();
            measurer.release();
        }
    }

    private void updateProgress(int current, int total) {
        runOnUiThread(() -> {
            ProgressBar pb = findViewById(R.id.progressBar);
            pb.setMax(total);
            pb.setProgress(current);
        });
    }

    private void showResult(String result) {
        runOnUiThread(() -> {
            ((TextView) findViewById(R.id.textResult)).setText(result);
        });
    }

    private void showError(String error) {
        runOnUiThread(() -> {
            Toast.makeText(this, error, Toast.LENGTH_SHORT).show();
        });
    }

    private void enableMeasureButtons(boolean enabled) {
        runOnUiThread(() -> {
            findViewById(R.id.btnMeasure).setEnabled(enabled);
        });
    }
}
```

---

## 6. 默认参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| FOV_HORIZONTAL | 70° | 水平视场角 |
| FOV_VERTICAL | 60° | 垂直视场角 |
| TILT_ANGLE | 45° | 俯视角度 |
| OBJECT_THRESHOLD | 50mm | 物体检测阈值 |
| MAX_THRESHOLD | 100mm | 阈值上限 |
| SAMPLES_PER_WINDOW | 10 | 每窗口采样帧数 |
| SAMPLE_INTERVAL | 500ms | 窗口间隔 |
| STABLE_COUNT | 10 | 稳定帧数要求 |
| DIMENSION_THRESHOLD | 20mm | 尺寸偏差阈值 |
| MIN_CONSECUTIVE_PIXELS | 8 | 最小连续像素数 |

---

## 7. 迁移计划

### 7.1 第一阶段：创建新模块

1. 创建 `com.hoho.android.usbserial.measurement` 包
2. 实现核心类和接口
3. 复用 `ObjectDimensionCalculator` 等现有组件

### 7.2 第二阶段：适配现有代码

1. 重构 `ObjectMeasurementActivity`，内部使用 `ObjectMeasurer`
2. 保留原有UI和交互逻辑
3. 验证功能一致性

### 7.3 第三阶段：清理

1. 移除 `ObjectMeasurementActivity` 中的冗余代码
2. 更新文档

---

## 8. 文件清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `ObjectMeasurer.java` | 主类 | 测量器主入口 |
| `ObjectMeasurerConfig.java` | 配置 | 配置参数封装 |
| `callback/*.java` | 接口 | 各类回调接口 |
| `model/*.java` | 数据类 | 结果、错误等数据类 |
| `internal/DepthFrameParser.java` | 内部 | 帧解析 |
| `internal/MedianFilter.java` | 内部 | 中值滤波 |
| `internal/StabilityDetector.java` | 内部 | 稳定性检测 |
| `internal/DynamicThresholdManager.java` | 内部 | 动态阈值管理 |
| `internal/MeasurementStateMachine.java` | 内部 | 状态机 |

---

## 9. 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| 1.0 | 2026-03-19 | 初始设计 |
