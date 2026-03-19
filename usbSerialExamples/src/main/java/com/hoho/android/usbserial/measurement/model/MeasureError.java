package com.hoho.android.usbserial.measurement.model;

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
        USB_NOT_CONNECTED("USB设备未连接"),
        /** USB权限被拒绝 */
        USB_PERMISSION_DENIED("USB权限被拒绝"),
        /** USB读取失败 */
        USB_READ_FAILED("USB读取失败"),

        // ========== 数据相关 ==========
        /** 无深度数据 */
        NO_DEPTH_DATA("无深度数据"),
        /** 无效深度数据 */
        INVALID_DEPTH_DATA("无效深度数据"),
        /** 深度数据超时 */
        DEPTH_DATA_TIMEOUT("深度数据获取超时"),

        // ========== 校准相关 ==========
        /** 尚未校准 */
        CALIBRATION_NOT_READY("尚未进行基线校准"),
        /** 校准失败 */
        CALIBRATION_FAILED("基线校准失败"),
        /** 正在校准中 */
        CALIBRATION_IN_PROGRESS("正在校准中"),

        // ========== 测量相关 ==========
        /** 未检测到物体 */
        OBJECT_NOT_DETECTED("未检测到物体"),
        /** 物体太小 */
        OBJECT_TOO_SMALL("物体太小，无法测量"),
        /** 正在测量中 */
        MEASUREMENT_IN_PROGRESS("正在测量中"),

        // ========== 状态相关 ==========
        /** 无效状态 */
        INVALID_STATE("无效的操作状态"),
        /** 内部错误 */
        INTERNAL_ERROR("内部错误");

        private final String defaultMessage;

        ErrorCode(String defaultMessage) {
            this.defaultMessage = defaultMessage;
        }

        public String getDefaultMessage() {
            return defaultMessage;
        }
    }

    private MeasureError(ErrorCode code, String message, Throwable cause) {
        this.code = code;
        this.message = message != null ? message : code.getDefaultMessage();
        this.cause = cause;
    }

    /**
     * 是否可恢复
     */
    public boolean isRecoverable() {
        switch (code) {
            case USB_NOT_CONNECTED:
            case USB_PERMISSION_DENIED:
            case USB_READ_FAILED:
            case NO_DEPTH_DATA:
            case INVALID_DEPTH_DATA:
            case DEPTH_DATA_TIMEOUT:
            case OBJECT_NOT_DETECTED:
            case OBJECT_TOO_SMALL:
                return true;
            default:
                return false;
        }
    }

    /**
     * 获取本地化消息
     */
    public String getLocalizedMessage() {
        return message;
    }

    /**
     * 创建错误实例
     */
    public static MeasureError of(ErrorCode code) {
        return new MeasureError(code, null, null);
    }

    /**
     * 创建错误实例
     */
    public static MeasureError of(ErrorCode code, String message) {
        return new MeasureError(code, message, null);
    }

    /**
     * 创建带异常的错误实例
     */
    public static MeasureError of(ErrorCode code, String message, Throwable cause) {
        return new MeasureError(code, message, cause);
    }

    @Override
    public String toString() {
        return "MeasureError{" +
                "code=" + code +
                ", message='" + message + '\'' +
                '}';
    }
}
