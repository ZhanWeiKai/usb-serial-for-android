# MaixSense-A010 二次开发手册

> 来源: https://wiki.sipeed.com/hardware/zh/maixsense/maixsense-a010/at_command.html

## AT 指令表

| AT 指令 | 说明 |
| --- | --- |
| +ISP | Image Signal Processor: =0 关闭ISP, =1 开启ISP |
| +BINN | full binning: =1 输出100x100, =2 输出50x50, =4 输出25x25 |
| +DISP | display mux: 显示输出控制 (lcd/usb/uart组合) |
| +BAUD | uart baudrate: 波特率设置 |
| +UNIT | quantization unit: =0 自动, =1-10 按单位(mm)量化 |
| +FPS | frame per second: =1-19 设置帧率 |
| +Save | save config: 保存当前配置 |

---

## ISP 指令

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+ISP? | \r | 返回当前ISP状态 |
| AT+ISP=? | \r | 返回所有支持的ISP状态 |
| AT+ISP=<MODE> | \r | 选择ISP状态 |

**参数：**

| <MODE> | 含义 |
| --- | --- |
| 0 "STOP ISP" | 立即关闭模组ISP，停止IR发射器 |
| 1 "LAUNCH ISP" | 计划启动模组ISP，实际出图需等待1～2秒 |

---

## BINN 指令

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+BINN? | \r | 返回当前BINN状态 |
| AT+BINN=? | \r | 返回所有支持的BINN状态 |
| AT+BINN=<MODE> | \r | 选择BINN状态 |

**参数：**

| <MODE> | 含义 |
| --- | --- |
| 1 "1x1 BINN" | 1x1相当于无binning，实际出图分辨率为100x100 |
| 2 "2x2 BINN" | 2×2 binning，4个像素点合并成1个，实际出图分辨率为50×50 |
| 4 "4x4 BINN" | 4×4 binning，16个像素点合并成1个，实际出图分辨率为25×25 |

---

## DISP 指令

请按需开启，避免资源过度占用

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+DISP? | \r | 返回当前DISP状态 |
| AT+DISP=? | \r | 返回所有支持的DISP状态 |
| AT+DISP=<MODE> | \r | 选择DISP状态 |

**参数：**

| <MODE> | 含义 |
| --- | --- |
| 0 | all off |
| 1 | lcd display on |
| 2 | usb display on |
| 3 | lcd and usb display on |
| 4 | uart display on |
| 5 | lcd and uart display on |
| 6 | usb and uart display on |
| 7 | lcd, usb and uart display on |

---

## BAUD 指令

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+BAUD? | \r | 返回当前BAUD状态 |
| AT+BAUD=? | \r | 返回所有支持的BAUD状态 |
| AT+BAUD=<MODE> | \r | 选择BAUD状态 |

**参数：**

| <MODE> | 含义 |
| --- | --- |
| 0 | 9600 |
| 1 | 57600 |
| 2 | 115200 |
| 3 | 230400 |
| 4 | 460800 |
| 5 | 921600 |
| 6 | 1000000 |
| 7 | 2000000 |
| 8 | 3000000 |

---

## UNIT 指令

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+UNIT? | \r | 返回当前UNIT值 |
| AT+UNIT=? | \r | 返回所有支持的UNIT值 |
| AT+UNIT=<UINT> | \r | 选择UNIT值 |

**参数：**

| <UINT> | 含义 |
| --- | --- |
| 0 "DEFAULT UNIT" | 采用默认量化策略，因tof特性导致成像近处精度优于远距离处，故放大近距离处差异，采用 `5.1*sqrt(x)` 将16bit的原数据量化为8bit |
| 1...9 "QUANTIZE UNIT" | 代表以x mm为单位进行量化，取值越小细节越多，同时可视距离越短，请合理设置 |

---

## FPS 指令

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+FPS? | \r | 返回当前FPS值 |
| AT+FPS=? | \r | 返回所有支持的FPS值 |
| AT+FPS=<FPS> | \r | 选择FPS值 |

**参数：**

| <FPS> | 含义 |
| --- | --- |
| 1...19 "frame per second" | tof出图帧率，越大越流畅 |

---

## SAVE 指令

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+SAVE | \r | 固化TOF摄像头当前配置，事后需要复位 |

> 多机和 AE 指令建议加入

---

## ANTIMMI 指令

**句法：**

| 输入 | 执行 | 注释 |
| --- | --- | --- |
| AT+ANTIMMI? | \r | 返回当前ANTIMMI状态 |
| AT+ANTIMMI=? | \r | 返回所有支持的ANTIMMI状态 |
| AT+ANTIMMI=<MODE> | \r | 选择ANTIMMI状态 |

**参数：**

| <MODE> | 含义 |
| --- | --- |
| -1 | disable anti-mmi |
| 0 | auto anti-mmi |
| 1-41 | manual anti-mmi usb display on |

---

## 图像数据包说明

上电默认启动ISP并在显示屏显示图像，同时输出图像数据到uart和usb

**图像数据封装成包（未稳定）：**

1. **包头 2 字节**：`0x00`、`0xFF`
2. **包长度 2 字节**：当前包剩余数据的字节数
3. **其他内容 16 字节**：包括包序号、包长度、分辨率等等
4. **图像帧**
5. **校验 1 字节**：之前所有字节的"和"低八位
6. **包尾 1 字节**：`0xDD`

---

## 距离计算方法

通过 `AT+UNIT?` 可查询当前UNIT值。

设 **p** 为图像帧各像素值，主要有以下两种情况：

- **若 UNIT 非0**：该像素离TOF距离 = `p` × `UNIT` (单位: mm)
- **若 UNIT 为0**：该像素离TOF距离 = (`p` / 5.1)² (单位: mm)

---

## 代码实现参考

### 解析数据包头

```java
// 包头: 0x00, 0xFF
if (buffer[0] == 0x00 && buffer[1] == (byte)0xFF) {
    // 有效的数据包开始
}
```

### 计算距离值

```java
/**
 * 计算像素点的实际距离
 * @param pixelValue 像素值 (0-255)
 * @param unit 当前UNIT设置值
 * @return 距离 (mm)
 */
public static int calculateDistance(int pixelValue, int unit) {
    if (unit != 0) {
        // UNIT非0: 距离 = p * UNIT
        return pixelValue * unit;
    } else {
        // UNIT为0: 距离 = (p/5.1)^2
        return (int) Math.pow(pixelValue / 5.1, 2);
    }
}
```

### 分辨率计算

```java
/**
 * 根据BINN设置计算实际分辨率
 * @param binn BINN模式 (1, 2, 4)
 * @return 分辨率 (如 100 表示 100x100)
 */
public static int getResolution(int binn) {
    return 100 / binn;  // 1->100, 2->50, 4->25
}
```
