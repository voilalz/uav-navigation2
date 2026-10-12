# STM32H743硬件移植

本仓库已在宿主机运行传感器解析、时间/队列和任务运行时测试；`nav_stm32_port`是可链接到真实CubeMX BSP的FreeRTOS移植库。没有PCB引脚、厂商HAL/CMSIS/FreeRTOS包、启动文件、链接脚本、实物和相机SDK，不能把宿主构建称为可烧录或已通过实机验证的固件。

## 已实现的驱动与时间接口

| 模块 | 实际行为 | 板级输入 |
|---|---|---|
| ADIS16470 | SPI模式3，最大1MHz，0x6800命令+20字节MSB响应；逐字节和校验；诊断拒绝；有符号16位缩放；DATA_CNTR回绕/重复/丢样统计；PROD_ID和配置回读 | 四路独立SPI/CS、DR捕获、2000Hz SYNC、复位和上电等待、DMA完成数据 |
| RM3100 | 模式0配置cycle count/TMRC/CMM，DRDY状态轮询，MSB优先signed24 | SPI、200 cycle典型75 counts/μT，精确增益和硬/软铁标定 |
| UBX NAV-PVT | 92字节payload，CK_A/CK_B，流式重同步；3D fix/gnssFixOK；SI速度、两种高度与精度；按已核实版本可选flags3/invalidLlh | UART字节、接收机协议版本、明确导航epoch的MCU时间映射 |
| Counter32 | 单写者扩展32位微秒计数器，捕获时刻以最近周期扩展，拒绝时间倒退 | 1MHz硬件计数器；采样间隔小于半周期约35.8分钟；跨任务需BSP临界区 |
| PPS | 周期间隔一致性、锁定与ppm诊断、2.5秒锁定超时 | MCU输入捕获；**不改变单调时间，不生成UTC/GPS周** |
| ExposureRegistry | 帧号一次消费、漏帧计数、回绕、触发偏移+曝光中点、2.56秒过期拒绝 | 实测trigger→曝光延时或实际Strobe沿，准确曝光长度 |

ADIS温度为0.1°C/LSB，0°C=0，250即25°C；加速度1.25mg/LSB，角速度0.1°/s/LSB。驱动只配置direct-sync的DATA_CNTR语义，不支持scaled-sync下同一字对应的TIME_STMP。读取完整176位期间CS保持有效。正常单寄存器SPI访问间隔16μs由`SpiBus::delayUs`落实。

F9P的串口接收时刻不是定位epoch。`decodeUbxNavPvt`保留调用者传入的`t_us`，**不估算UART串行/接收机输出延时**。只有以PPS/iTOW建立可信epoch映射后，BSP才应调用`gnssMeasurements`提交该时间；未映射则记录原始帧用于离线对照，不把接收时间作为授时精度声明。

PVT的offset78在旧F9P R04手册中为reserved，新协议布局为flags3。默认按已核实的旧F9P布局忽略reserved；只有BSP读取MON-VER并核实实际手册后才设`UbxParser(true)`/`decodeUbxNavPvt(...,true)`启用invalidLlh校验。新布局参考u-blox M10协议手册，不能把一个型号的新增flags直接宣称为所有F9P固件版本支持。

## 任务和内存所有权

`Runtime`的每个FixedQueue都只有一个生产者和消费者，使用32位无锁原子；容量为2的幂，满队列拒绝并计数。传感器任务与网络任务拥有不同量测队列，多个ISR不得同时生产同一队列。`NavTask`是`NavCore`唯一写者，输出与视觉是否到达无关；所有值通过协议codec序列化。

量测在NavTask的有界等待表内调度，稍后的合法量测等待对应IMU，未来超过50ms的量测丢弃/计数，不阻塞其后的有效量测。`Runtime::initialize`会清队列、缓存和会话序列；它只能在所有任务/ISR暂停时调用，运行期间禁止重置。

`ImuTask`从BSP的DMA完成队列读取四路共同epoch的采样组，调用核心阵列标定/中位数故障剔除和5点平均，按400Hz入队。**四路DR ISR只开始DMA/排队，不在ISR内运行滤波器。** `NavTask`按IMU依次推进再融合到期量测；未来量测最多等待对应IMU。`LinkTask`负责接收/发送，`SensorTask`每20ms轮询外部量测，`HealthTask`每50ms检查Nav/Link/Sensor推进并决定是否喂100ms看门狗。

相机/PPS ISR分别调用`cameraCaptureFromIsr`/`ppsCaptureFromIsr`；相机事件的`mid_us`必须已经是曝光中点。相机触发号和主机相机帧号必须共享明确编号，不能在掉帧后按到达次序推断配对。

STM32H7的DMA缓冲必须置于DMA可访问SRAM，满足cache line对齐和维护要求；DTCM不能作为常规DMA数据区。BSP须实现MPU/cache维护、外设超时、以太网RX/TX、UART环形DMA、CS时序、IRQ优先级、硬件时钟同步、IWDG和DWT计时。任务栈示例需用编译器栈用量报告和运行high-water mark重新核定，不能把示例大小当成已测足够。

本次GNU13宿主`-O2 -fstack-usage`显示ESKF predict约19.7KB、update约28.9KB，尚未加调用者/目标ARM差异。NavTask示例因此默认`NAV_NAVTASK_STACK_WORDS=16384`（32位StackType即64KB），可由BSP宏覆盖。它是为留出移植余量的起始配置，**不是实测目标板栈保证**；还要核算Runtime/历史/队列、任务栈、DMA、网络和BSP总SRAM预算并测最坏CPU时间。

## 交叉构建入口

先安装ARM GNU工具链，并提供一个真实外部CMake板工程，导出`nav_board_bsp`（HAL/FreeRTOS包含路径、驱动和cpu配置）。最终ELF目标由该BSP定义，链接`nav_stm32_port`；CubeMX的main完成时钟、GPIO、DMA、SPI、UART、以太网、缓存、定时器初始化后，调用`startFreeRtosPort`，确认返回true再启动调度器。

```sh
cmake -S . -B build-arm \
  -DCMAKE_TOOLCHAIN_FILE=firmware/cmake/arm-none-eabi.cmake \
  -DBUILD_TESTING=OFF -DNAV_BUILD_COMPANION=OFF \
  -DNAV_BUILD_STM32=ON -DNAV_BOARD_PROJECT=/path/to/actual-cubemx-bsp
cmake --build build-arm --target nav_stm32_port
```

移植库需要`configSUPPORT_STATIC_ALLOCATION=1`、`configTICK_RATE_HZ>=1000`和至少6级优先级。BSP还需提供idle/timer静态任务内存回调。初始化对准、陀螺/加计温度与安装矩阵、magnetic declination、雷达地形和空气数据的厂商协议均须用实际型号/标定完成；没有型号的设备不生成模拟读数代替。

最后用示波器实测DR、SYNC、相机Trigger/Strobe，核查400/100/30Hz节拍与时间误差，测算最坏CPU/栈占用，运行断线/CRC/DMA/看门狗故障试验。这些实机检查本次尚未进行。

## 厂商依据

- [Analog Devices ADIS16470 Rev.C](https://www.analog.com/media/en/technical-documentation/data-sheets/adis16470.pdf)：burst、缩放、TEMP_OUT、SPI/MSC_CTRL/DEC_RATE。
- [PNI RM3100-CB User Manual V1.0](https://www.pnicorp.com/wp-content/uploads/RM3100-CB-User-Manual-V1.0.pdf)：signed24、DRDY、cycle count/TMRC；型号寄存器应与实物版本核对。
- [u-blox ZED-F9P UBX-18010854 R04 厂商手册镜像](https://cdn.sparkfun.com/assets/learn_tutorials/8/5/6/ZED-F9P_UBX_NMEA_and_RTCM_protocols.pdf)：NAV-PVT和UBX checksum。
- [u-blox M10 SPG5.20协议](https://content.u-blox.com/sites/default/files/documents/u-blox-M10-SPG-5.20_InterfaceDescription_UBXDOC-304424225-20128.pdf)：新NAV-PVT flags3布局的依据，启用前仍需核实接收机实际版本。
- [ST STM32H743 DS12110 Rev.11](https://www.st.com/resource/en/datasheet/stm32h743zi.pdf)、[FreeRTOS静态任务API](https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/02-xTaskCreateStatic)。
