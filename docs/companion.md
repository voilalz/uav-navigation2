# 视觉导航、地形与参考库

本次实现提供标准C++17 CPU算法与真实图像文件回放。相机厂商SDK、Jetson统一内存/CUDA/VPI、学习模型和TensorRT引擎没有随方案提供，因此没有以假量测代替这些输入。

## 已实现的算法

| 算法 | 行为与适用条件 |
|---|---|
| 图像IO | P2/P5 PGM、8/16bit输入，尺寸/截断/像素范围验证 |
| 相机几何 | 标定针孔、FRD→NED姿态、固定下视安装、像素射线与地面平面交会、逆投影 |
| 平面正射 | 双线性重采样到北向上、统一GSD网格；有效mask；输入需先去畸变 |
| 景象定位 | coarse/fine NCC候选搜索、峰值/PSR/次峰/纹理/有效比例门限，返回位置与保守协方差 |
| 平面VO | 两帧实际纹理匹配与姿态/高度尺度换算；输出曝光时间、区间、二维NED速度/协方差 |
| DSM配准 | 2D高程网格搜索，消除常量高度偏差；几何/粗糙度/多解门限；输出平移和高程修正 |
| 稀疏3D | 已匹配像点的双帧射线三角化、正深度/基线角/射线间隙/重投影检查 |
| 参考库 | 空间窗口筛选、DOM/DSM元数据、SHA256校验、有限容量LRU、地图边界/高度基准检查 |

当前NCC不是SuperPoint/LightGlue/LoFTR；平面VO不是完整LK/VPI或滑窗VIO；稀疏三角化不等于稠密SfM、局部BA或城市真正射。平面假设在高楼、起伏和大倾角下须限制或升级DSM投影。跨季节/跨光谱效果与嵌入式时延没有真实数据验证。

`UnavailableLearnedMatcher`未有权重/运行时返回不可用；`UnavailableDenseReconstructor`显式失败。`LearnedMatcher`/`DenseReconstructor`是后续替换接口，接口本身不代表对应后端已实现。

## 演示与回放

```sh
./build/companion/nav_companion --demo
python tools/build_reference.py demo --output out/reference-demo
./build/companion/nav_companion --replay out/reference-demo/frames.manifest
```

演示实际产生PGM和DSM，再经过参考库加载与算法匹配。三幅影像的位移为北向每帧2m、东向5m，曝光相隔0.1s；预期VO为20/50m/s。DSM示例恢复(−6,+4)m平移和+7.5m高程偏差；稀疏3D示例有独立点位约束。

## 参考库生产

输入必须是已去畸变、正射、统一坐标与高度基准的DOM和高程网格。工具不会自动给普通图片做测绘地理定位、真正射或大地水准面转换。

```sh
python tools/build_reference.py build \
  --image corridor.pgm --height corridor.height --output out/reference \
  --origin-lat 30 --origin-lon 120 --north 200 --east -100 --gsd 1 \
  --height-datum WGS84_ELLIPSOID --acquired-utc 2026-01-01T00:00:00Z
```

高程文件是文本：第一行`NAVHEIGHT 1 width height`，随后width×height个有限m值。网格左上像素中心为(north,east)，行号增加向南、列号增加向东。切片产生`index.tsv`和每瓦片`.meta`/`.pgm`/`.height`，记录地理原点、GSD、时相、边界、纹理/粗糙度与SHA256。`index.tsv.sha256`保护清单完整性；哈希用于损坏检测，不是签名认证。

覆盖大航程时按瓦片局部原点做大地坐标转换，不应把数百km走廊当成一个精确平面。森林VO尺度应使用经过校正的真实地表高度；不能默认雷达树冠高度即正确尺度。

## 回放manifest

`NAVREPLAY 1`头，随后key=value元数据；`FRAMES`后的CSV列如下：

```text
image,t_us,north_m,east_m,height_m,plane_height_m,roll_rad,pitch_rad,yaw_rad,altitude_sigma_m
```

必填字段包括`reference`、`fx/fy/cx/cy`、`ortho_gsd_m/width/height`、`search_radius_m`、`height_datum=WGS84_ELLIPSOID`。可选`terrain_observed`及其网格原点/GSD/搜索半径。参见工具生成的`frames.manifest`完整示例。图像路径相对manifest，不允许`..`；曝光时间必须严格递增并属于MCU时钟。

stdout为JSONL：`image_fix`、`vo`、`terrain_registration`及summary。`valid=false`行不会发送量测。image_fix为局部NED二维位置；VO为NED二维速度且协方差同系。输出的`velocity_body`仅为辅助估计，二维平面VO传输必须使用`velocity_ned`。

## MCU通信

```sh
./build/companion/nav_companion --replay out/reference-demo/frames.manifest > out/measurements.jsonl
python -m integration.companion_sender --input out/measurements.jsonl \
  --origin-lat 30 --origin-lon 120 --output out/measurements.bin
```

显式`--udp host:port`开启传输，发送POSITION_FIX和带frame=NED/dimensions=2的VO；保留曝光时刻，不改成发送时刻。MCU仍检查年龄、序列号和新息。一次服务会话共享16bit序号，重启后需按硬件移植说明显式重置会话。

原始DSM注册输出是相对修正，需要上层根据重建时刻的导航先验和高程基准形成绝对位置量测；通用发送器不会凭一个相对偏移猜出绝对飞机高度。稠密DSM生产和此类实机调度属于后续目标平台接入。
