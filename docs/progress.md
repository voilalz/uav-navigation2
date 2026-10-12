# 实现记录 — docs/superpowers/plans/2026-10-12-navigation.md

用户提供三份方案并授权实现全部工程、提交指定仓库；采用既定MCU/Jetson/飞控分离架构。

- Task1 complete：导航、矩阵、机械编排、滤波、时延、模式、IMU、地形与回归。
- Task2 complete：七类C99协议、CRC、严格外帧重同步与速度frame。
- Task3 complete：CPU视觉/地形、参考库、图像回放和不提供权重时的明确无效接口。
- Task4 complete：设备解析、统一时基、FreeRTOS/BSP移植、MCU任务、飞控桥、部署文件。
- Task5 complete：CMake/CI、模拟与回放、100次长航段、600km级场景、实际图像/协议链路、中文运行与验证文档。

执行决定：CPU可复现基线作为当前视觉实现，GPU/模型/BSP依赖缺失不造假；代价是仍需实机/模型接入。短时延采用原方案名义历史近似，未宣称随机克隆；强相关/高动态时需升级。默认GPS_INPUT路径可在不修改上游源码情况下验证封装，原生External AHRS后端仍需上游版本联调。

审查修正：未来量测不再阻塞后续量测；IMU长缺口明确锁定失效并需重新对准；DEM元数据有限值检查；VO明确BODY/NED与观测维度；重启会话显式重置；NAV输出期限避免浮点累加；协方差预测配置下限20Hz；目标任务栈依据宿主报告增大且要求ARM复核。

证据：Release 15/15，ASan/UBSan 15/15（LSan环境受限），Python通信16例，100×1200秒MC与3600秒seed1。详细范围见verification.md。远端提交完成后以GitHub提交和树为最终交付记录。

远端实现提交：`de66b009450b0ba63cdca5a51aee9e0c4b0425fb`，main共91个文件；README、协议头和100次仿真JSON已逐字读取核对。
