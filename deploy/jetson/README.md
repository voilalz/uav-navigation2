# Jetson运行示例

`nav-gps-bridge.service`运行已实现的导航接收/飞控GPS_INPUT桥。先将仓库和构建结果放在`/opt/uav-navigation2`，创建`uav`账户，将`bridge.env.example`复制为`/etc/uav-navigation/bridge.env`并核对实际地址；复制service到`/etc/systemd/system`后由操作者执行`systemctl daemon-reload`和`systemctl enable --now nav-gps-bridge`。串口模式需要pyserial和dialout权限，按实际设备把ExecStart改为`--input-serial`/`--output-serial`。

MCU示例地址`192.168.42.1/24`，Jetson`192.168.42.2/24`，导航UDP接收端口14560。两个网卡配置都是模板：先核对网卡名称，再在networkd和NetworkManager/netplan中选择当前系统使用的一个。静态直连链路不添加默认网关。安装过程不由本仓库自动修改主机网络。

`nav-offline-replay.service`是按需启动的**一次性文件回放**，输入manifest和真实曝光时间，结果写JSONL。创建对应数据/日志目录并赋予`uav`权限。它不自动把JSONL转换为在线量测，也没有相机SDK、实时相机daemon、GPU神经网络或远端模型下载；板级相机采集和量测发送需要另行接入。可直接运行`build/companion/nav_companion --demo`检查CPU基线。

本示例GPS桥默认忽略高度，因为NAV_OUTPUT为椭球高而GPS_INPUT需要MSL；在确认当地大地水准面分离量之后才可显式增加`--geoid-separation N`。跨区域飞行应使用随位置变化的geoid模型替换常数。超时停止注入不会自动触发飞控切换，实际EKF和失效保护仍须独立配置与HIL验证。
