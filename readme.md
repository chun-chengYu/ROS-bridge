# wheeltec-4dw-pig-farm-sim — ROS2 → STM32 UART 轉換節點

將 ROS2 標準速度指令 `/cmd_vel` 轉換為STM32所需的序列埠控制封包，只要發布標準的 `geometry_msgs/msg/Twist` 訊息即可控制車輛。

## 功能

- 訂閱 `/cmd_vel`（`geometry_msgs/msg/Twist`），即時轉換並送出對應的 UART 控制封包
- 依協議規格將 `linear.x` / `linear.y` / `angular.z` 轉換為 mm/s、mm/s、rad/s×1000，並以 `int16_t` 大端序（Big-Endian）打包
- 封包自帶 XOR checksum，符合底盤原廠協議
- **500ms 逾時機制**：超過 0.5 秒沒收到新指令，每 200ms 自動送出一次全零的安全停止封包
- 皆透過同一個 `/cmd_vel` 介面接入

## 專案範圍

- 本節點**只負責** ROS2 端的封包轉換（`cpp_topic_pkg` 套件、`wheeltec_uart_bridge` 節點）
- **未修改**STM32 韌體
- 待實際接上車輛後，需將 `usart_port_name` 參數指向序列埠（預設 `/dev/wheeltec_controller`，或視情況使用 `/dev/ttyUSB0`）

## 通訊協議

| 項目 | 規格 |
|---|---|
| 串口參數 | 115200 Baud, 8 Data Bits, 1 Stop Bit, No Parity (8N1) |
| 硬體串口 | STM32 底盤預設 USART3 |
| 裝置路徑 | 預設 `/dev/wheeltec_controller`（udev symlink），可用 `usart_port_name` 參數覆寫 |

**下行封包結構（11 bytes）**

| Byte | 內容 |
|---|---|
| [0] | `0x7B`（帧頭） |
| [1]–[2] | 預留 |
| [3]–[4] | X 軸線速度（mm/s，Big-Endian） |
| [5]–[6] | Y 軸線速度（mm/s，Big-Endian） |
| [7]–[8] | Z 軸角速度（rad/s × 1000，Big-Endian） |
| [9] | Checksum（前 9 bytes XOR） |
| [10] | `0x7D`（帧尾） |

## 安裝與建置

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_topic_pkg --dependencies rclcpp std_msgs
# 加入 geometry_msgs 依賴、放入 src/uart_bridge_node.cpp

cd ~/ros2_ws
colcon build --packages-select cpp_topic_pkg
```

## 執行方式

```bash
source install/setup.bash
ros2 run cpp_topic_pkg wheeltec_uart_bridge --ros-args -p usart_port_name:=/dev/ttyACM0
```

未指定 `usart_port_name` ，使用 `/dev/ttyACM0`；接上實際車輛時請改為對應裝置路徑(此為測試板路徑)。

## 測試方式

由於開發階段沒有實體車輛可用，改以一塊**與車輛無關的 STM32 Nucleo-144 (F429ZI)** 開發板作為臨時接收端：韌體僅解析封包格式（帧頭/帧尾/checksum）並將解碼結果透過序列埠印出，驗證轉換節點輸出是否符合協議規格。

測試範圍：
- 正值 / 負值指令的打包與解碼正確性（含有號數 / 兩補數表示）
- 逾時停止機制是否正常觸發
- Checksum 計算正確性
- 手動逐位元組核對，確認邏輯無誤

測試環境搭建（含 WSL2 + `usbipd-win` 橋接 USB 裝置）。

## 專案結構

```
cpp_topic_pkg/
├── src/
│   └── uart_bridge_node.cpp   # 轉換節點主程式
├── package.xml
├── CMakeLists.txt

```
