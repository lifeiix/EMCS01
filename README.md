
# 使用TOOMOSS TCANLINPro进行bootloader升级
1. 在PC上安装[TOOMOSS TCANLINPro](http://www.toomoss.com/download/7-cn.html)
2. 用CAN总线连接TOOMOSS CAN适配器与目标板, 注意CAN_H和CAN_L不要颠倒, CAN适配器的具体引脚定义请参考[USB2XXX总线适配器文档集合](http://www.toomoss.com/download/)
3. 连接CAN适配器至PC, 打开TCANLINPro, 侧栏中点击对应使用的CAN通道(CAN1或者CAN2), **选择正常模式，禁止终端电阻，设置CAN波特率为500Kbps**
4. 克隆此工程并在KEIL MDK中打开workspace, 针对*MX017260CV01_Main*主板升级分别选择*STM32F30X-Bootloader*和*STM32F30X-Application*目标, 编译并下载至目标板
5. 在TCANLINPro中点击**高级功能** - **CAN固件升级**, 在弹出窗口中选择**标准帧**, 扫描节点**起始地址与结束地址设为0x12**, 点击确定, 如果CAN通信正常, 窗口中会列出CAN设备信息
6. 点击打开文件, 选择要升级的二进制文件(第3步*STM32F30X-Application*目标生成的加密文件*EMMC-APP_encrypt.bin*), 点击更新固件, 等待升级完成

# 待完成
- CAN波特率1Mbps
- Bootloader/Application优化
- 集成Bootloader至具体应用
