# tello_swarm
含有一个根据tello上的uwb标签位置控制它飞到指定位置的控制节点

往/expect_pos话题发送期望位置坐标和航向角增量，/command控制起飞降落悬停飞向期望点等
需要另外启动tello_driver_node(tello的ros驱动包，ros官方有），可能还要修改group
注意飞机cmd指令是前左下坐标系（符合遥控器摇杆），要和uwb基站的x、y轴对应上，具体请自行阅读并修改控制节点的代码

以上采用的是tello的热点模式，电脑连接tello的热点并启动tello_drive_node连接tello
多架tello时可以采用tello的station模式，让tello和电脑连接到同一个路由器网段（注意tello热点的网段是192.168.10.1，如果有tello开了热点，路由器要避开这个网段）
修改tello为station模式并设置连接wifi账号密码的方法在Multi-Tello-Formation-master里，用到了formation_setup.py和multi_tello_test.py，具体说明自己读里面的readme吧
此时tello的IP改变，使用tello_drive_node时需要自己修改tello_driver里的tello.py里的 self.tello_addr = ('192.168.10.1', 8889)为真实的IP，而不是在launch文件里修改IP(没用的）
用launch修改IP的方式还需要修改以下tello_driver里面param的代码

修改回热点AP模式，长按tello电源键5s重启即可

src里的两个压缩包都是睿炽客服邮件提供的代码
