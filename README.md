## 本仓库储存项目各模块核心代码
#### 开发过程中更新功能列表
---
### 装配核心流程控制（**最终目标**）   
    工作的主要流程，主要调用其它模块的函数、类等等。
### 主要功能模块
#### 通讯模块6YGY6
    1. 工控机与PMAC通讯。
    2. 与其它所有子系统的通讯建立、协议内容、编码解码、异常处理。
    3. 所有传感器的连接、读取数据、数据预处理。
#### 安全模块
    1. 开机自检、运行时检测、限位开关安全逻辑等。
    2. 异常记录与安全措施。
#### 界面UI
    本项目统一界面程序文件
#### 全局变量
    将所有需要跨文件访问的变量汇总？
#### 子功能模块
    封装成函数以供核心流程调用，或许还需增加中间层
##### 执行元件控制模块
    1. 单个电机运转控制、多电机协调控制，速度、加速度等控制。
    2. 抓手控制、气缸控制等。
##### 数据库模块
    储存装配历史、运行记录、错误记录等。
##### 视觉模块
    1. 与工控机建立连接、通讯协议。
    2. 条码识别、零件定位。

