# 基于STM32F103C8T6的飞控
## 一、引脚布局
![20200115132918.png](http://markdown.shaowp.cn/img/20200115132918.png)
四通道PWM输入输出
一条IIC总线连接MPU6050和AK8975
串口和USB负责和上位机通讯
两个LED指示灯

## 二、原理图
#### EEPROM
![20200115133143.png](http://markdown.shaowp.cn/img/20200115133143.png)


#### IMU
![20200115133216.png](http://markdown.shaowp.cn/img/20200115133216.png)

#### 控制
![20200115133244.png](http://markdown.shaowp.cn/img/20200115133244.png)

## 三、实物图
![20200115134349.png](http://markdown.shaowp.cn/img/20200115134349.png)
![20200115134320.png](http://markdown.shaowp.cn/img/20200115134320.png)
![20200115134333.png](http://markdown.shaowp.cn/img/20200115134333.png)
![20200115134401.png](http://markdown.shaowp.cn/img/20200115134401.png)
![20200115134418.png](http://markdown.shaowp.cn/img/20200115134418.png)
![20200115134428.png](http://markdown.shaowp.cn/img/20200115134428.png)
![20200115134439.png](http://markdown.shaowp.cn/img/20200115134439.png)
![20200115134450.png](http://markdown.shaowp.cn/img/20200115134450.png)
## 代码用的标准库，上位机是匿名4.12版本，参考的代码比较多，因为做完有好久了，一直没开源，有不懂得可以qq联系:416440965
