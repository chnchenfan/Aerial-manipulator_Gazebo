1. joint3：0.07；joint2：0.05【test1_uavArm5.bag】

效果：joint2存在稳态误差，joint3有一点，但看不出来；平均误差0.022，最大误差0.054

![alt text](image.png)

2.joint3：0.03；joint2：0.1【test1_uavArm6.bag】

效果：joint2稳态误差基本消除，joint2有一点；平均误差0.021,最大误差0.050

![alt text](image-1.png)

3.joint3：0.04；joint2：0.1【test1_uavArm7.bag】

效果：两个角都有稳态误差；平均误差0.024，最大误差0.051

![alt text](image-2.png)

**小结1：joint2的积分必须比joint3大，并且要大到一定程度。**

如果joint2与joint3相差不大，或者更小，则两个都有稳态误差；当joint3大于且大到一定程度joint2，joint2有稳态误差，joint3没有；如果joint2大于且达到一定程度join3，两者都无稳态误差。

‘‘‘
原因：典型串联链：arm_joint2 的子链是 arm_link2，而 arm_joint3 以 arm_link2 为父链接继续往下。因此 3 轴一动就会改变 2 轴承载的等效惯量/重力矩，2 轴响应会被扰动，这是物理耦合。
’’’


4.joint3：0.04，joint2：0.12【test1_uavArm8.bag】

效果：两个角都有问题误差；平均误差0.018，最大误差0.050

![alt text](image-3.png)


5.joint3：0.07，joint2：0.14【test1_uavArm9.bag】【test1_uavArm10.bag】

效果：前60秒，二者皆有稳态误差，后续稳态误差消除。平均误差：0.024，最大误差：0.053

![alt text](image-4.png)

6.joint3：0.1，joint2：0.2【test1_uavArm11.bag】【test1_uavArm13.bag】【test1_uavArm14.bag】13，14一起的

**效果：同上。平均误差：0.025，最大误差：0.046【最终选择的数据，从60s开始计数即可】**

![alt text](image-5.png)![alt text](image-7.png)

7.joint3：0.1，joint2：0.25【test1_uavArm12.bag】

效果，同上。平均误差：0.023，最大误差：0.056

![alt text](image-6.png)

**以上所有内容均基于参数 ‘‘‘ 2m_final.params ’’’ 。**
