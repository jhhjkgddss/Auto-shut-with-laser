# Auto-shut-with-laser
基于openmv的视觉瞄准射击
利用stm32f405作为主控，用openmv作为检测器，Openmv将目标物的实际坐标用IIC传给主控，利用PID控制两个舵机，一个舵机装openmv摄像头，另一个装激光发射器，先瞄准，然后激光发射器进行校正后对准开火。
