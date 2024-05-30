# Self_Balancing_Robot_on_DC_Motors V0.1

This project involves creating a self-balancing robot using DC motors. After completing Paul McWhorter's YouTube series on 9-Axis Inertial Measurement Units (IMUs) with Arduino, I implemented the design using anMPU6050 sensor, as I didn't have access to the BNO055 9-Axis IMU. The principles of noise filtering remain the same. Paul's organized approach was instrumental, covering low pass filters, angle calculations, sensor fusion, and data visualization with Vpython.

To build this robot, I first applied low pass filters and then used the Kalman filter to achieve a better response. This process also enhanced my understanding of PID tuning and its application in maintaining the robot's balance.

helpful links: 

<a href="https://www.youtube.com/playlist?list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9"> 9 Axis Inertial Measurement Units With Arduino Tutorial</a>

<a href="https://www.youtube.com/watch?v=wkfEZmsQqiA&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y&ab_channel=MATLAB">What Is PID Control?</a>

<a href="https://www.youtube.com/watchv=MJiVtz4Uj7M&list=PLGs0VKk2DiYzGCOzBrMNSWEdd2CIGC0kJ&ab_channel=PaulMcWhorter"> Visual Python 3D Graphics and Animations</a>


![image](https://github.com/IbrahimEssamIbrahim/Self_balancing_Robot_V0.1/assets/141848382/376acd3c-1e4c-4fe3-81a4-b295390ad15d)
