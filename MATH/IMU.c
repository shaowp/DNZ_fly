#include "IMU.h"

#define Kp 10.0f	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0001f; // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.00125f         // half the sample period
#define halfT 0.0025f // half the sample period = 3ms/2

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error

//////////////////////////////
//全局变量，存放姿态的数据
_st_IMU IMU;
//////////////////////////////

/**************************实现函数********************************************
*函数原型:	 float invSqrt(float x)
*功　　能:	 快速计算 1/Sqrt(x) 	
*输入参数：  要计算的值
*输出参数：  结果
*******************************************************************************/
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y = number;
	i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (threehalfs - (x2 * y * y));
	return y;
}

//求绝对值函数
float FL_ABS(float x)
{
	if (x < 0)
		return -x;
	else
		return x;
}

//四元数计算函数，详细的解释请参考资料包里的《姿态解算理解》一文，讲的挺详细，通俗易懂
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	double angle_x, angle_y, angle_z;
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	//xo11.00	 yo-188.00	 zo4.00	 0.98	 1.07
	//先把这些用得到的值算好
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3; //
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2; //
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	//陀螺仪数据单位的转化
	//传进来的是+-2000°/s的数据，
	//需要转化成弧度
	float yaw_G = gz * Gyro_G;
	gx = gx / 32.765 * 0.0174532;
	gy = gy / 32.765 * 0.0174532;
	gz = gz / 32.765 * 0.0174532;

	// printf("%.2f\t%.2f\t%.2f\n",gx,gy,gz);

	//加速度归一化
	norm = Q_rsqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	norm = Q_rsqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	// printf("mx%.2f\tmy%.2f\tmz%.2f\t",mx,my,mz);

	//磁力计算法
	hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
	hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	bx = sqrt(hx * hx + hy * hy);
	bz = hz;

	//加速度算法
	//vx、vy、vz，其实就是当前的机体坐标参照系上，换算出来的重力单位向量。(用表示机体姿态的四元数进行换算)
	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	// printf("%.2f\t%.2f\t%.2f\n", vx, vy, vz);

	//磁力计算法
	wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);

	// 加速度，磁力计融合
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	// 单独加速度，不带磁力计
	// ex = (ay * vz - az * vy);
	// ey = (az * vx - ax * vz);
	// ez = (ax * vy - ay * vx);

	if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki;
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;
	}

	// // adjusted gyroscope measurements
	if (FL_ABS(ay) < 0.9 && FL_ABS(az - 1) < 0.9)
	{
		gx = gx + Kp * ex + exInt;
	}
	else
	{
		gx = gx;
	}
	if (FL_ABS(ax) < 0.9 && FL_ABS(az - 1) < 0.9)
	{
		gy = gy + Kp * ey + eyInt;
	}
	else
	{
		gy = gy;
	}
	if (FL_ABS(ax) < 0.9 && FL_ABS(ay) < 0.9)
	{
		gz = gz + Kp * ez + ezInt;
	}
	else
	{
		gz = gz;
	}

	//gz = gz + Kp*ez + ezInt;

	// integrate quaternion rate and normalise
	//一阶龙格—库塔法解四元数微分方程。
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	// normalise quaternion
	//四元数归一化
	norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;

	angle_x = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1); // roll
	angle_y = asin(-2 * q1q3 + 2 * q0q2);							// pitch
	angle_z = atan2(2 * q1q2 + 2 * q0q3, 1 - 2 * q2q2 - 2 * q3q3);  //yaw

	angle_x *= RtA;
	angle_y *= RtA;
	angle_z *= RtA;
	///原来的，没考虑方向性
	//好像磁力计的方向是对的，但是6050的方向好像反了
	// IMU.roll = angle_x;
	// IMU.pitch = angle_y;
	// IMU.yaw_mag = angle_z; //磁力计融合出来的数据

	//考虑到方向性
	IMU.roll = angle_x;
	IMU.pitch = -angle_y;
	IMU.yaw_mag = angle_z; //磁力计融合出来的数据

	/*偏航角一阶互补*/
	if ((yaw_G > 2.0f) || (yaw_G < -2.0f)) //数据太小可以认为是干扰，不是偏航动作
	{
		IMU.yaw += gz * Gyro_G * 0.005;
	}
	if ((IMU.yaw_mag > 90 && IMU.yaw < -90) || (IMU.yaw_mag < -90 && IMU.yaw > 90))
		IMU.yaw = -IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;
	else
		IMU.yaw = IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;

	//***************主要是为了计算偏航角，但是没用到，节省计算量*****************/
	//倾斜角度的三角函数值
	// IMU.Cos_Pitch = cos(IMU.pitch * AtR);
	// IMU.Sin_Pitch = sin(IMU.pitch * AtR);
	// IMU.Cos_Roll = cos(IMU.roll * AtR);
	// IMU.Sin_Roll = sin(IMU.roll * AtR);
	// IMU.Cos_Yaw = cos(IMU.yaw * AtR);
	// IMU.Sin_Yaw = sin(IMU.yaw * AtR);
}
