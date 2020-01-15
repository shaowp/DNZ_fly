#include "IMU.h"

#define Kp 10.0f	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0001f; // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.00125f         // half the sample period
#define halfT 0.0025f // half the sample period = 3ms/2

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error
float yaw_control;	//偏航角控制量

//////////////////////////////
//全局变量，存放姿态的数据
Struct_IMU IMU;
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
	//将原始数据转化为度°
	float yaw_G = gz * Gyro_G;

	//2000°的量程应该选择 16.3825
	//1000°的量程应该选择 32.765
	//1度(°)=0.0174533弧度(rad)
	//由于陀螺仪的数据为16位的，所有最高为32767，最小为-32767
	//所以，32767对应2000°，-32767对应-2000°，即转化的关系为65535/4000
	//所以，原始数据可以通过这个转化为°，最后转化为弧度
	gx = gx / 16.3825 * 0.0174532;
	gy = gy / 16.3825 * 0.0174532;
	gz = gz / 16.3825 * 0.0174532;

	// printf("%.2f\t%.2f\t%.2f\n",gx,gy,gz);

	//加速度归一化
	norm = Q_rsqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//加速度算法
	//vx、vy、vz，其实就是当前的机体坐标参照系上，换算出来的重力单位向量。(用表示机体姿态的四元数进行换算)
	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	// printf("%.2f\t%.2f\t%.2f\n", vx, vy, vz);

	//磁力失效
	if (mx == 0 && my == 0 && mz == 0)
	{
		// 单独加速度，不带磁力计
		ex = (ay * vz - az * vy);
		ey = (az * vx - ax * vz);
		ez = (ax * vy - ay * vx);
	}
	else //磁力计工作
	{
		norm = Q_rsqrt(mx * mx + my * my + mz * mz);
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm;
		//磁力计算法
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
		bx = sqrt(hx * hx + hy * hy);
		bz = hz;
		//磁力计算法
		wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
		wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
		wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);
		// printf("mx%.2f\tmy%.2f\tmz%.2f\t",mx,my,mz);
		// 加速度，磁力计融合
		// error is sum of cross product between reference direction of fields and direction measured by sensors
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	}

	if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt += 0.005 * ex * Ki;
		eyInt += 0.005 * ey * Ki;
		ezInt += 0.005 * ez * Ki;
	}
	gx = gx + Kp * ex + exInt;
	gy = gy + Kp * ey + eyInt;
	gz = gz + Kp * ez + ezInt;

	// // adjusted gyroscope measurements
	// if (FL_ABS(ay) < 0.9 && FL_ABS(az - 1) < 0.9)
	// {
	// 	gx = gx + Kp * ex + exInt;
	// }
	// else
	// {
	// 	gx = gx;
	// }
	// if (FL_ABS(ax) < 0.9 && FL_ABS(az - 1) < 0.9)
	// {
	// 	gy = gy + Kp * ey + eyInt;
	// }
	// else
	// {
	// 	gy = gy;
	// }
	// if (FL_ABS(ax) < 0.9 && FL_ABS(ay) < 0.9)
	// {
	// 	gz = gz + Kp * ez + ezInt;
	// }
	// else
	// {
	// 	gz = gz;
	// }

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

	angle_x = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1); // roll
	angle_y = asin(-2 * q1 * q3 + 2 * q0 * q2);									// pitch
	angle_z = atan2(2 * q1 * q2 + 2 * q0 * q3, 1 - 2 * q2 * q2 - 2 * q3 * q3);  //y aw

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
	IMU.pitch = angle_y;
	IMU.yaw_mag = angle_z; //磁力计融合出来的数据

	/*偏航角一阶互补*/
	if ((yaw_G > 2.0f) || (yaw_G < -2.0f)) //数据太小可以认为是干扰，不是偏航动作
	{
		IMU.yaw += yaw_G * 0.005;
	}
	if ((IMU.yaw_mag > 90 && IMU.yaw < -90) || (IMU.yaw_mag < -90 && IMU.yaw > 90))
		IMU.yaw = -IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;
	else
		IMU.yaw = IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;

	if (IMU.yaw < 0)
		yaw_control = IMU.yaw + 360; //如果小于0，则+360
	else
		yaw_control = IMU.yaw;

	//***************主要是为了计算偏航角，但是没用到，节省计算量*****************/
	//倾斜角度的三角函数值
	// IMU.Cos_Pitch = cos(IMU.pitch * AtR);
	// IMU.Sin_Pitch = sin(IMU.pitch * AtR);
	// IMU.Cos_Roll = cos(IMU.roll * AtR);
	// IMU.Sin_Roll = sin(IMU.roll * AtR);
	// IMU.Cos_Yaw = cos(IMU.yaw * AtR);
	// IMU.Sin_Yaw = sin(IMU.yaw * AtR);
}

//梯度下降法
//梯度下降法
typedef volatile struct
{
	float q0;
	float q1;
	float q2;
	float q3;
} Quaternion;
Quaternion NumQ = {1.0f, 0.0f, 0.0f, 0.0f};

static const uint16_t Quad_Delay = 1; //5//往后的延迟解算，这里就是我多次运动之后回复缓慢的原因  源程序这里给的是10
const uint8_t Quad_Num = Quad_Delay;
float CNTLCYCLE = 0.005f;
float Yaw_Correct = 0;
static float Quad_Buf[Quad_Num + 1][4] = {0}; //延时取第10组以后的四元数

void AHRSUpdate_GraDes_Delay_Corretion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;				  // 平方根
	float s0, s1, s2, s3;			  // 梯度下降算子求出来的姿态
	float qDot1, qDot2, qDot3, qDot4; // 四元数微分方程求得的姿态
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
	float hx, hy;
	float delta;
	float tempx, tempy, tempz;
	u8 i = 0;

	float yaw_G = gz / 16.3825;

	for (i = Quad_Num; i > 0; i--) //将四元数历史值保存起来
	{
		Quad_Buf[i][0] = Quad_Buf[i - 1][0];
		Quad_Buf[i][1] = Quad_Buf[i - 1][1];
		Quad_Buf[i][2] = Quad_Buf[i - 1][2];
		Quad_Buf[i][3] = Quad_Buf[i - 1][3];
	}
	Quad_Buf[0][0] = NumQ.q0;
	Quad_Buf[0][1] = NumQ.q1;
	Quad_Buf[0][2] = NumQ.q2;
	Quad_Buf[0][3] = NumQ.q3;

	tempx = gx / 16.3825 * 0.0174532;
	tempy = gy / 16.3825 * 0.0174532;
	tempz = gz / 16.3825 * 0.0174532;

	/* 四元数微分方程计算本次待矫正四元数 */
	qDot1 = 0.5f * (-Quad_Buf[Quad_Delay][1] * tempx - Quad_Buf[Quad_Delay][2] * tempy - Quad_Buf[Quad_Delay][3] * tempz);
	qDot2 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempx + Quad_Buf[Quad_Delay][2] * tempz - Quad_Buf[Quad_Delay][3] * tempy);
	qDot3 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempy - Quad_Buf[Quad_Delay][1] * tempz + Quad_Buf[Quad_Delay][3] * tempx);
	qDot4 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempz + Quad_Buf[Quad_Delay][1] * tempy - Quad_Buf[Quad_Delay][2] * tempx);

	/* 加速度计输出有效时,利用加速度计补偿陀螺仪 */
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		recipNorm = Q_rsqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		/* 避免重复运算 */
		_2q0 = 2.0f * NumQ.q0;
		_2q1 = 2.0f * NumQ.q1;
		_2q2 = 2.0f * NumQ.q2;
		_2q3 = 2.0f * NumQ.q3;
		_4q0 = 4.0f * NumQ.q0;
		_4q1 = 4.0f * NumQ.q1;
		_4q2 = 4.0f * NumQ.q2;
		_8q1 = 8.0f * NumQ.q1;
		_8q2 = 8.0f * NumQ.q2;
		_2q0q2 = 2.0f * NumQ.q0 * NumQ.q2;
		_2q2q3 = 2.0f * NumQ.q2 * NumQ.q3;
		q0q0 = NumQ.q0 * NumQ.q0;
		q1q1 = NumQ.q1 * NumQ.q1;
		q2q2 = NumQ.q2 * NumQ.q2;
		q3q3 = NumQ.q3 * NumQ.q3;
		q0q1 = NumQ.q0 * NumQ.q1;
		q0q2 = NumQ.q0 * NumQ.q2;
		q0q3 = NumQ.q0 * NumQ.q3;
		q1q2 = NumQ.q1 * NumQ.q2;
		q1q3 = NumQ.q1 * NumQ.q3;
		q2q3 = NumQ.q2 * NumQ.q3;

		//屏蔽磁力计
		mx = 0;
		my = 0;
		mz = 0;
		//磁力计不工作的情况
		if (mx == 0 && my == 0 && mz == 0)
		{
			/* 梯度下降算法,计算误差函数的梯度 */
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * NumQ.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0f * q0q0 * NumQ.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0f * q1q1 * NumQ.q3 - _2q1 * ax + 4.0f * q2q2 * NumQ.q3 - _2q2 * ay;
		}
		else
		{

			recipNorm = Q_rsqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			_2q0mx = 2.0f * NumQ.q0 * mx;
			_2q0my = 2.0f * NumQ.q0 * my;
			_2q0mz = 2.0f * NumQ.q0 * mz;
			_2q1mx = 2.0f * NumQ.q1 * mx;

			hx = mx * q0q0 - _2q0my * NumQ.q3 + _2q0mz * NumQ.q2 + mx * q1q1 + _2q1 * my * NumQ.q2 + _2q1 * mz * NumQ.q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * NumQ.q3 + my * q0q0 - _2q0mz * NumQ.q1 + _2q1mx * NumQ.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * NumQ.q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * NumQ.q2 + _2q0my * NumQ.q1 + mz * q0q0 + _2q1mx * NumQ.q3 - mz * q1q1 + _2q2 * my * NumQ.q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		}

		/* 梯度归一化 */
		recipNorm = Q_rsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		{																  //快速补偿姿态，根据飞行器变化飞行角度的幅度
			const float Beta_Adjust[5] = {0.03, 0.03, 0.025, 0.02, 0.01}; //{0.04,0.03,0.025,0.02,0.01};//根据加速度模长的变化调整
			float BETADEF = 0.04f;
			float Tmep_Acce_Length;
			// Tmep_Acce_Length = LIMIT(Attitude.Acceleration_Length, 0, 1000); //正常悬停在500以内 //限幅
			Tmep_Acce_Length = 0;										   //正常悬停在500以内 //限幅
			BETADEF = Beta_Adjust[0] - 0.01f * Tmep_Acce_Length / 1000.0f; //动态步长
			qDot1 -= BETADEF * s0;
			qDot2 -= BETADEF * s1;
			qDot3 -= BETADEF * s2;
			qDot4 -= BETADEF * s3;
		}
	}

	/* 补偿由四元数微分方程引入的姿态误差 */
	/* 将四元数姿态导数积分,得到当前四元数姿态 */
	/* 二阶毕卡求解微分方程 */
	delta = (CNTLCYCLE * tempx) * (CNTLCYCLE * tempx) + (CNTLCYCLE * tempy) * (CNTLCYCLE * tempy) + (CNTLCYCLE * tempz) * (CNTLCYCLE * tempz);
	NumQ.q0 = (1.0f - delta / 8.0f) * NumQ.q0 + qDot1 * CNTLCYCLE;
	NumQ.q1 = (1.0f - delta / 8.0f) * NumQ.q1 + qDot2 * CNTLCYCLE;
	NumQ.q2 = (1.0f - delta / 8.0f) * NumQ.q2 + qDot3 * CNTLCYCLE;
	NumQ.q3 = (1.0f - delta / 8.0f) * NumQ.q3 + qDot4 * CNTLCYCLE;

	/* 单位化四元数 */
	recipNorm = Q_rsqrt(NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3);
	NumQ.q0 *= recipNorm;
	NumQ.q1 *= recipNorm;
	NumQ.q2 *= recipNorm;
	NumQ.q3 *= recipNorm;
	IMU.roll = atan2(2.0f * NumQ.q2 * NumQ.q3 + 2.0f * NumQ.q0 * NumQ.q1, -2.0f * NumQ.q1 * NumQ.q1 - 2.0f * NumQ.q2 * NumQ.q2 + 1.0f) * RtA; // Pitch
	IMU.pitch = asin(2.0f * NumQ.q0 * NumQ.q2 - 2.0f * NumQ.q1 * NumQ.q3) * RtA;															  // Roll
	// IMU.yaw_mag = atan2(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3);					  //y aw
																																			  //NumQ.angle[YAW] = atan2(2.0f * NumQ.q[1] * NumQ.q[2] + 2.0f * NumQ.q[0] * NumQ.q[3], -2.0f * NumQ.q[3] * NumQ.q[3] - 2.0f * NumQ.q[2] * NumQ.q[2] + 1.0f) * RAD2DEG;// Yaw

	/*偏航角一阶互补*/

	if ((yaw_G > 2.0f) || (yaw_G < -2.0f)) //数据太小可以认为是干扰，不是偏航动作
	{
		IMU.yaw += gz * Gyro_G * CNTLCYCLE;
	}

	//融合磁力计部分，由于当前磁力计距离电机太近了，所以不使用磁力计
	// if ((IMU.yaw_mag > 90 && IMU.yaw < -90) || (IMU.yaw_mag < -90 && IMU.yaw > 90))
	// 	IMU.yaw = -IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;
	// else
		// IMU.yaw = IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;
	if (IMU.yaw < 0)
		yaw_control = IMU.yaw + 360; //如果小于0，则+360
	else
		yaw_control = IMU.yaw;

	//倾斜角度的三角函数值
	IMU.Cos_Pitch = cos(IMU.pitch * AtR);
	IMU.Sin_Pitch = sin(IMU.pitch * AtR);
	IMU.Cos_Roll = cos(IMU.roll * AtR);
	IMU.Sin_Roll = sin(IMU.roll * AtR);
	IMU.Cos_Yaw = cos(IMU.yaw * AtR);
	IMU.Sin_Yaw = sin(IMU.yaw * AtR);
}
