
#include "filter.h"
#include "math.h"
#include "include.h"
float Accel_x; //X轴加速度值暂存
float Accel_y; //Y轴加速度值暂存
float Accel_z; //Z轴加速度值暂存

float Gyro_x; //X轴陀螺仪数据暂存
float Gyro_y; //Y轴陀螺仪数据暂存
float Gyro_z; //Z轴陀螺仪数据暂存

//float Angle_gy;    //由角速度计算的倾斜角度
float Angle_x_temp; //由加速度计算的x倾斜角度
float Angle_y_temp; //由加速度计算的y倾斜角度
float Angle_z_temp;

float Angle_X_Final; //X最终倾斜角度
float Angle_Y_Final; //Y最终倾斜角度
float Angle_Z_Final; //Z最终倾斜角度

//卡尔曼参数
char C_0 = 1;
float Q_bias_x, Q_bias_y, Q_bias_z;
float Angle_err_x, Angle_err_y, Angle_err_z;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = {0, 0, 0, 0};
float PP[2][2] = {{1, 0}, {0, 1}};

double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;
	static double x_last;
	double x_mid = x_last;
	double x_now;
	static double p_last;
	double p_mid;
	double p_now;
	double kg;
	x_mid = x_last;							  //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid = p_last + Q;						  //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg = p_mid / (p_mid + R);				  //kg=kalman filter,R=噪声
	x_now = x_mid + kg * (ResrcData - x_mid); //估最优值
	p_now = (1 - kg) * p_mid;				  //最优值对应的covariance
	p_last = p_now;							  //更新covariance值
	x_last = x_now;							  //更新系统状态值
	return x_now;
}

//角度计算
void Angle_Calcu(void)
{
	//范围为2g时，换算关系：16384 LSB/g
	//deg = rad*180/3.14
	float x = 0, y = 0, z = 0;

	Accel_x = MPU6050_Raw_data.accx; //x轴加速度值暂存
	Accel_y = MPU6050_Raw_data.accy; //y轴加速度值暂存
	Accel_z = MPU6050_Raw_data.accz; //z轴加速度值暂存
	Gyro_x = MPU6050_Raw_data.gyrox; //x轴陀螺仪值暂存
	Gyro_y = MPU6050_Raw_data.gyroy; //y轴陀螺仪值暂存
	Gyro_z = MPU6050_Raw_data.gyroz; //z轴陀螺仪值暂存

	//处理x轴加速度

	//用加速度计算三个轴和水平面坐标系之间的夹角
	Angle_x_temp = (atan2(x, z)) * 180 / Pi;
	Angle_y_temp = (atan2(y, z)) * 180 / Pi;
	Angle_z_temp = (atan2(y, x)) * 180 / Pi;

	//角度的正负号
	Gyro_y = -Gyro_y / 131.00;
	Gyro_y = -Gyro_y / 131.00;
	Gyro_y = -Gyro_y / 131.00;
	//角速度
	//向前运动
	if (Gyro_x < 32768)
		Gyro_x = -(Gyro_x / 16.4); //范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if (Gyro_x > 32768)
		Gyro_x = +(65535 - Gyro_x) / 16.4;
	//向前运动
	if (Gyro_y < 32768)
		Gyro_y = -(Gyro_y / 16.4); //范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if (Gyro_y > 32768)
		Gyro_y = +(65535 - Gyro_y) / 16.4;
	//向前运动
	if (Gyro_z < 32768)
		Gyro_z = -(Gyro_z / 16.4); //范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if (Gyro_z > 32768)
		Gyro_z = +(65535 - Gyro_z) / 16.4;

	//Angle_gy = Angle_gy + Gyro_y*0.025;  //角速度积分得到倾斜角度.越大积分出来的角度越大
	//Kalman_Filter_X(Angle_x_temp, Gyro_x); //卡尔曼滤波计算X倾角
	//	yijiehubu_P(Angle_x_temp, Gyro_x);
	Erjielvbo(Angle_x_temp, Gyro_x);
	// printf("%f\n",Gyro_x);
	// Erjielvbo(Angle_y_temp, Gyro_y);
	// Erjielvbo(Angle_z_temp, Gyro_z);
	// Kalman_Filter_Y(Angle_y_temp, Gyro_y); //卡尔曼滤波计算Y倾角
	// Kalman_Filter_Z(Angle_z_temp, Gyro_z); //卡尔曼滤波计算Y倾角

	pitch = Angle_Y_Final;
	roll = Angle_X_Final;
	yaw = Angle_Z_Final;
}

void Kalman_Filter_X(float Accel, float Gyro) //卡尔曼函数
{
	Angle_X_Final += (Gyro - Q_bias_x) * dt; //先验估计

	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;

	PP[0][0] += Pdot[0] * dt; // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt; // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err_x = Accel - Angle_X_Final; //zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0; //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	Angle_X_Final += K_0 * Angle_err_x; //后验估计
	Q_bias_x += K_1 * Angle_err_x;		//后验估计
	Gyro_x = Gyro - Q_bias_x;			//输出值(后验估计)的微分=角速度
}

void Kalman_Filter_Y(float Accel, float Gyro) //卡尔曼函数
{
	Angle_Y_Final += (Gyro - Q_bias_y) * dt; //先验估计

	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;

	PP[0][0] += Pdot[0] * dt; // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt; // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err_y = Accel - Angle_Y_Final; //zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0; //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	Angle_Y_Final += K_0 * Angle_err_y; //后验估计
	Q_bias_y += K_1 * Angle_err_y;		//后验估计
	Gyro_y = Gyro - Q_bias_y;			//输出值(后验估计)的微分=角速度
}

void Kalman_Filter_Z(float Accel, float Gyro) //卡尔曼函数
{
	Angle_Z_Final += (Gyro - Q_bias_z) * dt; //先验估计

	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;

	PP[0][0] += Pdot[0] * dt; // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt; // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err_z = Accel - Angle_Z_Final; //zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0; //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	Angle_Z_Final += K_0 * Angle_err_z; //后验估计
	Q_bias_z += K_1 * Angle_err_z;		//后验估计
	Gyro_z = Gyro - Q_bias_z;			//输出值(后验估计)的微分=角速度
}

float angle_P, angle_R;
float A_P, A_R, A2_P;

void yijiehubu_P(float angle_m, float gyro_m)
{
	float K1 = 0.09;
	float d = 0.01;
	angle_P = K1 * angle_m + (1 - K1) * (angle_P + gyro_m * d);
	A_P = angle_P;
}

void erjiehubu_P(float angle_m, float gyro_m)
{
	float K = 0.05;
	float y1;
	float x2;
	float x1 = (angle_m - angle_P) * K * K;
	y1 = y1 + x1 * dt;
	x2 = y1 + 2 * K * (angle_m - angle_P) + gyro_m;
	angle_P = angle_P + x2 * dt;
	A_P = angle_P;
	Angle_X_Final = angle_P;
}

float K2 = 0.2;
float x1, x2, y1;
float angle2;

void Erjielvbo(float angle_m, float gyro_m)
{
	x1 = (angle_m - angle2) * (1 - K2) * (1 - K2);

	y1 = y1 + x1 * dt;

	x2 = y1 + 2 * (1 - K2) * (angle_m - angle2) + gyro_m;

	angle2 = angle2 + x2 * dt;
	printf("x2 %f\n", angle2);
	Angle_X_Final = angle_P;
}
