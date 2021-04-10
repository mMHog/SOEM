
int setpoint[SERVO_NUMBER]={0};       //设定值
double Kp=0.2;     //比例系数
double Ki=0.015;      //积分系数
double Kd=0.2;    //微分系数

int lasterror[SERVO_NUMBER]={0};     //前一拍偏差   
int preerror[SERVO_NUMBER]={0};     //前两拍偏差
int result[SERVO_NUMBER]={0}; //输出值
int processValue[SERVO_NUMBER]={0};


void PIDRegulation(int *processValue)
{
  int thisError[SERVO_NUMBER];
  double increment[SERVO_NUMBER];
  int pError[SERVO_NUMBER],dError[SERVO_NUMBER],iError[SERVO_NUMBER];
  for (size_t jj = 0; jj < 1; jj++)
  {  
    thisError[jj]=setpoint[jj]-processValue[jj]; //当前误差等于设定值减去当前值
    //计算公式中除系数外的三个 乘数
    pError[jj]=thisError[jj]-lasterror[jj];//两次偏差差值err(k)-err(k-1)
    iError[jj]=thisError[jj];
    dError[jj]=thisError[jj]-2*(lasterror[jj])+preerror[jj];

    increment[jj]=Kp*pError[jj]+Ki*iError[jj]+Kd*dError[jj];   //增量计算

    preerror[jj]=lasterror[jj];  //存放偏差用于下次运算
    lasterror[jj]=thisError[jj];
    
    result[jj]+=(int)increment[jj];//结果当然是上次结果 加上本次增量
    printf("%d %lf\n",thisError[0],increment[0]);
  }
}