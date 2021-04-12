#include "pid.h"
 
int main(){
	PID_init();
	int count=0;
	while(count<500)
	{
	double speed=PID_realize(1.57);
	printf("%f\n",speed);
	count++;
	}
    count=0;
	while(count<500)
	{
	double speed=PID_realize(0);
	printf("%f\n",speed);
	count++;
	}
}