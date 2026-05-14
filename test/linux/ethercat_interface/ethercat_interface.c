#include "ethercat_interface.h"
int main(){
    start_ethercat();
   //  while (1)
   //  {
   //     printf("%lf",feed(0));
   //  }
   printf("START\n");
   osal_usleep(1000000);

   double commands[SERVO_NUMBER];
   for (size_t jj = 0; jj < SERVO_NUMBER; jj++)
   {
    commands[jj]=0;     
   }
   joint_command(commands);
   commands[0]=0;
   joint_command(commands);
    close_ethercat();

}
