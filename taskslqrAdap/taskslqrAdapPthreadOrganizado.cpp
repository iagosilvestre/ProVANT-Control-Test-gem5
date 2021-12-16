#include "Icontroller.hpp"
#include <Eigen/Eigen>
#include "include/Sensor.h"
#include "include/SensorArray.h"
#include "math.h"
//#include "hinfinity.cpp"
#include "adap.cpp"
#include "lqr.cpp"
#include "m5op.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
int countAdap=0;

void *adap( void *ptr );
void *lqr( void *ptr );
main()
{
     pthread_t thread1, thread2;
     char *message1 = "Thread 1";
     char *message2 = "Thread 2";
     int  iret1, iret2;

    /* Create independent threads each of which will execute function */

     iret1 = pthread_create( &thread1, NULL, adap, (void*) message1);
     iret2 = pthread_create( &thread2, NULL, lqr, (void*) message2);

     /* Wait till threads are complete before main continues. Unless we  */
     /* wait we run the risk of executing an exit which will terminate   */
     /* the process and all threads before the threads have completed.   */

     pthread_join( thread1, NULL);
     pthread_join( thread2, NULL);

     printf("Thread 1 returns: %d\n",iret1);
     printf("Thread 2 returns: %d\n",iret2);
     exit(0);
}

void *adap( void *ptr )
{
   simulator_msgs::SensorArray arraymsg;
   std::vector<double> outA;
   std::vector<double> outL;
   std::vector<double> xref;
   std::vector<double> error;
   std::vector<double> x;

   vant3_adaptiveMixCtrl2* controlA = new vant3_adaptiveMixCtrl2();

   controlA->config();

   while(countAdap<100){
   m5_reset_stats(0,0);
   outA=controlA->execute(arraymsg);
   m5_dump_stats(0,0);
   countAdap++;
   }
   pthread_exit(NULL);				/* terminate the thread */
}
void *lqr( void *ptr )
{
   simulator_msgs::SensorArray arraymsg;
   std::vector<double> outA;
   std::vector<double> outL;
   std::vector<double> xref;
   std::vector<double> error;
   std::vector<double> x;

   teste* controlL = new teste();

   controlL->config();

   while(countAdap<100){
   outL=controlL->execute(arraymsg);
   }
   pthread_exit(NULL);	/* terminate the thread */
}
