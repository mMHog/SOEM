#include <data_handle.h>

int main()
{
	double outputx[1000000];
	//printf("%d\n", rad2inc(-0.5,1));
	int num=data_process(outputx,rad2inc(0.4,1),-1286841,200,10,0.1);
	for (int i = 0; i < num; ++i)
	{
		//printf("%d\n", (int)outputx[i]);
	}
						
	printf("%ld %ld %lf\n",rad2inc(-0.470702,1),deg2inc(60,1),-0.470702*180/3.1415926);
	printf("%ld %ld %lf\n",rad2inc(-0.533108,2),deg2inc(23,2),-0.533108*180/3.1415926);
	printf("%ld %ld %lf\n",rad2inc(0.783913,3),deg2inc(60,3),0.783913*180/3.1415926);
	printf("%ld %ld %lf\n",rad2inc(-1.445173,4),deg2inc(60,4),-1.445173*180/3.1415926);
	printf("%ld %ld %lf\n",rad2inc(-1.115951,5),deg2inc(60,5),-1.115951*180/3.1415926);
	printf("%ld %ld %lf\n",rad2inc(1.290883,6),deg2inc(60,6),1.290883*180/3.1415926);
	printf("%d\n", (int)(-0.533108*180/3.1415926*65238+66246));
	return 0;
}