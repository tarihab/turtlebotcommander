#include "polyintegration.h"

double integrand_const(double x, double y, void* data)
{
	return 1.0;
}

double integrand_exp(double x, double y, void* data)
{
	return exp(10*x)*tan(y);
}

class myclass {
	public:
		double integrand_exp(double x, double y, void* data)
		{
			return exp(10*x)*tan(y);
		}

};

double integrand_wrapper(void *optr, double x, double y, void* data)
{
	myclass* op = (myclass*) optr;
	double ret = op->integrand_exp(x, y, data);
	return ret;
}

int main(void)
{
	std::vector<double> xborder;
	std::vector<double> yborder;

	// unit square border
	xborder.push_back(0.0);
	xborder.push_back(1.0);
	xborder.push_back(1.0);
	//xborder.push_back(0.0);
	yborder.push_back(0.0);
	yborder.push_back(0.0);
	yborder.push_back(1.0);
	//yborder.push_back(1.0);

	double reltol = 1;
	//int resx = 50;  // resolution in x
	//int resy = 50;  // resolution in y
	double intv1;
	double intv2;

	//intv = polyintegrate(xborder, yborder, &integrand_exp, resx, resy);
	//intv1 = polyintegrate(xborder, yborder, &integrand_exp, NULL, reltol);
	intv2 = polyintegrate_gl(xborder, yborder, &integrand_exp, NULL, 512);

	//std::cout<<"The integral1 value is : "<<intv1<<std::endl;
	std::cout<<"The integral2 value is : "<<intv2<<std::endl;

	myclass *myptr;
	intv2 = polyintegrate_gl(xborder, yborder, &integrand_wrapper, myptr, NULL, 256);
	std::cout<<"The integral2 value is : "<<intv2<<std::endl;

	return 0;
}
