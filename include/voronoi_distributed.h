#include <iostream>
#include <vector>
#include <cmath>

#define TOL 0.000000001

// structure to describe a line using two points (x1,y1) and (x2,y2)
struct linedesc {
	double x1;
	double y1;
	double x2;
	double y2;
};

double euclideandistance2d(double& x1,double& y1,double& x2,double& y2)
{
	return sqrt(((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)));
}

void perpbisector(struct linedesc line1, struct linedesc *bis)
{
	double xnew1 = (line1.x1+line1.x2)/2;
	double ynew1 = (line1.y1+line1.y2)/2;

	double xnew2, ynew2;
	double m;
	// slope of the perpendicular bisector
	if(((line1.x2 - line1.x1)<TOL) && ((line1.x2 - line1.x1)>-TOL)) {
		m = 0;	
		xnew2 = xnew1 + 0.5;
		ynew2 = ynew1;
	}
	else {
		if(((line1.y2 - line1.y1)<TOL) && ((line1.y2 - line1.y1)>-TOL)) {
			// slope zero for line1 and consequently the perp bisector has slope infinity
			ynew2 = ynew1 + 0.5;
			xnew2 = xnew1;
		}
		else {
			double m1 = (line1.y2 - line1.y1)/(line1.x2 - line1.x1);
			m = -1/m1;
			if((m<=1.0) && (m>=-1.0)) {
				xnew2 = xnew1 + 0.5;
				ynew2 = (xnew1*(line1.x1-line1.x2) + ynew1*(line1.y1-line1.y2) - xnew2*(line1.x1-line1.x2))/(line1.y1-line1.y2);
			}
			else {
				ynew2 = ynew1 + 0.5;
				xnew2 = ((ynew2-ynew1) + m*xnew1)/m;
			}
		}
	}

	bis->x1 = xnew1;
	bis->y1 = ynew1;
	bis->x2 = xnew2;
	bis->y2 = ynew2;
}

void sortsites(int j, int n, std::vector<double>& distancetoagents, std::vector<double>& sortedxsites, std::vector<double>& sortedysites)
{
	double temp;
	// sortedxsites and sortedysites are of length (n-1)
	for(int i=0; i<n-2; i++) {
		for(int k=i+1; k<n-1; k++) {

			if(distancetoagents[i]>distancetoagents[k]) {
				temp = distancetoagents[i];	
				distancetoagents[i] = distancetoagents[k];
				distancetoagents[k] = temp;

				temp = sortedxsites[i];	
				sortedxsites[i] = sortedxsites[k];
				sortedxsites[k] = temp;

				temp = sortedysites[i];	
				sortedysites[i] = sortedysites[k];
				sortedysites[k] = temp;
			}
		}
	}
}

// find intersection of two lines assuming they intersect ie. they are not parallel
void computeintersection(struct linedesc line1, struct linedesc line2, double& ix, double& iy)
{
	int condition1 = 0;
	int condition2 = 0;

	// check if lines have slope infinity
	if(((line1.x2 - line1.x1)<TOL) && ((line1.x2 - line1.x1)>-TOL)) {
		condition1 = 1;
	}
	if(((line2.x2 - line2.x1)<TOL) && ((line2.x2 - line2.x1)>-TOL)) {
		condition2 = 1;
	}

	double m1, m2;
	if(condition1==1 || condition2==1) {
	
		// both conditions will not be 1 at the same time since it is assumed that the lines are not parallel
		if(condition1==1) {
			m2 = (line2.y2 - line2.y1)/(line2.x2 - line2.x1);
			ix = line1.x1;
			iy = line2.y1 + m2*(ix-line2.x1);
		}
		else {  // condition2 = 1
			m1 = (line1.y2 - line1.y1)/(line1.x2 - line1.x1);
			ix = line2.x1;
			iy = line1.y1 + m1*(ix-line1.x1);
		}
	}
	else {
		// slopes of the two lines
		m1 = (line1.y2 - line1.y1)/(line1.x2 - line1.x1);
		m2 = (line2.y2 - line2.y1)/(line2.x2 - line2.x1);

		// computing the intersection
		ix = (m2*line2.x1 - m1*line1.x1 + line1.y1 - line2.y1)/(m2-m1);
		iy = line1.y1 + m1*(ix-line1.x1);
	}

}

// intersection of a line with polygon defined by its vertices
void lineintersection(std::vector<double>& borderx, std::vector<double>& bordery, struct linedesc line, double myx, double myy)
{
	double m;
	int side;

	int condition = 0;
	// case where slope is infinity
	if(((line.x2 - line.x1)<TOL) && ((line.x2 - line.x1)>-TOL)) {
		condition = 1;
	}
	
	if(condition==1) {
		if((myx-line.x1)>0) {
			side = 1;
		}
		else {
			side = -1;
		}
	}
	else {
	
		// slope of the line
		m = (line.y2 - line.y1)/(line.x2 - line.x1);

		// determine which side of the line the point is
		if(((myy-line.y1) - m*(myx-line.x1)) > 0) {
			side = 1;
		}
		else {
			side = -1;
		}
	}

	std::vector<double> borderxtmp(borderx);
	std::vector<double> borderytmp(bordery);

	int n = static_cast<int>(borderx.size());
	// determine the polygon vertices which lie on the same side as the point
	std::vector<int> ind(n,0);
	int nochange = 1;
	for(int i=0; i<n; i++) {
	
		double currx = borderx[i];
		double curry = bordery[i];
		switch(side)	
		{
			case 1:
				if(condition==1) {
					if((currx-line.x1)>=0) {
						ind[i] = 1;
					}
					else {
						nochange = 0;
					}
				}
				else {
					if(((curry-line.y1) - m*(currx-line.x1)) >= 0) {
						ind[i] = 1;
					}
					else {
						nochange = 0;
					}
				}
				break;
			case -1:
				if(condition==1) {
				
					if((currx-line.x1)<=0) {
						ind[i] = 1;
					}
					else {
						nochange = 0;
					}
				}
				else {
					if(((curry-line.y1) - m*(currx-line.x1)) <= 0) {
						ind[i] = 1;
					}
					else {
						nochange = 0;
					}
				}
				break;
		}
	}

	// if no changes to be done, return from the function
	if(nochange==1) {
		return;
	}

	int i1,i2;
	double ix, iy;
	struct linedesc line2;
	if(ind[0]==0) {
	
		for(int i=0; i<n-1; i++) {
		
			if((ind[i]==0) && (ind[i+1]==1)) {
			
				i1 = i;

				// compute intersection
				line2.x1 = borderx[i];
				line2.y1 = bordery[i];
				line2.x2 = borderx[i+1];
				line2.y2 = bordery[i+1];
				computeintersection(line, line2, ix, iy);
				
				// update the boundary variables
				borderxtmp[i] = ix;
				borderytmp[i] = iy;
			}
			if((ind[i]==1) && (ind[i+1]==0)) {
			
				i2 = i+1;

				// compute intersection
				line2.x1 = borderx[i];
				line2.y1 = bordery[i];
				line2.x2 = borderx[i+1];
				line2.y2 = bordery[i+1];
				computeintersection(line, line2, ix, iy);
				
				// update the boundary variables
				borderxtmp[i+1] = ix;
				borderytmp[i+1] = iy;
			}
		}
		if(ind[n-1]==1) {
		
			i2 = 0;
	
			// compute intersection
			line2.x1 = borderx[n-1];
			line2.y1 = bordery[n-1];
			line2.x2 = borderx[0];
			line2.y2 = bordery[0];
			computeintersection(line, line2, ix, iy);
				
			// update the boundary variables
			if(i1==i2) {
				borderxtmp.push_back(ix);
				borderytmp.push_back(iy);
			}
			else {
				borderxtmp[0] = ix;
				borderytmp[0] = iy;
			}

			if((i2+1)<(i1)) {
			borderxtmp.erase(borderxtmp.begin() + (i2 + 1), borderxtmp.begin() + (i1));
			borderytmp.erase(borderytmp.begin() + (i2 + 1), borderytmp.begin() + (i1));
			}
			
		}
		else
		{
			if(0<(i1)) {
			borderxtmp.erase(borderxtmp.begin(), borderxtmp.begin() + (i1));
			borderytmp.erase(borderytmp.begin(), borderytmp.begin() + (i1));
			}

			if((i2+1)<=(n-1)) {
			borderxtmp.erase(borderxtmp.begin() + (i2+1), borderxtmp.end());
			borderytmp.erase(borderytmp.begin() + (i2+1), borderytmp.end());
			}
		}
		
	}
	else {
	
		if(ind[n-1]==1) {
		
			for(int i=0; i<n-1; i++) {
			
				if((ind[i]==1) && (ind[i+1]==0)) {
				
					i1 = i+1;

					// compute intersection
					line2.x1 = borderx[i];
					line2.y1 = bordery[i];
					line2.x2 = borderx[i+1];
					line2.y2 = bordery[i+1];
					computeintersection(line, line2, ix, iy);
				
					// update the boundary variables
					borderxtmp[i+1] = ix;
					borderytmp[i+1] = iy;
				}
				if((ind[i]==0) && (ind[i+1]==1)) {

					i2 = i;

					// compute intersection
					line2.x1 = borderx[i];
					line2.y1 = bordery[i];
					line2.x2 = borderx[i+1];
					line2.y2 = bordery[i+1];
					computeintersection(line, line2, ix, iy);
				
					// update the boundary variables
					if(i1==i2) {
						//borderxtmp.push_back(ix);
						//borderytmp.push_back(iy);
						borderxtmp.insert(borderxtmp.begin()+(i+1),ix);
						borderytmp.insert(borderytmp.begin()+(i+1),iy);
					}
					else {
						borderxtmp[i] = ix;
						borderytmp[i] = iy;
					}
				}
			}

			// remove elements from i1 to i2
			if((i1+1)<(i2)) {
				borderxtmp.erase(borderxtmp.begin() + (i1+1), borderxtmp.begin() + (i2));
				borderytmp.erase(borderytmp.begin() + (i1+1), borderytmp.begin() + (i2));
			}

		}
		else {

			for(int i=0; i<n-1; i++) {

				if((ind[i]==1) && (ind[i+1]==0)) {

					i1 = i+1;

					// compute intersection
					line2.x1 = borderx[i];
					line2.y1 = bordery[i];
					line2.x2 = borderx[i+1];
					line2.y2 = bordery[i+1];
					computeintersection(line, line2, ix, iy);
				
					// update the boundary variables
					borderxtmp[i+1] = ix;
					borderytmp[i+1] = iy;
				}
			}

			i2 = n-1;

			// compute intersection
			line2.x1 = borderx[0];
			line2.y1 = bordery[0];
			line2.x2 = borderx[n-1];
			line2.y2 = bordery[n-1];
			computeintersection(line, line2, ix, iy);
				
			// update the boundary variables
			if(i1==i2) {
				borderxtmp.push_back(ix);
				borderytmp.push_back(iy);
			}
			else {
				borderxtmp[n-1] = ix;
				borderytmp[n-1] = iy;

				// remove elements from i1 to i2
				if((i1+1)<(i2)) {
					borderxtmp.erase(borderxtmp.begin()+i1+1,borderxtmp.begin()+i2);
					borderytmp.erase(borderytmp.begin()+i1+1,borderytmp.begin()+i2);
				}
			}
		

		}
	}
	borderx = borderxtmp;
	bordery = borderytmp;
}

void compute_voronoi(int j, int n, std::vector<double> xborder, std::vector<double> yborder, std::vector<double> xsites, std::vector<double> ysites, std::vector<double>& xvert, std::vector<double>& yvert)
{
	double myx = xsites[j];
	double myy = ysites[j];

	std::vector<double> sortedxsites(xsites);
	std::vector<double> sortedysites(ysites);
	// erase the jth element
	sortedxsites.erase(sortedxsites.begin()+j);
	sortedysites.erase(sortedysites.begin()+j);

	// sort elements based on distance to agent j
	std::vector<double> distancetoagents(n-1);
	for(int i=0; i<n-1; i++) {
		distancetoagents[i] = pow((myx - sortedxsites[i]),2) + pow((myy - sortedysites[i]),2);
	}
	sortsites(j, n, distancetoagents, sortedxsites, sortedysites);

	std::vector<double> borderx = xborder;
	std::vector<double> bordery = yborder;

	struct linedesc bis, curr;
	curr.x1 = myx;
	curr.y1 = myy;

	// loop over the sites other than j
	for(int i=0; i<n-1; i++) {

		double xcur = sortedxsites[i];
		double ycur = sortedysites[i];
		//if(i!=j)
		//{
		curr.x2 = xcur;
		curr.y2 = ycur;
		perpbisector(curr, &bis);
		lineintersection(borderx, bordery, bis, myx, myy);
		//}
	}
	xvert = borderx;
	yvert = bordery;

}

