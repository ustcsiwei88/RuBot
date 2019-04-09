#include<iostream>
#include<vector>
#include<cmath>
#include<Eigen/Dense>

using namespace Eigen;
using namespace std;

double PI = std::acos(-1);
std::vector<double> a{0, -0.612, -0.5723, 0, 0, 0};
std::vector<double> d{0.1273, 0, 0, 0.163941, 0.1157, 0.0922};
std::vector<double> alpha{PI/2, 0, 0, PI/2, -PI/2, 0};

void kinematic(vector<double> theta){
	Matrix4d Z[6];
	Matrix4d X[6];
	for(int i=0; i<6; i++){
		Z[i] << cos(theta[i]), -sin(theta[i]), 0, 0, sin(theta[i]), cos(theta[i]),
				0, 0, 0, 0, 1, d[i], 0, 0, 0, 1;
		X[i] << 1, 0, 0, a[i], 0, cos(alpha[i]), -sin(alpha[i]), 0,
				0, sin(alpha[i]), cos(alpha[i]), 0, 0, 0, 0, 1;
	}
	Matrix4d res = Z[0]*X[0];
	for(int i=1; i<6; i++){
		res *= Z[i] * X[i];
	}
	std::cout << res << std::endl;
}

void invkinematic(vector<double> pos){
	vector<double>
	double x, y;
	double h;
	
	double tmp = sqrt(x*x + y*y - d[3]*d[3] + h*h);
	theta[2] = acos((a[1]*a[1]+a[2]*a[2]-tmp)/(2*a[1]*a[2]));
	theta[1] = asin(h/tmp) + acos((a1*a1+tmp*tmp-a2*a2)/(2*a1*tmp));
	theta[3] = acos(h/tmp) + acos((a2*a2+tmp*tmp-a1*a1)/(2*a2*tmp));
}
#ifndef MAIN_FILE
#define MAIN_FILE


int main(){
	kinematic(vector<double>{0,0,0,0,0,0});
	return 0;
}

#endif