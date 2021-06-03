#ifndef __ANGLE
#define __ANGLE
#include <math.h>

#define PI 3.141592
#define rad_deg(x) (double)x * 180.0 / PI //rad값을 deg단위로 변환
#define deg_rad(x) (double)x / 180.0 * PI //deg단위를 rad값으로 변환

#define L1 160.0  //안쪽암의 길이(mm단위 아닌 m단위 사용)
#define L2 98.5  //바깥쪽 암의 길이

typedef double element;

typedef struct deg { //각 모터의 각도값 표현
	element seta1; //안쪽 모터의 각도
	element seta2; //바깥쪽 모터의 각도
}DEG;

typedef struct xy {
	element x; //외부로 부터 들어온 xy계 좌표값 중 x
	element y; //외부로 부터 들어온 xy계 좌표값 중 y
}XY;

//1:안쪽 모터, 2: 바깥쪽 모터

DEG xy_ang(element x, element y) //mm단위로 가로,세로 입력->각 모터의 각도값으로 변환
{
	element c2 = 0, s2 = 0;
	
	DEG res;
	res.seta1 = 0;
	res.seta2 = 0;

	c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2); //cos(바깥모터각도)
	s2 = sqrt(1 - c2 * c2); //cos(안쪽모터각도)
	
	res.seta2 = atan2(s2 , c2); //atan(sin/cos) = 각도값
	
	//res.seta2 = atan2(s2 , c2);

	res.seta1 = atan2(y , x) - atan2((L2 * s2) , (L1 + L2 * c2)); //Seta1값 계산

	//if (res.seta1 < 0) //seta1각도값이 음수가 나오면 sin(seta2)값을 음수로 전환시켜 seta1가 양수가 나오도록 계산
	//{
          //s2 = -s2;
          //res.seta2 = atan2(s2, c2);
          //res.seta1 = atan2(y, x) - atan2((L2 * s2), (L1 + L2 * c2)); //Seta1값 계산
	//}

	return res; //각도값 반환
}
XY deg_to_p1(element seta1) //rad로 계산된 첫번째 관절값 입력
{
	XY axl;
	axl.x = L1 * cos(seta1);
	axl.y = L1 * sin(seta1);

	return axl;
}//반환은 m단위로 반환

XY deg_to_p2(element seta1, element seta2)
{
	XY axl;
	axl.x = L1 * cos(seta1) + L2 * cos(seta1 + seta2);
	axl.y = L1 * sin(seta1) + L2 * sin(seta1 + seta2);

	return axl; //반환은 m단위로 반환
}

#endif