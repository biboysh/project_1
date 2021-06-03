#pragma once
/*
1차 방정식 만들기
*/
//#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include "angle.h"

#define S_ERROR -1000

#define INIT_ANG 45.0
#define MIN_ANG 500
#define MAX_ANG MIN_ANG + 2000
#define RESOLUTION (MAX_ANG - MIN_ANG)/180.0
#define ANG_SIZE 7

#define START_X -146.0
#define START_Y 32.0
#define SCALE 0.42

extern XY pos;


double make_slope(XY* a, XY* b)
{
	if ((a->x - b->x) == 0) return S_ERROR;
	return (a->y - b->y) / (a->x - b->x);
}

double make_constant(XY* a, XY* b, double* slope)
{
	return a->y - a->x * (*slope);
}

XY convert(XY pos)
{
  pos.x *= ( SCALE );
  pos.y *= ( SCALE );
  pos.x += START_X;
  pos.y += START_Y;
  return pos;
}

void def_CCR1(double angle)
{
  angle = 180.0 - angle;
  double res = RESOLUTION;
  TIM2->CCR1 = (int)(MIN_ANG + angle*res);
}

void def_CCR2(double angle)
{ 
  double res = RESOLUTION;
  TIM2->CCR2 = (int)(MIN_ANG + angle*res);
 }

//double return_yloc(XY* a, XY* b, double* x_new)
//{
//	return *x_new * make_slope(a, b) + make_constant(a, b);
//}

//사용할 함수
//XY* a는 이전에 그려진 좌표 b는 뒤에 호출된 자표
XY point;
DEG ang;

void move_arm(double *x, double *y)
{
  point.x = *x;
  point.y = *y;
  point = convert(point);
  ang = xy_ang(point.x,point.y);
  ang.seta1 = INIT_ANG + rad_deg(ang.seta1);
  ang.seta2 = /*-90.0 +*/ rad_deg(ang.seta2);
  def_CCR1(ang.seta1);
  def_CCR2(ang.seta2);
}

void draw_line(XY* a, XY* b)
{
	double slope, constant;
	double x = 0;
	double y = 0;
      
//	double adder = 0.1;
	slope = make_slope(a, b);
	if (slope == S_ERROR) //기울기가 무한대일때
	{
		if (a->y > b->y)
			for (y = a->y; y >= b->y; y -= 0.1)
			{
				x = a->x;
				/*
				x,y좌표로 행할 행위 지정
				*/
                                move_arm(&x,&y);
			}
		else
			for (y = a->y; y <= b->y; y += 0.1)
			{
				x = a->x;
				/*
				x,y좌표로 행할 행위 지정
				*/
                                move_arm(&x,&y);
			}
		return;
	}
	constant = make_constant(a, b, &slope);
	if (a->x > b->x)
		for (x = a->x; x >= b->x; x -= 0.1)
		{
			y = (x * slope + constant);
			/*
			x,y좌표로 행할 행위 지정
			*/
                        move_arm(&x,&y);
		}
	else
		for (x = a->x; x <= b->x; x += 0.1)
		{
			y = (x * slope + constant);
			/*
			x,y좌표로 행할 행위 지정
			*/
                        move_arm(&x,&y);
		}
}

