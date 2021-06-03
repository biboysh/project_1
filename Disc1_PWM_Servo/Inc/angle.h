#ifndef __ANGLE
#define __ANGLE
#include <math.h>

#define PI 3.141592
#define rad_deg(x) (double)x * 180.0 / PI //rad���� deg������ ��ȯ
#define deg_rad(x) (double)x / 180.0 * PI //deg������ rad������ ��ȯ

#define L1 160.0  //���ʾ��� ����(mm���� �ƴ� m���� ���)
#define L2 98.5  //�ٱ��� ���� ����

typedef double element;

typedef struct deg { //�� ������ ������ ǥ��
	element seta1; //���� ������ ����
	element seta2; //�ٱ��� ������ ����
}DEG;

typedef struct xy {
	element x; //�ܺη� ���� ���� xy�� ��ǥ�� �� x
	element y; //�ܺη� ���� ���� xy�� ��ǥ�� �� y
}XY;

//1:���� ����, 2: �ٱ��� ����

DEG xy_ang(element x, element y) //mm������ ����,���� �Է�->�� ������ ���������� ��ȯ
{
	element c2 = 0, s2 = 0;
	
	DEG res;
	res.seta1 = 0;
	res.seta2 = 0;

	c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2); //cos(�ٱ����Ͱ���)
	s2 = sqrt(1 - c2 * c2); //cos(���ʸ��Ͱ���)
	
	res.seta2 = atan2(s2 , c2); //atan(sin/cos) = ������
	
	//res.seta2 = atan2(s2 , c2);

	res.seta1 = atan2(y , x) - atan2((L2 * s2) , (L1 + L2 * c2)); //Seta1�� ���

	//if (res.seta1 < 0) //seta1�������� ������ ������ sin(seta2)���� ������ ��ȯ���� seta1�� ����� �������� ���
	//{
          //s2 = -s2;
          //res.seta2 = atan2(s2, c2);
          //res.seta1 = atan2(y, x) - atan2((L2 * s2), (L1 + L2 * c2)); //Seta1�� ���
	//}

	return res; //������ ��ȯ
}
XY deg_to_p1(element seta1) //rad�� ���� ù��° ������ �Է�
{
	XY axl;
	axl.x = L1 * cos(seta1);
	axl.y = L1 * sin(seta1);

	return axl;
}//��ȯ�� m������ ��ȯ

XY deg_to_p2(element seta1, element seta2)
{
	XY axl;
	axl.x = L1 * cos(seta1) + L2 * cos(seta1 + seta2);
	axl.y = L1 * sin(seta1) + L2 * sin(seta1 + seta2);

	return axl; //��ȯ�� m������ ��ȯ
}

#endif