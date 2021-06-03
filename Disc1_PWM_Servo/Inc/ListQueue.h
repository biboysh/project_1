#pragma once
#include <stdlib.h>
#include <string.h>

typedef enum {
	NOT_EMPTY	=	01U,
	EMPTY	=	00U
}Queue_Type;

typedef struct QNode {
	char str[15];
	struct QNode* next;
}QNode;

typedef struct ListQueue {
	QNode* front, * rear;
}ListQueue;


void Init_Queue(ListQueue *samp)
{
	samp->front = samp->rear = (QNode*)malloc(sizeof(QNode));
	samp->front = samp->rear = NULL;
}

void insert(ListQueue* samp, char string[])
{
	if (samp->front == NULL)
	{
		QNode* temp = (QNode*)malloc(sizeof(QNode));
		strcpy(temp->str, string);
		temp->next = NULL;
		samp->front = samp->rear = temp;
	}
	else
	{
		QNode* temp = (QNode*)malloc(sizeof(QNode));
		strcpy(temp->str, string);
		temp->next = NULL;
		samp->rear->next = temp;
		samp->rear = temp;
	}
}

char* delete_front(ListQueue* samp)
{
	char cpystr[30];
	if (samp->front == NULL)
	{  
		strcpy(cpystr, "0 0 1");
		return cpystr;
	}
	QNode* temp = samp->front;
	
	strcpy(cpystr, temp->str);
	samp->front = samp->front->next;
	free(temp);

	return cpystr;
}


Queue_Type is_empty(ListQueue* q)
{
	if (q->front == NULL) return EMPTY;
	return NOT_EMPTY;
}
