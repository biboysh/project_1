#pragma once
#define MAX_DSIZE 10000
#include <string.h>
char rtr[15] = "0 0 1";

typedef enum {
  EMPTY,
  NOT_EMPTY,
  FULL,
  NOT_FULL
}Dq_check;

typedef struct {
  char element[MAX_DSIZE][15];
  int front, rear;
}Dequeue;

Dq_check is_full(Dequeue* dq)
{
  if ((dq->rear + 1) % MAX_DSIZE == dq->front) return FULL;
  return NOT_FULL;
}

Dq_check is_empty(Dequeue* dq)
{
  if (dq->rear == dq->front) return EMPTY;
  return NOT_EMPTY;
}

void init_dqueue(Dequeue* dq)
{
  dq->front = dq->rear = 0;
}

void insert(Dequeue* dq, char string[])
{
  if (is_full(dq) == FULL) return;
  dq->rear = (dq->rear + 1) % MAX_DSIZE;
  strcpy(dq->element[dq->rear], string);
}

char* delete_front(Dequeue* dq)
{
  if (is_empty(dq) == EMPTY)
  {
    strcpy(rtr,"0 0 1");
    return rtr;
  }
  dq->front = (dq->front + 1) % MAX_DSIZE;
  return dq->element[dq->front];
}
