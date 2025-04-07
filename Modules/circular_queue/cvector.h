#ifndef __CVECTOR_H
#define __CVECTOR_H

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define MIN_LEN 5
#define EXPANED_VAL 1

/** 动态数组  **/
/**
cv_len : 数组中元素个数
cv_size: 单个元素大小(字节)
cv_tot_len: 数组总大小(字节)
**/
#pragma pack(1)
typedef struct {
	void* cv_pdata;
	size_t cv_len;
	size_t cv_tot_len;
	size_t cv_size;
}cvector;
#pragma pack()

cvector* cvector_create(const size_t size);
void cvector_destroy(cvector* cv);
size_t cvector_length(cvector* cv);
void* cvector_pushback(cvector* cv, void* memb);
void* cvector_val_at(cvector* cv, size_t index);

#endif

