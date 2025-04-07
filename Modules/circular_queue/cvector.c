#include "cvector.h"
#include "RMLibHead.h"

/**
 * 创建数组
 * size: 数组大小
**/ 
cvector* cvector_create(const size_t size)
{
	cvector* cv = (cvector *)RMLIB_MALLOC(sizeof(cvector ));
	
	if(!cv) return 0;
	
	cv->cv_pdata = RMLIB_MALLOC(MIN_LEN * size);
	if(!cv->cv_pdata) {
		free(cv);
		return 0;
	}
	
	cv->cv_size = size;  // 每个单位的大小
	cv->cv_tot_len = MIN_LEN;
	cv->cv_len = 0;
	return cv;
}

/* 删除数组 */
void cvector_destroy(cvector* cv)
{
	free(cv->cv_pdata);
	free(cv);
}

size_t cvector_length(cvector* cv) {
	return cv->cv_len;
}

void* cvector_pushback(cvector* cv, void* memb)
{
	if(cv->cv_len >= cv->cv_tot_len) {
		// 内存不足时以 cv->cv_tot_len 为最小单位扩张数组
		cv->cv_tot_len <<= EXPANED_VAL;
		cv->cv_pdata = realloc(cv->cv_pdata, cv->cv_tot_len * cv->cv_size);
	}
	
	memcpy((char* )cv->cv_pdata + cv->cv_len * cv->cv_size, memb, cv->cv_size);
    cv->cv_len++;

	return (void *)(cv->cv_pdata + (cv->cv_len - 1) * cv->cv_size);
}

void* cvector_val_at(cvector* cv, size_t index) {
	return cv->cv_pdata + index * cv->cv_size;
}
