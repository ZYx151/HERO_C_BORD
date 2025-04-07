#include "RMQueue.h"

RM_Status RMQueueInit(RMQueue_Handle *handle, uint32_t typeSize, uint32_t depth) {
    handle->dataPtr = (void **) RMLIB_MALLOC(sizeof(void **) * depth);
    if (handle->dataPtr == NULL) return RM_ERROR;
    for (int i = 0; i < depth; i++) {
        handle->dataPtr[i] = RMLIB_MALLOC(typeSize);
        if (handle->dataPtr[i] == NULL) {
            for (int j = 0; j < i; j++)
                RMLIB_FREE(handle->dataPtr[j]);
            return RM_ERROR;
        }
    }
    handle->typeSize = typeSize;
    handle->fifoSize = depth;
    handle->head = 0;
    handle->end = 0;
    handle->size = 0;
    handle->Lock = 0;
    return RM_SUCCESS;
}

RM_Status RMQueuePush(RMQueue_Handle *handle, void *dataPtr) {
	if(handle->size == handle->fifoSize)  {
     /* 队列头索引前移动,相当于抛弃前一个位置的数据,被抛弃的位置稍后会被写入新的数据 */
		handle->head = (handle->head + 1) % handle->fifoSize;
		handle->size--;/* 相当于出列 size-1 */ 
		}
    if ( handle->Lock == 0) {
        memcpy(handle->dataPtr[handle->head], dataPtr, handle->typeSize);
        handle->end = (handle->end + 1) % handle->fifoSize;
        handle->size++;
        return RM_SUCCESS;
    } else return RM_ERROR;
}

void *RMQueueTop(RMQueue_Handle *handle) {
    if (handle->size != 0)
        return handle->dataPtr[handle->head];
    else return NULL;
}

void *RMQueuePop(RMQueue_Handle *handle) {
    if (handle->size != 0) {
      void *tmp = handle->dataPtr[handle->head];
        handle->head = (handle->head + 1) % handle->fifoSize;
        handle->size--;
        return tmp;
    } else return NULL;
}

uint8_t RMQueuePop_sub(RMQueue_Handle *handle, void *tmp) {
    if (handle->size == 0) return 0;
    memcpy(tmp, handle->dataPtr[handle->head], handle->typeSize);
    handle->head = (handle->head++) % handle->fifoSize; // 队列头索引增加
	handle->size--;                                 // pop 一个数据, 长度减一
    return 1;
}

void RMQueueDelete(RMQueue_Handle *handle) {
    for (int i = 0; i < handle->fifoSize; i++)
        RMLIB_FREE(handle->dataPtr[i]);
    RMLIB_FREE(handle->dataPtr);
}

void *RMQueueGetEndPtr(RMQueue_Handle *handle) {
    if (handle->size < handle->fifoSize && handle->Lock == 0) {
        handle->Lock = 1;
        return handle->dataPtr[handle->end];
    } else return NULL;
}

RM_Status RMQueuePushEndPtr(RMQueue_Handle *handle) {
    if (handle->Lock == 1) {
        handle->Lock = 0;
        handle->end = (handle->end + 1) % handle->fifoSize;
        handle->size++;
        return RM_SUCCESS;
    } else return RM_ERROR;
}
