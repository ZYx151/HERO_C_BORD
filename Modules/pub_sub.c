#include "pub_sub.h"
#include "stdlib.h"
#include "string.h"


/* message_center是fake head node,方便链表编写,这样就不需要处理链表头的特殊情况*/
static Publisher_t message_center = {
	.topic_name = "Message_Manager",
	.first_subs = NULL,
	.next_topic_node = NULL };

/* 如果队列为空,会返回0;成功获取数据,返回1;后续可以做更多的修改,比如剩余消息数目等 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
	if(sub->queue.size != 0) {      /* 读取队列头部指针 */
		return RMQueuePop_sub(&sub->queue, data_ptr);
	}
	else return 0;
//    if (sub->queue.size == 0) return 0;
//    memcpy(data_ptr, sub->queue.dataPtr[sub->queue.head], sub->queue.typeSize);
//    sub->queue.head = (sub->queue.head++) % QUEUE_SIZE; // 队列头索引增加
//	sub->queue.size--;                                 // pop 一个数据, 长度减一
    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
	static Subscriber_t *iter;
	iter = pub->first_subs; // iter作为订阅者指针,遍历订阅该话题的所有订阅者,如果为空说明遍历结束
   	// 遍历订阅了当前话题的所有订阅者,依次填入最新消息
   	while (iter) { //  如果队列已满,删除头部数据(最老的数据)
//   	   	if(iter->temp_size == QUEUE_SIZE) { 
//        iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
//        iter->temp_size--;  
		  // 将数据写入队列尾部
	RMQueuePush(&iter->queue, data_ptr);
//		memcpy(iter->queue[iter->front_idx], data_ptr, pub->data_len);
//        iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE; // 队列尾部前移
//		iter->temp_size++;
		
		iter = iter->next_subs_queue;  // 访问下一个订阅者
	}
	return 1;
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    Publisher_t *node = &message_center;
	while (node->next_topic_node) {
		node = node->next_topic_node;  // 切换到下一个发布者(话题)结点
		if (strcmp(node->topic_name, name) == 0) { //如果已经注册了相同的话题,直接返回该指针
			node->pub_registered_flag = 1;
			return node;
		}
	}// 遍历完发现尚未创建name对应的话题
   //  在链表尾部创建新的话题并初始化
	node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t	));
	node->next_topic_node->data_len = data_len;
	strcpy(node->next_topic_node->topic_name, name);
	node->pub_registered_flag = 1;
	
    return node->next_topic_node;
}

Subscriber_t *SubRegister(char *name, uint16_t typeSize)
{
	Publisher_t *pub = PubRegister(name, typeSize); // 用于查找或创建新的发布者
	// 创建新的订阅者结点,申请内存,注意要memset因为新空间不一定是空的,可能有之前留存的垃圾值
	Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
	memset(ret, 0, sizeof(Subscriber_t));
	// 对新键的Subscriber的队列进行初始化
//	ret->data_len = typeSize; // 设定数据长度
//	for (size_t i = 0; i < QUEUE_SIZE ; ++i)
//		ret->queue[i] = malloc (typeSize);
	RMQueueInit(&ret->queue, typeSize, QUEUE_SIZE);
    // 如果是第一个订阅者,特殊处理一下,将first_subs指针指向新建的订阅者(详见文档)
	if (pub->first_subs ==NULL) {
		pub->first_subs = ret;
		return ret;
	}
    // 若该话题已经有订阅者, 遍历订阅者链表,直到到达尾部
  	Subscriber_t *sub = pub->first_subs;  // 作为iterator
	while (sub->next_subs_queue) {     // 遍历订阅了该话题的订阅者链表
		sub = sub->next_subs_queue; 
	}
	sub->next_subs_queue = ret;
	return ret;
}


