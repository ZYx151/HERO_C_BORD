#include "RM_Cilent_Ul.h"
#include "CRC.h"

uint8_t UI_Seq  ;   
uint8_t UI_BUFF[127];

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/
void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer)
{
    UI_Packhead     framehead;
    UI_Data_Operate datahead;
    UI_Data_Delete  del;
    
    /* 填充帧头数据 */
    framehead.SOF          = UI_SOF;
    framehead.Data_Length  = 8;
    framehead.Seq          = UI_Seq;
    framehead.CRC8         = get_CRC8_check_sum((uint8_t *)&framehead, 4, 0xff);
    framehead.CMD_ID       = UI_CMD_Robo_Exchange;
    
    /* 填充子帧头数据 */
    datahead.Data_ID     = UI_Data_ID_Del;
    datahead.Sender_ID   = Robot_ID;
    datahead.Receiver_ID = Cilent_ID;
    
    /* 控制信息 */
    del.Delete_Operate = Del_Operate;
    del.Layer          = Del_Layer;
   
    /* 将数据写入缓冲区 */
    memset (UI_BUFF,         0,                    sizeof(UI_BUFF));
	 memcpy (UI_BUFF,         (uint8_t*)&framehead, sizeof(framehead));
	 memcpy (UI_BUFF + 7,     (uint8_t*)&datahead,  sizeof(datahead));
	 memcpy (UI_BUFF + 7 + 6, (uint8_t*)&del,       sizeof(del));
	 *(uint16_t *)(UI_BUFF + 7 + 6 + 2) = get_CRC16_check_sum(UI_BUFF, 7 + 6 + 2, 0xffff);
	 
	 HAL_UART_Transmit_DMA (&huart6, UI_BUFF, 7 + 6 + 2 + 2);
     UI_Seq++;                                                                //包序号+1
}

/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/      
void Line_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
   for(uint8_t i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
void Rectangle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
   for(uint8_t i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
void Circle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Radius)
{
   for(uint8_t i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t x_Length,uint32_t y_Length)
{
   for(uint8_t i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}

/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
void Float_Draw(Float_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Float)
{ 
   for(uint8_t i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = Graph_Float;
}

/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/   
void Char_Draw(String_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,char *Char_Data)
{
   for(uint8_t i=0;i<3&&imagename[i]!='\0';i++)
      image->Graph_Control.graphic_name[2-i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(uint8_t i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Graph_ReFresh(int cnt,...)
{
    UI_Packhead     framehead;//帧头
    UI_Data_Operate datahead; //子帧头（数据段中）
    Graph_Data      imageData;//图形（数据段中）
 
    va_list ap; 
    va_start(ap,cnt);
    
    /* 填充帧头+CMD */
   framehead.SOF            = UI_SOF;
   framehead.Data_Length    = 6 + cnt * 15;
   framehead.Seq            = UI_Seq;
   framehead.CRC8           = get_CRC8_check_sum((uint8_t *)&framehead, 4, 0xff);
   framehead.CMD_ID         = UI_CMD_Robo_Exchange;
   /* 填充子帧头 */
   switch(cnt){
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
   datahead.Sender_ID   = Robot_ID;
   datahead.Receiver_ID = Cilent_ID;
   
   /* 将数据写入缓冲区 */
   memset (UI_BUFF,      0,                   sizeof(UI_BUFF));
   memcpy (UI_BUFF,     (uint8_t*)&framehead, sizeof(framehead));
   memcpy (UI_BUFF + 7, (uint8_t *)&datahead, sizeof(datahead));
   for(uint8_t i=0;i<cnt;i++){
      imageData=va_arg(ap,Graph_Data);
      memcpy (UI_BUFF + 7 + 6 + i * 15, (uint8_t*)&imageData, sizeof(imageData));
   }
   *(uint16_t *)(UI_BUFF + 7 + 6 + cnt * 15) = get_CRC16_check_sum(UI_BUFF, 13 + cnt * 15, 0xffff);
   
   HAL_UART_Transmit_DMA (&huart6, UI_BUFF, 7 + 6 + cnt * 15 + 2); 
	 
   va_end(ap);
   
   UI_Seq++;     //包序号+1
   return 0;
}

/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
    UI_Packhead     framehead;
    UI_Data_Operate datahead;
    /* 填充帧头+CMD */
    framehead.SOF         = UI_SOF;
    framehead.Data_Length = 6 + 45;
    framehead.Seq         = UI_Seq;
    framehead.CRC8        = get_CRC8_check_sum((uint8_t*)&framehead,4, 0xff);
    framehead.CMD_ID      = UI_CMD_Robo_Exchange;
   /* 填充子帧头 */
    datahead.Data_ID     = UI_Data_ID_DrawChar;
    datahead.Sender_ID   = Robot_ID;
    datahead.Receiver_ID = Cilent_ID;
   /* 将数据写入缓冲区 */
    memset (UI_BUFF,            0,                      sizeof(UI_BUFF ));
    memcpy (UI_BUFF,            (uint8_t*)&framehead,   sizeof(framehead));
    memcpy (UI_BUFF + 7,        (uint8_t*)&datahead,    sizeof(datahead));
    memcpy (UI_BUFF + 7 + 6,    (uint8_t*)&string_Data, sizeof(String_Data));
    *(uint16_t *)(UI_BUFF + 7 + 6 + 45) = get_CRC16_check_sum(UI_BUFF, 7 + 6 + 45, 0xffff);
	
    HAL_UART_Transmit_DMA (&huart6, UI_BUFF, 7 + 6 + 45 + 2);
    
   UI_Seq++;                                                         //包序号+1
   return 0;
}

void RADA_SEND_UI()
{
    UI_Packhead     framehead;
    UI_Data_Operate datahead;
    /* 填充帧头+CMD */
    framehead.SOF         = UI_SOF;
    framehead.Data_Length = 6 + 45;
    framehead.Seq         = UI_Seq;
    framehead.CRC8        = get_CRC8_check_sum((uint8_t*)&framehead,4, 0xff);
    framehead.CMD_ID      = UI_CMD_Robo_Exchange;
   /* 填充子帧头 */
    datahead.Data_ID     = UI_Data_ID_RADA;
    datahead.Sender_ID   = Robot_ID;
    datahead.Receiver_ID = Cilent_ID;
   /* 将数据写入缓冲区 */

    *(uint16_t *)(UI_BUFF + 7 + 6 + 45) = get_CRC16_check_sum(UI_BUFF, 7 + 6 + 45, 0xffff);
	
    HAL_UART_Transmit_DMA (&huart6, UI_BUFF, 7 + 6 + 45 + 2);
    
   UI_Seq++;                                                         //包序号+1
}
