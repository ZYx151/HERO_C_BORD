#include "UI.h"
#include "Referee_unpack.h"
#include "pub_sub.h"
#include "robot_def.h"

/* 1.????????? */
Graph_Data Shoot_Line[12];   //?????????  ???????
Graph_Data Scale_Line[28];

char String_1m[] = "1", String_3m[] = "3",String_5m[] = "5",String_7m[] = "7",String_9m[] = "9", String_Q[] = "Q",String_J[] = "J",String_B[] = "B";
String_Data Shoot_String[8];
Graph_Data Shoot_Rectangle[3];//???
extern Referee_data_t referee_send_data;
/* 2.???鷶Χ */
Graph_Data Camera_Rectangle;//????????????????
Graph_Data Camera_Centre;//????????
Graph_Data Scale_Line[28];
/* 3.???????? */
Graph_Data SuperCap_Rectangle;//??????
Graph_Data SuperCap_Line; //????

/* 4.????????? */
Graph_Data Chassis_Line[2];

/* 5.????λ?? */
Graph_Data Armor_Circle[4];

/* 6.???????? */
int  UI_sign = 0;
Float_Data Fric_NUM;
uint32_t Fric_Speed = 500;

int i = 0;

String_Data ChassisMode_String[4];
char String_Chassis[]  = "3:";
char String_Follow []  = "FOLL";
char String_Rotate []  = "RATE";
char String_Lock   []  = "LOCK";

String_Data GimbalMode_String[3];
char String_Head []  = "GIMBAL:";
char String_Front[]  = "FRONT";
char String_Back []  = "BACK";

String_Data ShootMode_String[4];
char String_Shoot [] = "SHOOT:";
char String_Stop  [] = "STOP";
char String_Ready [] = "READY";
char String_Stuck [] = "STUCKING";

String_Data AimMode_String[3];
char String_Aim [] = "AIM:";
char String_Off [] = "OFF";
char String_On  [] = "ON";

String_Data RPY[3];

String_Data Vision_d[3];

char String_d [] = "Vision_d:NUM";

char FRIC [] = "FRIC:SPEED";


Graph_Data Mode_Rectangle[5];

#define ROBOT_ID 4

#if ROBOT_ID == 3
char String_Lid [] = "Lid:";
String_Data LidMode_String[3];
#endif

/* ????????????UI???? */
void Task_UI(void *pvParameters)
{
//    Line_Draw(&Shoot_Line[0], "S1m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -100, X_CENTRE + 30, Y_CENTRE -100);
//    Line_Draw(&Shoot_Line[1], "S3m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -150, X_CENTRE + 30, Y_CENTRE -150);
//    Line_Draw(&Shoot_Line[2], "S5m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -200, X_CENTRE + 30, Y_CENTRE -200);
//    Line_Draw(&Shoot_Line[3], "S7m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -250, X_CENTRE + 30, Y_CENTRE -250);
//    Line_Draw(&Shoot_Line[4], "S9m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -300, X_CENTRE + 30, Y_CENTRE -300);
//    Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_ADD, 1, UI_Color_Yellow, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
    static portTickType currentTime;
    for (;;)
    {
        currentTime = xTaskGetTickCount();      // ????????
        if(referee_send_data.refree_status == Device_Online || UI_sign == 1){
            if(0 & 0x01)
                  UI_STATE = INIT;
              if(UI_STATE == INIT){
                  UI_Init();
                  UI_STATE = INITING;
              }
              else if(UI_STATE == MOVEING)
                  UI_Move();
//            UI_Delete(UI_Data_Del_ALL, 9);
//            UI_Delete(UI_Data_Del_ALL, 8);
//            UI_Delete(UI_Data_Del_ALL, 7);
//            UI_Delete(UI_Data_Del_ALL, 6);
//            UI_Delete(UI_Data_Del_ALL, 5);
//            UI_Delete(UI_Data_Del_ALL, 4);
//            UI_Delete(UI_Data_Del_ALL, 3);
//            UI_Delete(UI_Data_Del_ALL, 2);
//            UI_Delete(UI_Data_Del_ALL, 1);
//            UI_Delete(UI_Data_Del_ALL, 0);
//            Graph_ReFresh(7, Shoot_Line[0], Shoot_Line[1], Shoot_Line[2], Shoot_Line[3], Shoot_Line[4], Shoot_Line[5], SuperCap_Rectangle);
              UI_Refresh();
              UI_sign = 1; 
        }
        vTaskDelayUntil(&currentTime, 40);
    }
}

/* ???UI????????? */
void UI_Init()
{
    /* ????????? */
    uint8_t Shoot_Char_Size = 8;    //??????????
    uint8_t Shoot_Char_Width = 2;      //??????????
//    Fric_NUM.graph_Float = 1314;
#if ROBOT_ID == 1
    //T???????
    Line_Draw(&Shoot_Line[0], "HEN", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE - 400, Y_CENTRE, X_CENTRE + 400, Y_CENTRE);
    Line_Draw(&Shoot_Line[1], "SHU", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE, Y_CENTRE+500 , X_CENTRE, Y_CENTRE - 500 ); 
    
    //???
    Line_Draw(&Scale_Line[0], "000", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE-250, X_CENTRE + 40, Y_CENTRE-250);
    Line_Draw(&Scale_Line[1], "001", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE-200, X_CENTRE + 40, Y_CENTRE-200);
    Line_Draw(&Scale_Line[2], "002", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE-150, X_CENTRE + 40, Y_CENTRE-150);
    Line_Draw(&Scale_Line[3], "003", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE-100, X_CENTRE + 40, Y_CENTRE-100);
    Line_Draw(&Scale_Line[4], "004", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE-50,  X_CENTRE + 40, Y_CENTRE-50);
    Line_Draw(&Scale_Line[5], "005", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE+50,  X_CENTRE + 40, Y_CENTRE+50);
    Line_Draw(&Scale_Line[6], "006", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE+100,  X_CENTRE + 40,Y_CENTRE+100);
    Line_Draw(&Scale_Line[7], "007", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE+150, X_CENTRE + 40, Y_CENTRE+150);
    Line_Draw(&Scale_Line[8], "008", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE+200, X_CENTRE + 40, Y_CENTRE+200);
    Line_Draw(&Scale_Line[9], "009", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE , Y_CENTRE+250, X_CENTRE + 40, Y_CENTRE+250);
    
    Line_Draw(&Scale_Line[10], "010", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +400, Y_CENTRE, X_CENTRE +400, Y_CENTRE +40);
    Line_Draw(&Scale_Line[11], "011", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +350, Y_CENTRE, X_CENTRE +350, Y_CENTRE +40);
    Line_Draw(&Scale_Line[12], "012", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +300, Y_CENTRE, X_CENTRE +300, Y_CENTRE +40);
    Line_Draw(&Scale_Line[13], "013", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +250, Y_CENTRE, X_CENTRE +250, Y_CENTRE +40);
    Line_Draw(&Scale_Line[14], "014", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +200,  Y_CENTRE,  X_CENTRE +200,   Y_CENTRE +40);
    Line_Draw(&Scale_Line[15], "015", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +150,  Y_CENTRE,  X_CENTRE +150,   Y_CENTRE +40);
    Line_Draw(&Scale_Line[16], "016", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +100, Y_CENTRE, X_CENTRE +100, Y_CENTRE +40);
    Line_Draw(&Scale_Line[17], "017", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE +50, Y_CENTRE, X_CENTRE +50, Y_CENTRE +40);
    Line_Draw(&Scale_Line[18], "018", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -50, Y_CENTRE, X_CENTRE -50, Y_CENTRE +40);
    Line_Draw(&Scale_Line[19], "019", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -100, Y_CENTRE, X_CENTRE -100, Y_CENTRE +40);
    Line_Draw(&Scale_Line[20], "020", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -150, Y_CENTRE, X_CENTRE -150, Y_CENTRE +1);
    Line_Draw(&Scale_Line[21], "021", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -200, Y_CENTRE, X_CENTRE -200, Y_CENTRE +1);
    Line_Draw(&Scale_Line[22], "022", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -250, Y_CENTRE, X_CENTRE -250, Y_CENTRE +1);
    Line_Draw(&Scale_Line[23], "023", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -300, Y_CENTRE, X_CENTRE -300, Y_CENTRE +1);
    Line_Draw(&Scale_Line[24], "024", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -350, Y_CENTRE, X_CENTRE -350, Y_CENTRE +1);
    Line_Draw(&Scale_Line[25], "025", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE -400, Y_CENTRE, X_CENTRE -400, Y_CENTRE +1);
    Line_Draw(&Scale_Line[26], "026", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE, Y_CENTRE+300, X_CENTRE +40, Y_CENTRE+300);
    Line_Draw(&Scale_Line[27], "027", UI_Graph_ADD, 1, UI_Color_Purplish_red, 1, X_CENTRE, Y_CENTRE-300, X_CENTRE +40, Y_CENTRE-300);
    
    //???????  10m
    Rectangle_Draw(&Shoot_Rectangle[0], "QSZ", UI_Graph_ADD, 1, UI_Color_Cyan, 1, X_CENTRE+10, Y_CENTRE - 305, X_CENTRE + 90, Y_CENTRE - 345);//80  50   9M   10M???65  40
    Char_Draw(&Shoot_String[5], "CQS", UI_Graph_ADD, 1, UI_Color_Yellow, 15, sizeof(String_Q), Shoot_Char_Width, X_CENTRE + 50, Y_CENTRE - 330, String_Q);

    //?????? 7m
    Rectangle_Draw(&Shoot_Rectangle[1], "JDD", UI_Graph_ADD, 1, UI_Color_Cyan, 1, X_CENTRE + 0, Y_CENTRE + 300, X_CENTRE +130, Y_CENTRE + 230);//130  70
    Char_Draw(&Shoot_String[6], "CJD",  UI_Graph_ADD, 1, UI_Color_Yellow, 15, sizeof(String_J), Shoot_Char_Width, X_CENTRE + 65, Y_CENTRE + 265, String_J);

    //?????? 12m
    Rectangle_Draw(&Shoot_Rectangle[2], "BUB", UI_Graph_ADD, 1, UI_Color_Cyan, 1, X_CENTRE -30, Y_CENTRE - 30, X_CENTRE -10, Y_CENTRE-10);
    Char_Draw(&Shoot_String[7], "CBB",  UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_B), Shoot_Char_Width, X_CENTRE - 20, Y_CENTRE - 20, String_B);

    /* ????????? + ????е???????????е?????? */
    Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_ADD, 1, UI_Color_Cyan, 1, X_CENTRE -400, Y_CENTRE-300, X_CENTRE + 400, Y_CENTRE+300);
    Circle_Draw(&Camera_Centre, "CAT", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE, Y_CENTRE, 2);
    
    /* ?????? */
    Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE - 200, Y_CENTRE -100, X_CENTRE - 300, Y_CENTRE -450);
    Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE + 200, Y_CENTRE -100, X_CENTRE + 300, Y_CENTRE -450);
    
#elif ROBOT_ID == 3
    Line_Draw(&Shoot_Line[0], "S1m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -100, X_CENTRE + 30, Y_CENTRE -100);
    Line_Draw(&Shoot_Line[1], "S3m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -150, X_CENTRE + 30, Y_CENTRE -150);
    Line_Draw(&Shoot_Line[2], "S5m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -200, X_CENTRE + 30, Y_CENTRE -200);
    Line_Draw(&Shoot_Line[3], "S7m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -250, X_CENTRE + 30, Y_CENTRE -250);
    Line_Draw(&Shoot_Line[4], "S9m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -300, X_CENTRE + 30, Y_CENTRE -300);
    Char_Draw(&Shoot_String[0], "C1m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_1m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -100, String_1m);
    Char_Draw(&Shoot_String[1], "C3m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_3m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -150, String_3m);
    Char_Draw(&Shoot_String[2], "C5m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_5m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -200, String_5m);
    Char_Draw(&Shoot_String[3], "C7m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_7m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -250, String_7m);
    Char_Draw(&Shoot_String[4], "C9m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_9m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -300, String_9m);
    Line_Draw(&Shoot_Line[5], "CTR", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE , Y_CENTRE , X_CENTRE, Y_CENTRE - 500 ); //??????
    /* ????????? + ????е???????????е?????? */
    Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_ADD, 1, UI_Color_Cyan, 1, X_CENTRE - 300, Y_CENTRE - 300, X_CENTRE + 300, Y_CENTRE + 350);
    Circle_Draw(&Camera_Centre, "CAT", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE, Y_CENTRE, 2);
    /* ?????? */
    Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE - 200, Y_CENTRE -100, X_CENTRE - 300, Y_CENTRE -450);
    Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE + 200, Y_CENTRE -100, X_CENTRE + 300, Y_CENTRE -450);
    
#elif ROBOT_ID == 4
    Line_Draw(&Shoot_Line[0], "S1m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -100, X_CENTRE + 30, Y_CENTRE -100);
    Line_Draw(&Shoot_Line[1], "S3m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -150, X_CENTRE + 30, Y_CENTRE -150);
    Line_Draw(&Shoot_Line[2], "S5m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -200, X_CENTRE + 30, Y_CENTRE -200);
    Line_Draw(&Shoot_Line[3], "S7m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -250, X_CENTRE + 30, Y_CENTRE -250);
    Line_Draw(&Shoot_Line[4], "S9m", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE - 30, Y_CENTRE -300, X_CENTRE + 30, Y_CENTRE -300);
    Char_Draw(&Shoot_String[0], "C1m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_1m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -100, String_1m);
    Char_Draw(&Shoot_String[1], "C3m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_3m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -150, String_3m);
    Char_Draw(&Shoot_String[2], "C5m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_5m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -200, String_5m);
    Char_Draw(&Shoot_String[3], "C7m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_7m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -250, String_7m);
    Char_Draw(&Shoot_String[4], "C9m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_9m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -300, String_9m);
//    Line_Draw(&Shoot_Line[5], "CTR", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE , Y_CENTRE , X_CENTRE, Y_CENTRE - 500 ); //??????
//    Float_Draw(Float_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float Graph_Float)
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
    Float_Draw(&Fric_NUM,"FUM",UI_Graph_ADD, 2, UI_Color_Cyan, 19, 2, 2, X_CENTRE + 400, Y_CENTRE + 200, Fric_Speed);
     
    /* ????????? + ????е???????????е?????? */
    Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_ADD, 1, UI_Color_Cyan, 1, X_CENTRE -400, Y_CENTRE-300, X_CENTRE + 400, Y_CENTRE+300);
    Circle_Draw(&Camera_Centre, "CAT", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE, Y_CENTRE, 5);
    /* ?????? */
    Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE - 200, Y_CENTRE -100, X_CENTRE - 300, Y_CENTRE -450);
    Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE + 200, Y_CENTRE -100, X_CENTRE + 300, Y_CENTRE -450);
#endif
    
    /* ??????????? */
    Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_ADD, 1, UI_Color_Yellow, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
    Line_Draw(&SuperCap_Line, "CAP", UI_Graph_ADD, 1, UI_Color_Green, 10,  X_CENTRE - 400, Y_CENTRE - 390, X_CENTRE - 400, Y_CENTRE - 390);

    /* ???????(????????UI) */
    uint8_t R = 10;
//    Circle_Draw(&Armor_Circle[0], "AM0", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw),            Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw),          R);
//    Circle_Draw(&Armor_Circle[1], "AM1", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw - PI/2),     Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw- PI/2),    R);
//    Circle_Draw(&Armor_Circle[2], "AM2", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw - PI),       Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw- PI),      R);
//    Circle_Draw(&Armor_Circle[3], "AM3", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw - PI/2 *3 ), Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw- PI/2 *3), R);
    
    /* ????????????????*/
    uint8_t Mode_Char_Size = 13;    //??????????
    uint8_t Mode_Char_Width = 2;      //??????????
    Char_Draw(&ChassisMode_String[0], "CHA", UI_Graph_ADD, 1, UI_Color_Green,  Mode_Char_Size, sizeof(String_Chassis), Mode_Char_Width+1, X_CENTRE - 700, Y_CENTRE +200, String_Chassis);
    Char_Draw(&ChassisMode_String[1], "CHF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Follow),  Mode_Char_Width,   X_CENTRE - 700, Y_CENTRE +170, String_Follow);
    Char_Draw(&ChassisMode_String[2], "CHR", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Rotate),  Mode_Char_Width,   X_CENTRE - 700, Y_CENTRE +140, String_Rotate);
    Char_Draw(&ChassisMode_String[3], "CHL", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Lock),    Mode_Char_Width,   X_CENTRE - 700, Y_CENTRE +110, String_Lock);
    
    Char_Draw(&GimbalMode_String[0], "GIM", UI_Graph_ADD, 1, UI_Color_Green, Mode_Char_Size,  sizeof(String_Head),  Mode_Char_Width+1, X_CENTRE - 570, Y_CENTRE +200, String_Head);
    Char_Draw(&GimbalMode_String[1], "GIF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Front), Mode_Char_Width,   X_CENTRE - 570, Y_CENTRE+170,  String_Front);
    Char_Draw(&GimbalMode_String[2], "GIB", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Back),  Mode_Char_Width,   X_CENTRE - 570, Y_CENTRE +140, String_Back);

    Char_Draw(&ShootMode_String[0], "SHO", UI_Graph_ADD, 1, UI_Color_Green,  Mode_Char_Size, sizeof(String_Shoot), Mode_Char_Width+1, X_CENTRE - 460, Y_CENTRE +200, String_Shoot);
    Char_Draw(&ShootMode_String[1], "SHS", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Stop),  Mode_Char_Width,   X_CENTRE - 460, Y_CENTRE +170, String_Stop);
    Char_Draw(&ShootMode_String[2], "SHR", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Ready), Mode_Char_Width,   X_CENTRE - 460, Y_CENTRE +140, String_Ready);
    Char_Draw(&ShootMode_String[3], "SHK", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Stuck), Mode_Char_Width,   X_CENTRE - 460, Y_CENTRE +110, String_Stuck);

    Char_Draw(&AimMode_String[0], "AIM", UI_Graph_ADD, 1, UI_Color_Green,  Mode_Char_Size, sizeof(String_Aim), Mode_Char_Width+1, X_CENTRE - 350, Y_CENTRE +200, String_Aim);
    Char_Draw(&AimMode_String[1], "AMF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Off), Mode_Char_Width,   X_CENTRE - 350, Y_CENTRE +170, String_Off);
    Char_Draw(&AimMode_String[2], "AMN", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_On),  Mode_Char_Width,   X_CENTRE - 350, Y_CENTRE +140, String_On);
    
    
#if ROBOT_ID == 3 
    Char_Draw(&LidMode_String[0], "LID", UI_Graph_ADD, 1, UI_Color_Green,  Mode_Char_Size, sizeof(String_Lid), Mode_Char_Width+1, X_CENTRE - 280, Y_CENTRE +200, String_Lid);
    Char_Draw(&LidMode_String[1], "LDF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Off), Mode_Char_Width,   X_CENTRE - 280, Y_CENTRE +170, String_Off);
    Char_Draw(&LidMode_String[2], "LDN", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_On),  Mode_Char_Width,   X_CENTRE - 280, Y_CENTRE +140, String_On);
    Rectangle_Draw(&Mode_Rectangle[4], "RLD", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 285, Y_CENTRE +200, X_CENTRE - 235, Y_CENTRE +190);  
#endif
    Rectangle_Draw(&Mode_Rectangle[0], "RCA", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 705, Y_CENTRE +200, X_CENTRE - 605, Y_CENTRE +180);   
    Rectangle_Draw(&Mode_Rectangle[1], "RGI", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 575, Y_CENTRE +200, X_CENTRE - 485, Y_CENTRE +190);   
    Rectangle_Draw(&Mode_Rectangle[2], "RSH", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 465, Y_CENTRE +200, X_CENTRE - 375, Y_CENTRE +190);   
    Rectangle_Draw(&Mode_Rectangle[3], "RAM", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 355, Y_CENTRE +200, X_CENTRE - 305, Y_CENTRE +190);
}
/* ???UI???? */
void UI_Move()
{
    static uint8_t twink_flag = 0; //????????????????
    static uint8_t Armor_flag = 0; //???????????1S
    /* ?л???*/
    
    
//    if(Communication_Action_Rx.ChassisAction == CHASSIS_FOLLOW)//??????
//        Rectangle_Draw(&Mode_Rectangle[0], "RCA", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 705, Y_CENTRE +175, X_CENTRE - 625, Y_CENTRE +150); 
//     else if(Communication_Action_Rx.ChassisAction == CHASSIS_SPIN)
//        Rectangle_Draw(&Mode_Rectangle[0], "RCA", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 705, Y_CENTRE +145, X_CENTRE - 625, Y_CENTRE +120); 
//     else if(Communication_Action_Rx.ChassisAction == CHASSIS_NORMAL)
//        Rectangle_Draw(&Mode_Rectangle[0], "RCA", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 705, Y_CENTRE +115, X_CENTRE - 645, Y_CENTRE +90); 
//     
//     if(Communication_Action_Rx.MidMode == FRONT)//???????
//        Rectangle_Draw(&Mode_Rectangle[1], "RGI", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 575, Y_CENTRE +175, X_CENTRE - 505, Y_CENTRE +150); 
//     else if(Communication_Action_Rx.MidMode == BACK)
//        Rectangle_Draw(&Mode_Rectangle[1], "RGI", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 575, Y_CENTRE +145, X_CENTRE - 515, Y_CENTRE +120); 
//    
//      if(Communication_Action_Rx.ShootAction == SHOOT_STOP)//?????????
//        Rectangle_Draw(&Mode_Rectangle[2], "RSH", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 465, Y_CENTRE +175, X_CENTRE - 405, Y_CENTRE +150); 
//     else if(Communication_Action_Rx.ShootAction == SHOOT_READY)
//        Rectangle_Draw(&Mode_Rectangle[2], "RSH", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 465, Y_CENTRE +145, X_CENTRE - 395, Y_CENTRE +120); 
//     else if(Communication_Action_Rx.ShootAction == SHOOT_STUCKING)
//        Rectangle_Draw(&Mode_Rectangle[2], "RSH", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 465, Y_CENTRE +115, X_CENTRE - 355, Y_CENTRE +90); 
//    
//    if(Communication_Action_Rx.AimAction == AIM_STOP)//??????
//        Rectangle_Draw(&Mode_Rectangle[3], "RAM", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 355, Y_CENTRE +175, X_CENTRE - 315, Y_CENTRE +150); 
//     else if(Communication_Action_Rx.AimAction == AIM_AUTO )
//        Rectangle_Draw(&Mode_Rectangle[3], "RAM", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 355, Y_CENTRE +145, X_CENTRE - 325, Y_CENTRE +120); 

#if ROBOT_ID == 3
     if(Communication_Action_Rx.LidMode == LID_OFF)//????
        Rectangle_Draw(&Mode_Rectangle[4], "RLD", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 285, Y_CENTRE +175, X_CENTRE - 245, Y_CENTRE +150); 
     else if(Communication_Action_Rx.LidMode == LID_ON )
        Rectangle_Draw(&Mode_Rectangle[4], "RLD", UI_Graph_Change, 1, UI_Color_Green, 2, X_CENTRE - 285, Y_CENTRE +145, X_CENTRE - 255, Y_CENTRE +120); 
#endif
//     
//     /* ?????д?????????飬??????????????, UI?????? */
//     if(Up_State == Device_Offline && Examine_Motor_State() != osOK)//????3????? CHASSIS
//         Rectangle_Draw(&Mode_Rectangle[0], "RCA", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 705, Y_CENTRE +205, X_CENTRE - 605, Y_CENTRE +90); 
//     if(Communication_Action_Rx.Device_State & 0x01)//?????????? GIMBAL
//         Rectangle_Draw(&Mode_Rectangle[1], "RGI", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 575, Y_CENTRE +205, X_CENTRE - 485, Y_CENTRE +120); 
//     if(Communication_Action_Rx.Device_State & 0x02)//?????????? SHOOT
//        Rectangle_Draw(&Mode_Rectangle[2], "RSH", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 465, Y_CENTRE +205, X_CENTRE - 370, Y_CENTRE +90); 
//     if(Communication_Action_Rx.Device_State & 0x04)//PC???? IMU???? AIM
//        Rectangle_Draw(&Mode_Rectangle[3], "RAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 355, Y_CENTRE +205, X_CENTRE - 305, Y_CENTRE +120); 
//     
//     /* ???嶯???????? */
//        if(Armor_time > 0)
//            Armor_time --;
//         
//        (Damage_status.armor_id & 0x00) && Armor_time ?//ID 1
//         Circle_Draw(&Armor_Circle[0], "AM0", UI_Graph_Change, 1, UI_Color_Purplish_red, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw),         Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw),         10) :
//         Circle_Draw(&Armor_Circle[0], "AM0", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw),         Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw),         10);
//    
//        (Damage_status.armor_id & 0x01) && Armor_time ?//ID 2
//         Circle_Draw(&Armor_Circle[1], "AM1", UI_Graph_Change, 1, UI_Color_Purplish_red, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw + PI/2),  Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw + PI/2),   10) :
//         Circle_Draw(&Armor_Circle[1], "AM1", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw + PI/2),  Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw + PI/2),   10);
//    
//        (Damage_status.armor_id & 0x02) && Armor_time ?//ID 3
//         Circle_Draw(&Armor_Circle[2], "AM2", UI_Graph_Change, 1, UI_Color_Purplish_red, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw + PI),     Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw + PI),     10) :
//         Circle_Draw(&Armor_Circle[2], "AM2", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw + PI),     Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw + PI),     10);
//    
//        (Damage_status.armor_id & 0x03) && Armor_time ?//ID 4
//         Circle_Draw(&Armor_Circle[3], "AM3", UI_Graph_Change, 1, UI_Color_Purplish_red, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw + PI/2*3), Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw + PI/2*3), 10) :
//         Circle_Draw(&Armor_Circle[3], "AM3", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw + PI/2*3), Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw + PI/2*3), 10);

//     /* ???????????????? */
//     if(Cap_State == Device_Online){
//        Line_Draw(&SuperCap_Line, "CAP", UI_Graph_Change, 1, UI_Color_Green, 10,  X_CENTRE - 450, Y_CENTRE - 440, X_CENTRE - (450 - SuperCap_Rx.cap_percent * 3), Y_CENTRE - 440);
//         if(SuperCap_Rx.cap_percent == 0){ //?????????0?????DCDC????????????
//             if(twink_flag <=11){
//                 Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Purplish_red, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
//             } else {
//                 Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Green, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
//             if(twink_flag == 20)
//                twink_flag = 0;
//             }
//             twink_flag ++;
//         } else {
//             Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Green, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);             
//         }
//     } else {       //?????????????????°????can?????
//         Line_Draw(&SuperCap_Line, "CAP", UI_Graph_Change, 1, UI_Color_Purplish_red, 10,  X_CENTRE - 450, Y_CENTRE - 440, X_CENTRE - 200, Y_CENTRE - 440);
//         Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Purplish_red, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 200, Y_CENTRE - 430);
//     }
     
#if ROBOT_ID == 1
     /* ???????? */
     Communication_Action_Rx.Device_State & 0x08 ?//??????????????
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 1, X_CENTRE - 300, Y_CENTRE - 200, X_CENTRE + 300, Y_CENTRE + 200):
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Cyan, 1, X_CENTRE - 400, Y_CENTRE - 200, X_CENTRE + 400, Y_CENTRE + 200);

     /* ?????? */
        Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE - (100 + Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 - Offset_Angle_Rx.Pitch * 10), X_CENTRE - (250 + Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);//Xб??Y????
        Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE + (100 + Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 - Offset_Angle_Rx.Pitch * 10), X_CENTRE + (250 + Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);
#elif ROBOT_ID == 3
     /* ???????? */
     Communication_Action_Rx.Device_State & 0x08 ?//??????????????
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 1, X_CENTRE - 300, Y_CENTRE - 200, X_CENTRE + 300, Y_CENTRE + 200):
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Cyan, 1, X_CENTRE - 400, Y_CENTRE - 200, X_CENTRE + 400, Y_CENTRE + 200);

     /* ?????? */
        Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE - (90 - Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 + Offset_Angle_Rx.Pitch * 10), X_CENTRE - (375 - Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);//Xб??Y????
        Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE + (90 - Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 + Offset_Angle_Rx.Pitch * 10), X_CENTRE + (375 - Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);
#elif ROBOT_ID == 4
     /* ????? */
        Fric_Speed++;
        Float_Draw(&Fric_NUM,"FUM",UI_Graph_Change, 2, UI_Color_Cyan, 19, 2, 2, X_CENTRE + 400, Y_CENTRE + 200, Fric_Speed);    

//     Communication_Action_Rx.Device_State & 0x08 ?//??????????????
//        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 1, X_CENTRE - 300, Y_CENTRE - 200, X_CENTRE + 300, Y_CENTRE + 200):
//        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Cyan, 1, X_CENTRE - 400, Y_CENTRE - 200, X_CENTRE + 400, Y_CENTRE + 200);

//     /* ?????? */
//        Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE - (80 - Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 + Offset_Angle_Rx.Pitch * 10), X_CENTRE - (375 - Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);//Xб??Y????
//        Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE + (80 - Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 + Offset_Angle_Rx.Pitch * 10), X_CENTRE + (375 - Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);
#endif
}

/* ???UI???? */
void UI_Refresh()
{
    static uint8_t cnt = 0;
    if(UI_STATE == INITING){//???
        switch(cnt++){
            case 0: UI_Delete(UI_Data_Del_ALL, 9);break;
            case 1: UI_Delete(UI_Data_Del_ALL, 8);break;
            case 2: UI_Delete(UI_Data_Del_ALL, 7);break;
            case 3: UI_Delete(UI_Data_Del_ALL, 6);break;
            case 4: UI_Delete(UI_Data_Del_ALL, 5);break;
            case 5: UI_Delete(UI_Data_Del_ALL, 4);break;
            case 6: UI_Delete(UI_Data_Del_ALL, 3);break;
            case 7: UI_Delete(UI_Data_Del_ALL, 2);break;
            case 8: UI_Delete(UI_Data_Del_ALL, 1);break;
            case 9: UI_Delete(UI_Data_Del_ALL, 0);break;
            case 10: Graph_ReFresh(1, SuperCap_Rectangle); break;
            case 11: Graph_ReFresh(7, SuperCap_Line, Armor_Circle[0], Armor_Circle[1], Armor_Circle[2], Armor_Circle[3], Chassis_Line[0], Chassis_Line[1]); break;
            case 12: Char_ReFresh(ChassisMode_String[0]); break;
            case 13: Char_ReFresh(ChassisMode_String[1]); break;
            case 14: Char_ReFresh(ChassisMode_String[2]); break;
            case 15: Char_ReFresh(ChassisMode_String[3]); break;
            case 16: Char_ReFresh(GimbalMode_String[0]); break;
            case 17: Char_ReFresh(GimbalMode_String[1]); break;
            case 18: Char_ReFresh(GimbalMode_String[2]); break;
            case 19: Char_ReFresh(ShootMode_String[0]); break;
            case 20: Char_ReFresh(ShootMode_String[1]); break;
            case 21: Char_ReFresh(ShootMode_String[2]); break;
            case 22: Char_ReFresh(ShootMode_String[3]); break;
            case 23: Char_ReFresh(AimMode_String[0]);  break;
            case 24: Char_ReFresh(AimMode_String[1]);  break;
            case 25: Char_ReFresh(AimMode_String[2]);  break;
#if ROBOT_ID == 1
            case 26: Graph_ReFresh(5, Shoot_Rectangle[0], Shoot_Rectangle[1], Shoot_Rectangle[2], Shoot_Line[0], Shoot_Line[1]); break;
            case 28: Char_ReFresh(Shoot_String[5]);  break;
            case 29: Char_ReFresh(Shoot_String[6]);  break;
            case 30: Char_ReFresh(Shoot_String[7]);  break;
            case 31: Graph_ReFresh(5, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Mode_Rectangle[4], Camera_Centre, Camera_Rectangle);  break;
            case 32:  Graph_ReFresh(7, Scale_Line[0],Scale_Line[1],Scale_Line[2],Scale_Line[3],Scale_Line[4],Scale_Line[5],Scale_Line[6]); break;                     
            case 33:  Graph_ReFresh(7, Scale_Line[7],Scale_Line[8],Scale_Line[9],Scale_Line[10],Scale_Line[11],Scale_Line[12],Scale_Line[13]); break;   
            case 34:  Graph_ReFresh(5, Scale_Line[14],Scale_Line[15], Scale_Line[16], Scale_Line[17], Scale_Line[18], Scale_Line[19]); break;           
            
            case 36: UI_STATE = MOVEING; cnt=0; break;                     
#elif ROBOT_ID == 3
            case 26: Graph_ReFresh(7, Shoot_Line[0], Shoot_Line[1], Shoot_Line[2], Shoot_Line[3], Shoot_Line[4], Shoot_Line[5], SuperCap_Rectangle); break;
            case 27: Char_ReFresh(LidMode_String[0]);  break;
            case 28: Char_ReFresh(LidMode_String[1]);  break;
            case 29: Char_ReFresh(LidMode_String[2]);  break;
            case 30: Char_ReFresh(Shoot_String[0]); break;
            case 31: Char_ReFresh(Shoot_String[1]); break;
            case 32: Char_ReFresh(Shoot_String[2]); break;
            case 33: Char_ReFresh(Shoot_String[3]); break;
            case 34: Char_ReFresh(Shoot_String[4]); break;            
            case 35: Graph_ReFresh(7, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Mode_Rectangle[4], Camera_Centre, Camera_Rectangle);  break;
            case 36: UI_STATE = MOVEING; cnt=0; break;
#elif ROBOT_ID == 4
            case 26: Graph_ReFresh(7, Shoot_Line[0], Shoot_Line[1], Shoot_Line[2], Shoot_Line[3], Shoot_Line[4], Shoot_Line[5], SuperCap_Rectangle); break;
            case 27: Char_ReFresh(Shoot_String[0]); break;
            case 28: Char_ReFresh(Shoot_String[1]); break;
//            case 29: Char_ReFresh(Shoot_String[2]); break;
            case 30: Char_ReFresh(Shoot_String[3]); break;
            case 31: Char_ReFresh(Shoot_String[4]); break;
            case 32: Graph_ReFresh(1, Fric_NUM); break;            
            case 33: Graph_ReFresh(5, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Camera_Rectangle);  break;
            case 34: UI_STATE = MOVEING; cnt=0; break;
#endif
            default: break;
        }
    }
    if(UI_STATE == MOVEING){//???
        if(cnt == 1){
#if ROBOT_ID == 3
            Graph_ReFresh(7, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Mode_Rectangle[4], Chassis_Line[0], Chassis_Line[1]);
#else
            Graph_ReFresh(7, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Chassis_Line[0], Chassis_Line[1], Camera_Centre);
#endif
            cnt = 0;
        } else if (cnt == 0){
            Graph_ReFresh(7, Camera_Rectangle, SuperCap_Rectangle, SuperCap_Line, Armor_Circle[0], Armor_Circle[1], Armor_Circle[2], Armor_Circle[3] );

            cnt = 2;
        }
        else
        {   
            Graph_ReFresh(1, Fric_NUM);
            cnt = 1;
        }
    }
}