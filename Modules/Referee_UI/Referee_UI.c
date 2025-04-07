#include "Referee_UI.h"
#include "Referee_unpack.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "SuperCAP.h"
#include "arm_math.h"
/* 1.????????? */
Graph_Data Shoot_Line[12];   //?????????  ???????
Graph_Data Scale_Line[28];

char String_1m[] = "1", String_3m[] = "3",String_5m[] = "5",String_7m[] = "7",String_9m[] = "9", String_Q[] = "Q",String_J[] = "J",String_B[] = "B";
String_Data Shoot_String[1];
Graph_Data Shoot_Rectangle[3];//???
extern Referee_data_t Referee_SendData;
Graph_Data Scale_Line[28];

extern Gimbal_data_t receive_vision;
extern Gimbal_action_t receive_action;
/* 2.???���� */
Graph_Data Camera_Rectangle;//????????????????
Graph_Data Camera_Centre;//????????
Graph_Data Chassis_status;
Graph_Data Scale_Line[28];
Graph_Data Gimbal_status[3];
Graph_Data Shoot_status;
Graph_Data enemy_HP_line;
game_robot_HP_t robot_HP;
/* 3.???????? */
Graph_Data SuperCap_Rectangle;//??????
Graph_Data SuperCap_Line; //????
Graph_Data Acr_17mmAmmo;
Graph_Data Acr_42mmAmmo;
Graph_Data Acr_life;
/* 4.????????? */
Graph_Data Chassis_Line[2];

/* 5.????��?? */
Graph_Data Armor_Circle[4];

/* 6.???????? */
int  UI_sign = 0;
Float_Data Fric_NUM;
uint32_t Fric_Speed = 500;
  
Float_Data Vision_DIS;
uint32_t Vision_distance = 0;

Float_Data Vision_NUM;
  
float HP_rate = 0.00f;
uint16_t HP_lose = 0;

Float_Data Ammo_NUM;
uint16_t Ammo_remain = 0;
uint16_t Ammo_total = 400; 
float Ammo_rate = 0.00f;
uint16_t Ammo_lose = 0;
float enemy_rate = 0.00f;
uint16_t enemy_HP = 0;
uint16_t add_vis = 0;

String_Data ChassisMode_String[1];
char String_Chassis[]  = "C";

String_Data GimbalMode_String[1];
char String_Head []  = "G";

String_Data ShootMode_String[1];
char String_Shoot [] = "S";

String_Data Gimbal_status_String[2];
char String_Gimbal_status_p [] = "P";
char String_Gimbal_status_y [] = "Y";

String_Data Chassis_status_String[1];
char Chassis_status_offline[] = "C";
//String_Data Gimbal_status_String[];

String_Data Vision_status_String[1];
char Vision_status_offline[] = "V";

String_Data RPY[3];

String_Data Vision_d[3];

char String_d [] = "Vision_d:NUM";

char FRIC [] = "FRIC:SPEED";


Graph_Data Mode_Rectangle[5];

extern Supercap_TypeDef SuperCap_Rx;

#define ROBOT_ID 4

#if ROBOT_ID == 3
char String_Lid [] = "Lid:";
String_Data LidMode_String[3];
#endif

/* ????????????UI???? */
void Task_Referee_UI(void *pvParameters)
{
    static portTickType currentTime;
    for (;;)
    {
        currentTime = xTaskGetTickCount();      // ????????
        if(Referee_SendData.refree_status == Device_Online || UI_sign == 6){
            if(receive_action.Key && 0x01)
                  UI_STATE = INIT;
              if(UI_STATE == INIT || UI_sign <= 5){
                  UI_Init();
                  UI_STATE = INITING;
                  UI_sign++;
              }
              else if(UI_STATE == MOVEING)
                  UI_Move();
              UI_Refresh();
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

    /* ????????? + ????��???????????��?????? */
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
    /* ????????? + ????��???????????��?????? */
    Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_ADD, 1, UI_Color_Cyan, 1, X_CENTRE - 300, Y_CENTRE - 300, X_CENTRE + 300, Y_CENTRE + 350);
    Circle_Draw(&Camera_Centre, "CAT", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE, Y_CENTRE, 2);
    /* ?????? */
    Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE - 200, Y_CENTRE -100, X_CENTRE - 300, Y_CENTRE -450);
    Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_ADD,  1, UI_Color_Yellow, 1, X_CENTRE + 200, Y_CENTRE -100, X_CENTRE + 300, Y_CENTRE -450);
    
#elif ROBOT_ID == 4
    Line_Draw(&Shoot_Line[0], "S1m", UI_Graph_ADD, 1, UI_Color_Main, 3, X_CENTRE - 60, Y_CENTRE + 100, X_CENTRE + 8, Y_CENTRE + 100);
    Line_Draw(&Shoot_Line[1], "S3m", UI_Graph_ADD, 1, UI_Color_Main, 3, X_CENTRE - 92, Y_CENTRE + 130, X_CENTRE + 31, Y_CENTRE + 130);
//    Line_Draw(&Shoot_Line[2], "S5m", UI_Graph_ADD, 1, UI_Color_Main, 3, X_CENTRE - 40, Y_CENTRE + 200, X_CENTRE + 25, Y_CENTRE + 200);
//    Line_Draw(&Shoot_Line[3], "S7m", UI_Graph_ADD, 1, UI_Color_Main, 3, X_CENTRE - 50, Y_CENTRE + 250, X_CENTRE + 35, Y_CENTRE + 250);
//    Line_Draw(&Shoot_Line[4], "S9m", UI_Graph_ADD, 1, UI_Color_Main, 3, X_CENTRE - 60, Y_CENTRE + 300, X_CENTRE + 45, Y_CENTRE + 300);
    uint8_t R = 10;
    Circle_Draw(&Armor_Circle[0], "AM0", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * arm_sin_f32(receive_vision.Offset_Angle),            Y_CENTRE + 50 * cos(receive_vision.Offset_Angle),          R);
    Circle_Draw(&Armor_Circle[1], "AM1", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * arm_sin_f32(receive_vision.Offset_Angle - PI/2),     Y_CENTRE + 50 * cos(receive_vision.Offset_Angle- PI/2),    R);
    Circle_Draw(&Armor_Circle[2], "AM2", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * arm_sin_f32(receive_vision.Offset_Angle - PI),       Y_CENTRE + 50 * cos(receive_vision.Offset_Angle- PI),      R);
    Circle_Draw(&Armor_Circle[3], "AM3", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * arm_sin_f32(receive_vision.Offset_Angle - PI/2 *3 ), Y_CENTRE + 50 * cos(receive_vision.Offset_Angle- PI/2 *3), R);
   
//    Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_ADD, 1, UI_Color_Yellow, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
//    Line_Draw(&SuperCap_Line, "CAP", UI_Graph_ADD, 1, UI_Color_Green, 10,  X_CENTRE - 400, Y_CENTRE - 390, X_CENTRE - 400, Y_CENTRE - 390);

//    Char_Draw(&Shoot_String[0], "C1m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_1m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -100, String_1m);
//    Char_Draw(&Shoot_String[1], "C3m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_3m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -150, String_3m);
//    Char_Draw(&Shoot_String[2], "C5m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_5m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -200, String_5m);
//    Char_Draw(&Shoot_String[3], "C7m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_7m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -250, String_7m);
//    Char_Draw(&Shoot_String[4], "C9m", UI_Graph_ADD, 1, UI_Color_Yellow, Shoot_Char_Size, sizeof(String_9m), Shoot_Char_Width, X_CENTRE + 30, Y_CENTRE -300, String_9m);
    Arc_Draw(&Acr_17mmAmmo, "A17", UI_Graph_ADD, 3, UI_Color_Cyan, 3, 40, 3, X_CENTRE + 30, Y_CENTRE + 30, 100, 100);
    Arc_Draw(&Acr_42mmAmmo, "A42", UI_Graph_ADD, 2, UI_Color_Orange, 10, 170, 3, X_CENTRE + 260, Y_CENTRE , 125, 280);
    Arc_Draw(&Acr_life, "Ale", UI_Graph_ADD, 2, UI_Color_Green, 190, -170, 3, X_CENTRE - 260, Y_CENTRE , 125, 280);
//    Circle_Draw(&Camera_Centre, "CAT", UI_Graph_ADD, 4, UI_Color_Yellow, 2 ,X_CENTRE, Y_CENTRE, 5);
    Circle_Draw(&Chassis_status, "CSU", UI_Graph_ADD, 4, UI_Color_Yellow, 2 ,X_CENTRE - 800, Y_CENTRE + 200, 18);
//    Circle_Draw(&Gimbal_status[0], "GSU", UI_Graph_ADD, 4, UI_Color_Yellow, 2 ,X_CENTRE - 750, Y_CENTRE + 200, 18);
    Circle_Draw(&Gimbal_status[1], "GSP", UI_Graph_ADD, 4, UI_Color_Yellow, 2 ,X_CENTRE + 900, Y_CENTRE + 190, 18);
    Circle_Draw(&Gimbal_status[2], "GSY", UI_Graph_ADD, 4, UI_Color_Yellow, 2 ,X_CENTRE + 900, Y_CENTRE + 145, 18);
//    Circle_Draw(&Shoot_status, "SSU", UI_Graph_ADD, 4, UI_Color_Yellow, 2 ,X_CENTRE - 700, Y_CENTRE + 200, 18);
//  Arc_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_tX_CENTREGraph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t x_Length,uint32_t y_Length)
//    Line_Draw(&Shoot_Line[5], "CTR", UI_Graph_ADD, 1, UI_Color_Yellow, 1, X_CENTRE , Y_CENTRE , X_CENTRE, Y_CENTRE - 500 ); //??????
//    Float_Draw(Float_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float Graph_Float)
/************************************************���Ƹ���������*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_Size     �ֺ�
        Graph_Digit    С��λ��
        Start_x��Start_x    ��ʼ����
        Graph_Float   Ҫ��ʾ�ı���
        #define UI_Color_Main 0         //��/����������ɫ��
        #define UI_Color_Yellow 1   //����
        #define UI_Color_Green 2    //����
        #define UI_Color_Orange 3
        #define UI_Color_Purplish_red 4 //�Ϻ�ɫ
        #define UI_Color_Pink 5
        #define UI_Color_Cyan 6         //��ɫ����
        #define UI_Color_Black 7
        #define UI_Color_White 8
**********************************************************************************************************/
//    Float_Draw(&Fric_NUM,"FUM",UI_Graph_ADD, 2, UI_Color_Cyan, 19, 2, 1, X_CENTRE + 400, Y_CENTRE + 200, Fric_Speed);
    Float_Draw(&Ammo_NUM,"AUM",UI_Graph_ADD, 2, UI_Color_Cyan, 19, 2, 2, X_CENTRE + 500, Y_CENTRE + 300, Referee_SendData.Ammo_consume * 1000);
//    Float_Draw(&Vision_DIS, "VDS", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 2, 2, X_CENTRE + 205, Y_CENTRE + 160, receive_vision.vision_distance * 1000);
//    Float_Draw(&Vision_NUM, "VNM", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 2, 2, X_CENTRE - 200, Y_CENTRE + 330, receive_action.vision_number * 1000);
    Line_Draw(&enemy_HP_line, "EHI", UI_Graph_ADD,  4, UI_Color_Yellow, 10, X_CENTRE - 160, Y_CENTRE + 320, X_CENTRE  - 98, Y_CENTRE + 320);

    /* ????????? + ????��???????????��?????? */
//    Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_ADD, 4, UI_Color_Cyan, 1, X_CENTRE - 200, Y_CENTRE - 150, X_CENTRE + 200, Y_CENTRE + 150);

    Circle_Draw(&Camera_Centre, "CAT", UI_Graph_ADD, 4, UI_Color_Yellow, 2 ,X_CENTRE, Y_CENTRE, 13);
    /* ?????? */
    Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_ADD,  4, UI_Color_Yellow, 1, X_CENTRE - 200, Y_CENTRE -200, X_CENTRE - 300, Y_CENTRE -450);
    Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_ADD,  4, UI_Color_Yellow, 1, X_CENTRE + 200, Y_CENTRE -200, X_CENTRE + 300, Y_CENTRE -450);
#endif


    /* ??????????? */
    Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_ADD, 2, UI_Color_Yellow, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
    Line_Draw(&SuperCap_Line, "CAP", UI_Graph_ADD, 2, UI_Color_Green, 10,  X_CENTRE - 400, Y_CENTRE - 390, X_CENTRE - 400, Y_CENTRE - 390);
    
    /* ???????(????????UI) */
//    uint8_t R = 10;
//    Circle_Draw(&Armor_Circle[0], "AM0", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw),            Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw),          R);
//    Circle_Draw(&Armor_Circle[1], "AM1", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw - PI/2),     Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw- PI/2),    R);
//    Circle_Draw(&Armor_Circle[2], "AM2", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw - PI),       Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw- PI),      R);
//    Circle_Draw(&Armor_Circle[3], "AM3", UI_Graph_ADD, 1, UI_Color_Green, 2 ,X_CENTRE + 50 * sin(Offset_Angle_Rx.Yaw - PI/2 *3 ), Y_CENTRE + 50 * cos(Offset_Angle_Rx.Yaw- PI/2 *3), R);
    
    /* ????????????????*/
    uint8_t Mode_Char_Size = 13;    //??????????
    uint8_t Mode_Char_Width = 2;      //??????????
    Char_Draw(&ChassisMode_String[0], "CHA", UI_Graph_ADD, 1, UI_Color_Yellow,  Mode_Char_Size, sizeof(String_Chassis), Mode_Char_Width+1, X_CENTRE - 800, Y_CENTRE + 205, String_Chassis);
//    Char_Draw(&ChassisMode_String[1], "CHF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Follow),  Mode_Char_Width,   X_CENTRE - 850, Y_CENTRE +170, String_Follow);
//    Char_Draw(&ChassisMode_String[2], "CHR", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Rotate),  Mode_Char_Width,   X_CENTRE - 850, Y_CENTRE +140, String_Rotate);
//    Char_Draw(&ChassisMode_String[3], "CHL", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Lock),    Mode_Char_Width,   X_CENTRE - 850, Y_CENTRE +110, String_Lock);
//    
//    Char_Draw(&GimbalMode_String[0], "GIM", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size,  sizeof(String_Head),  Mode_Char_Width+1, X_CENTRE - 750, Y_CENTRE + 205, String_Head);
//    Char_Draw(&GimbalMode_String[1], "GIF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Front), Mode_Char_Width,   X_CENTRE - 570, Y_CENTRE+170,  String_Front);
//    Char_Draw(&GimbalMode_String[2], "GIB", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Back),  Mode_Char_Width,   X_CENTRE - 570, Y_CENTRE +140, String_Back);

//    Char_Draw(&ShootMode_String[0], "SHO", UI_Graph_ADD, 1, UI_Color_Yellow,  Mode_Char_Size, sizeof(String_Shoot), Mode_Char_Width+1, X_CENTRE - 700, Y_CENTRE + 205, String_Shoot);
//    Char_Draw(&ShootMode_String[1], "SHS", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Stop),  Mode_Char_Width,   X_CENTRE - 460, Y_CENTRE +170, String_Stop);
//    Char_Draw(&ShootMode_String[2], "SHR", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Ready), Mode_Char_Width,   X_CENTRE - 460, Y_CENTRE +140, String_Ready);
//    Char_Draw(&ShootMode_String[3], "SHK", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Stuck), Mode_Char_Width,   X_CENTRE - 460, Y_CENTRE +110, String_Stuck);
    Char_Draw(&Gimbal_status_String[0], "GSS", UI_Graph_ADD, 5, UI_Color_Yellow, 20, sizeof(String_Gimbal_status_p), 3, X_CENTRE + 900, Y_CENTRE + 205, String_Gimbal_status_p);
    Char_Draw(&Gimbal_status_String[1], "CHY", UI_Graph_ADD, 5, UI_Color_Yellow, 20, sizeof(String_Gimbal_status_y), 3, X_CENTRE + 900, Y_CENTRE + 155, String_Gimbal_status_y);    

//    Char_Draw(&AimMode_String[0], "AIM", UI_Graph_ADD, 1, UI_Color_Green,  Mode_Char_Size, sizeof(String_Aim), Mode_Char_Width+1, X_CENTRE - 350, Y_CENTRE +200, String_Aim);
//    Char_Draw(&AimMode_String[1], "AMF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Off), Mode_Char_Width,   X_CENTRE - 350, Y_CENTRE +170, String_Off);
//    Char_Draw(&AimMode_String[2], "AMN", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_On),  Mode_Char_Width,   X_CENTRE - 350, Y_CENTRE +140, String_On);
    
    
#if ROBOT_ID == 3 
    Char_Draw(&LidMode_String[0], "LID", UI_Graph_ADD, 1, UI_Color_Green,  Mode_Char_Size, sizeof(String_Lid), Mode_Char_Width+1, X_CENTRE - 280, Y_CENTRE +200, String_Lid);
    Char_Draw(&LidMode_String[1], "LDF", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_Off), Mode_Char_Width,   X_CENTRE - 280, Y_CENTRE +170, String_Off);
    Char_Draw(&LidMode_String[2], "LDN", UI_Graph_ADD, 1, UI_Color_Yellow, Mode_Char_Size, sizeof(String_On),  Mode_Char_Width,   X_CENTRE - 280, Y_CENTRE +140, String_On);
    Rectangle_Draw(&Mode_Rectangle[4], "RLD", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 285, Y_CENTRE +200, X_CENTRE - 235, Y_CENTRE +190);  
#endif
//    Rectangle_Draw(&Mode_Rectangle[0], "RCA", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 705, Y_CENTRE +200, X_CENTRE - 605, Y_CENTRE +180);   
//    Rectangle_Draw(&Mode_Rectangle[1], "RGI", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 575, Y_CENTRE +200, X_CENTRE - 485, Y_CENTRE +190);   
//    Rectangle_Draw(&Mode_Rectangle[2], "RSH", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 465, Y_CENTRE +200, X_CENTRE - 375, Y_CENTRE +190);   
//    Rectangle_Draw(&Mode_Rectangle[3], "RAM", UI_Graph_ADD, 1, UI_Color_Green, 2, X_CENTRE - 355, Y_CENTRE +200, X_CENTRE - 305, Y_CENTRE +190);
}
/* ???UI???? */
void UI_Move()
{
    static uint8_t twink_flag = 0; //????????????????
    static uint8_t Armor_flag = 0; //???????????1S
    /* ?��???*/
    Fric_Speed = Referee_SendData.robot_HP;
    HP_rate = (float)(Referee_SendData.Max_HP - Referee_SendData.robot_HP) / (float)Referee_SendData.Max_HP;
    HP_lose = (uint16_t)(155 * HP_rate);
//    Ammo_rate = (float)(Ammo_amount.projectile_allowance_17mm) / (float)100.00;
//    Ammo_lose = (uint16_t)(157 * Ammo_rate);
//    Ammo_remain =  Ammo_total - Referee_SendData.Ammo_consume;   
    if(Ammo_amount.projectile_allowance_42mm <= 10)
    {
        Ammo_rate = (float)(Ammo_amount.projectile_allowance_42mm) / (float)10.00;
        Ammo_lose = (uint16_t)(157 * Ammo_rate);
        Ammo_remain =  Ammo_total - Referee_SendData.Ammo_consume;  
    }
    else
        Ammo_lose = 157;
    if(Robot_state.robot_id < 100)
    {
        switch(receive_action.vision_number)
        {
            case 1: enemy_HP = robot_HP.blue_1_robot_HP;break;
            case 2: enemy_HP = robot_HP.blue_2_robot_HP;break;
            case 3: enemy_HP = robot_HP.blue_3_robot_HP;break;
            case 4: enemy_HP = robot_HP.blue_4_robot_HP;break;
            case 7: enemy_HP = robot_HP.blue_7_robot_HP;break;
            default: enemy_HP = 0; break;
        }
    }
    else
    {
        switch(receive_action.vision_number)
        {
            case 1: enemy_HP = robot_HP.red_1_robot_HP;break;
            case 2: enemy_HP = robot_HP.red_2_robot_HP;break;
            case 3: enemy_HP = robot_HP.red_3_robot_HP;break;
            case 4: enemy_HP = robot_HP.red_4_robot_HP;break;
            case 7: enemy_HP = robot_HP.red_7_robot_HP;break;
            default: enemy_HP = 0; break;
        }
        
    }
//typedef struct{
//    uint16_t red_1_robot_HP;    //!< @brief �� 1 Ӣ�ۻ�����Ѫ�������û�����δ�ϳ����߱����£���Ѫ��Ϊ 0
//    uint16_t red_2_robot_HP;    //!< @brief �� 2 ���̻�����Ѫ��
//    uint16_t red_3_robot_HP;    //!< @brief �� 3 ����������Ѫ��
//    uint16_t red_4_robot_HP;    //!< @brief �� 4 ����������Ѫ��
//    uint16_t red_5_robot_HP;    //!< @brief �� 5 ����������Ѫ��
//    uint16_t red_7_robot_HP;    //!< @brief �� 7 �ڱ�������Ѫ��
//    uint16_t red_outpost_HP;    //!< @brief �췽ǰ��վѪ��
//    uint16_t red_base_HP;       //!< @brief �췽����Ѫ��
//    uint16_t blue_1_robot_HP;   //!< @brief �� 1 Ӣ�ۻ�����Ѫ��
//    uint16_t blue_2_robot_HP;   //!< @brief �� 2 ���̻�����Ѫ��
//    uint16_t blue_3_robot_HP;   //!< @brief �� 3 ����������Ѫ��
//    uint16_t blue_4_robot_HP;   //!< @brief �� 4 ����������Ѫ��
//    uint16_t blue_5_robot_HP;   //!< @brief �� 5 ����������Ѫ��
//    uint16_t blue_7_robot_HP;   //!< @brief �� 7 �ڱ�������Ѫ��
//    uint16_t blue_outpost_HP;   //!< @brief ����ǰ��վѪ��
//    uint16_t blue_base_HP;      //!< @brief ��������Ѫ��
//} game_robot_HP_t;
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
//     /* ?????��?????????�??????????????, UI?????? */
//     if(Up_State == Device_Offline && Examine_Motor_State() != osOK)//????3????? CHASSIS
//         Rectangle_Draw(&Mode_Rectangle[0], "RCA", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 705, Y_CENTRE +205, X_CENTRE - 605, Y_CENTRE +90); 
//     if(Communication_Action_Rx.Device_State & 0x01)//?????????? GIMBAL
//         Rectangle_Draw(&Mode_Rectangle[1], "RGI", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 575, Y_CENTRE +205, X_CENTRE - 485, Y_CENTRE +120); 
//     if(Communication_Action_Rx.Device_State & 0x02)//?????????? SHOOT
//        Rectangle_Draw(&Mode_Rectangle[2], "RSH", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 465, Y_CENTRE +205, X_CENTRE - 370, Y_CENTRE +90); 
//     if(Communication_Action_Rx.Device_State & 0x04)//PC???? IMU???? AIM
//        Rectangle_Draw(&Mode_Rectangle[3], "RAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 2, X_CENTRE - 355, Y_CENTRE +205, X_CENTRE - 305, Y_CENTRE +120); 
//     
//     /* ???�N???????? */
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
//     } else {       //?????????????????��????can?????
//         Line_Draw(&SuperCap_Line, "CAP", UI_Graph_Change, 1, UI_Color_Purplish_red, 10,  X_CENTRE - 450, Y_CENTRE - 440, X_CENTRE - 200, Y_CENTRE - 440);
//         Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Purplish_red, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 200, Y_CENTRE - 430);
//     }
     
#if ROBOT_ID == 1
     /* ???????? */
        Communication_Action_Rx.Device_State & 0x08 ?//??????????????
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 1, X_CENTRE - 300, Y_CENTRE - 200, X_CENTRE + 300, Y_CENTRE + 200):
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Cyan, 1, X_CENTRE - 400, Y_CENTRE - 200, X_CENTRE + 400, Y_CENTRE + 200);
        
     /* ?????? */
        Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE - (100 + Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 - Offset_Angle_Rx.Pitch * 10), X_CENTRE - (250 + Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);//X��??Y????
        Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE + (100 + Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 - Offset_Angle_Rx.Pitch * 10), X_CENTRE + (250 + Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);
#elif ROBOT_ID == 3
     /* ???????? */
     Communication_Action_Rx.Device_State & 0x08 ?//??????????????
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 1, X_CENTRE - 300, Y_CENTRE - 200, X_CENTRE + 300, Y_CENTRE + 200):
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Cyan, 1, X_CENTRE - 400, Y_CENTRE - 200, X_CENTRE + 400, Y_CENTRE + 200);

     /* ?????? */
        Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE - (90 - Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 + Offset_Angle_Rx.Pitch * 10), X_CENTRE - (375 - Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);//X��??Y????
        Line_Draw(&Chassis_Line[1], "CKR", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE + (90 - Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 + Offset_Angle_Rx.Pitch * 10), X_CENTRE + (375 - Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);
#elif ROBOT_ID == 4
     /* ????? */
//        Fric_Speed++;

        Float_Draw(&Fric_NUM,"FUM",UI_Graph_Change, 2, UI_Color_Cyan, 19, 2, 2, X_CENTRE + 400, Y_CENTRE + 200, Fric_Speed * 1000); 
        Float_Draw(&Ammo_NUM,"AUM",UI_Graph_Change, 2, UI_Color_Orange, 19, 2, 2, X_CENTRE + 350, Y_CENTRE + 280, Referee_SendData.Ammo_consume * 1000);
 
 
         Circle_Draw(&Armor_Circle[0], "AM0", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin((float)receive_vision.Offset_Angle / 1000),         Y_CENTRE + 50 * cos((float)receive_vision.Offset_Angle / 1000),         10);
         Circle_Draw(&Armor_Circle[1], "AM1", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin((float)receive_vision.Offset_Angle / 1000 + PI/2),  Y_CENTRE + 50 * cos((float)receive_vision.Offset_Angle / 1000 + PI/2),   10);
         Circle_Draw(&Armor_Circle[2], "AM2", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin((float)receive_vision.Offset_Angle / 1000 + PI),     Y_CENTRE + 50 * cos((float)receive_vision.Offset_Angle / 1000  + PI),     10);
         Circle_Draw(&Armor_Circle[3], "AM3", UI_Graph_Change, 1, UI_Color_Green,        2 ,X_CENTRE + 50 * sin((float)receive_vision.Offset_Angle / 1000 + PI/2*3), Y_CENTRE + 50 * cos((float)receive_vision.Offset_Angle / 1000  + PI/2*3), 10);

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

        
        if(Fric_Speed >= 800)        
        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 4, UI_Color_Green, 3, X_CENTRE - 400, Y_CENTRE - 300, X_CENTRE + 400, Y_CENTRE + 300);
        
        if(receive_action.Gimbal_status.Pitch == 0)  
        {
            Circle_Draw(&Gimbal_status[0], "GSU", UI_Graph_Change, 4, UI_Color_Purplish_red, 2 ,X_CENTRE - 750, Y_CENTRE + 200, 18);
            Circle_Draw(&Gimbal_status[1], "GSP", UI_Graph_Change, 4, UI_Color_Purplish_red, 2 ,X_CENTRE + 900, Y_CENTRE + 190, 18);

        }
        if(receive_action.Gimbal_status.Yaw == 0)  
        {
            Circle_Draw(&Gimbal_status[0], "GSU", UI_Graph_Change, 4, UI_Color_Purplish_red, 2 ,X_CENTRE - 750, Y_CENTRE + 200, 18);
            Circle_Draw(&Gimbal_status[2], "GSY", UI_Graph_Change, 4, UI_Color_Purplish_red, 2 ,X_CENTRE + 900, Y_CENTRE + 145, 18);
        }
        else if (receive_action.Gimbal_status.Pitch != 0 && receive_action.Gimbal_status.Yaw != 0)
        {
            Circle_Draw(&Gimbal_status[0], "GSU", UI_Graph_Change, 4, UI_Color_Green, 2 ,X_CENTRE - 750, Y_CENTRE + 200, 18);
            Circle_Draw(&Gimbal_status[1], "GSP", UI_Graph_Change, 4, UI_Color_Green, 2 ,X_CENTRE + 900, Y_CENTRE + 190, 18);
            Circle_Draw(&Gimbal_status[2], "GSY", UI_Graph_Change, 4, UI_Color_Green, 2 ,X_CENTRE + 900, Y_CENTRE + 145, 18);
        }

        switch(receive_action.move_status){
            case 0:
//                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                Circle_Draw(&Chassis_status, "CSU", UI_Graph_Change, 4, UI_Color_Purplish_red, 2 ,X_CENTRE - 800, Y_CENTRE + 200, 18);
            break;
            case 1:
//                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                Circle_Draw(&Chassis_status, "CSU", UI_Graph_Change, 4, UI_Color_Yellow, 2 ,X_CENTRE - 800, Y_CENTRE + 200, 18);
            break;
            case 2:
//                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                Circle_Draw(&Chassis_status, "CSU", UI_Graph_Change, 4, UI_Color_Green, 2 ,X_CENTRE - 800, Y_CENTRE + 200, 18);
            break;
            case 3:
//                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                Circle_Draw(&Chassis_status, "CSU", UI_Graph_Change, 4, UI_Color_Cyan, 2 ,X_CENTRE - 800, Y_CENTRE + 200, 18);
            break;
            case 4:
//                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                Circle_Draw(&Chassis_status, "CSU", UI_Graph_Change, 4, UI_Color_Orange, 2 ,X_CENTRE - 800, Y_CENTRE + 200, 18);
            break;
            
            default:break;
        }
//            else
//            Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_Del, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
             if(receive_action.shoot_status == 0)
                Circle_Draw(&Camera_Centre, "CAT",UI_Graph_Change, 4, UI_Color_Purplish_red, 2 ,X_CENTRE - 24, Y_CENTRE + 130, 13);    
             else if(receive_action.shoot_status == 1)
                switch(receive_action.shoot_mode){
                    case 0:
        //                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                        Circle_Draw(&Camera_Centre, "CAT", UI_Graph_Change, 4, UI_Color_Yellow, 2 ,X_CENTRE - 28, Y_CENTRE + 130, 13);
                    break;
                    case 1:
        //                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                        Circle_Draw(&Camera_Centre, "CAT", UI_Graph_Change, 4, UI_Color_Cyan, 2 ,X_CENTRE - 28, Y_CENTRE + 130, 13);
                    break;
                    case 2:
        //                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                         Circle_Draw(&Camera_Centre, "CAT", UI_Graph_Change, 4, UI_Color_Orange, 2 ,X_CENTRE - 28, Y_CENTRE + 130, 13);
                    break;
                    case 3:
        //                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
                        Circle_Draw(&Camera_Centre, "CAT", UI_Graph_Change, 4, UI_Color_Green, 2 ,X_CENTRE - 28, Y_CENTRE + 130, 13);
                    break;
        //            case 4:
        ////                Char_Draw(&Chassis_status_String[0], "CSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Chassis_status_offline), 3, X_CENTRE + 900, Y_CENTRE + 105, Chassis_status_offline);
        //                Circle_Draw(&Shoot_status, "SSU", UI_Graph_Change, 4, UI_Color_Purplish_red, 2 ,X_CENTRE - 800, Y_CENTRE + 200, 18);
        //            break;
                    default:break;
            }

        
        if(receive_action.vision_status == 0)
            Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 4, UI_Color_Purplish_red, 1, X_CENTRE - 200, Y_CENTRE - 150, X_CENTRE + 200, Y_CENTRE + 150);

//            Char_Draw(&Vision_status_String[0], "VSS", UI_Graph_ADD, 5, UI_Color_Purplish_red, 20, sizeof(Vision_status_offline), 3, X_CENTRE + 900,  Y_CENTRE + 55, Vision_status_offline);
        else if(receive_action.vision_status == 1)  
            Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 4, UI_Color_Cyan, 1, X_CENTRE - 200, Y_CENTRE - 150, X_CENTRE + 200, Y_CENTRE + 150);
        

        if(receive_action.shoot_mode == 3 && receive_action.vision_status == 1)
            Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 4, UI_Color_Green, 2, X_CENTRE - 200, Y_CENTRE - 150, X_CENTRE + 200, Y_CENTRE + 150);
            
//            if(receive_action.vision_number != 0 && add_vis == 0) 
//            {
//                Float_Draw(&Vision_DIS, "VDS", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 2, 2, X_CENTRE + 400, Y_CENTRE + 100, receive_vision.vision_distance * 1000);
//                Float_Draw(&Vision_NUM, "VNM", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 2, 2, X_CENTRE - 200, Y_CENTRE + 330, receive_action.vision_number * 1000);
//                Line_Draw(&enemy_HP_line, "EHI", UI_Graph_ADD,  2, UI_Color_Green, 10, X_CENTRE - 100, Y_CENTRE + 330, X_CENTRE +  -100 + 50, Y_CENTRE + 330);
//                add_vis = 1;
//            }
//            if(receive_action.vision_number != 0 )
//            {
                Float_Draw(&Vision_DIS, "VDS", UI_Graph_Change, 2, UI_Color_Cyan, 20, 2, 2, X_CENTRE + 205, Y_CENTRE + 150, receive_vision.vision_distance * 1000);
                Float_Draw(&Vision_NUM, "VNM", UI_Graph_Change, 2, UI_Color_Cyan, 20, 2, 2, X_CENTRE - 200, Y_CENTRE + 330, receive_action.vision_number * 1000);
                Line_Draw(&enemy_HP_line, "EHI", UI_Graph_Change,  4, UI_Color_Yellow, 10, X_CENTRE - 160, Y_CENTRE + 320, X_CENTRE + enemy_HP - 0, Y_CENTRE + 320);       

//            }
//            else if(receive_action.vision_number == 0)
//            {
//                Float_Draw(&Vision_DIS, "VDS", UI_Graph_Change, 2, UI_Color_Yellow, 20, 2, 2, X_CENTRE + 400, Y_CENTRE + 100, receive_vision.vision_distance * 1000);
//                Float_Draw(&Vision_NUM, "VNM", UI_Graph_Change, 2, UI_Color_Yellow, 20, 2, 2, X_CENTRE - 200, Y_CENTRE + 330, receive_action.vision_number * 1000);
//                Line_Draw(&enemy_HP_line, "EHI", UI_Graph_Change,  2, UI_Color_Cyan, 10, X_CENTRE - 100, Y_CENTRE + 330, X_CENTRE +  - 100 + 50, Y_CENTRE + 330);
////                add_vis = 0;
//            }
        
            Line_Draw(&SuperCap_Line, "CAP", UI_Graph_Change, 2, UI_Color_Green, 10,  X_CENTRE - 450, Y_CENTRE - 440, X_CENTRE - (450 - SuperCap_Rx.cap_percent * 3), Y_CENTRE - 440);
             if(SuperCap_Rx.cap_percent == 0){ //?????????0?????DCDC????????????
//                 if(twink_flag <=11){
                     Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
                 } 
//                 else {
//                     Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Green, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);
//                 if(twink_flag == 20)
//                    twink_flag = 0;
//                 }
//                 twink_flag ++;
//             } else {
//                 Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Green, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 150, Y_CENTRE - 430);             
//             }
//          else {       //?????????????????°????can?????
//             Line_Draw(&SuperCap_Line, "CAP", UI_Graph_Change, 1, UI_Color_Purplish_red, 10,  X_CENTRE - 450, Y_CENTRE - 440, X_CENTRE - 200, Y_CENTRE - 440);
//             Rectangle_Draw(&SuperCap_Rectangle, "SUP", UI_Graph_Change, 1, UI_Color_Purplish_red, 3, X_CENTRE - 450, Y_CENTRE - 450, X_CENTRE - 200, Y_CENTRE - 430);
//         }
//        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Purplish_red, 1, X_CENTRE - 300, Y_CENTRE - 200, X_CENTRE + 300, Y_CENTRE + 200):
//        Rectangle_Draw(&Camera_Rectangle, "CAM", UI_Graph_Change, 1, UI_Color_Cyan, 1, X_CENTRE - 400, Y_CENTRE - 200, X_CENTRE + 400, Y_CENTRE + 200);
          Arc_Draw(&Acr_42mmAmmo, "A42", UI_Graph_Change, 2, UI_Color_Orange, 10 + Ammo_lose, 170 , 3, X_CENTRE + 260, Y_CENTRE , 125, 280);
          Arc_Draw(&Acr_life, "Ale", UI_Graph_Change, 2, UI_Color_Green, 190, -165 - HP_lose, 3, X_CENTRE - 260, Y_CENTRE , 125, 280);
//        if(receive_action.shoot_status != 0)
//        if (referee_send_data.Ammo_remain <= 100)
//            Arc_Draw(&Acr_42mmAmmo, "A42", UI_Graph_Change, 2, UI_Color_Orange, 10 + 150, 170 , 3, X_CENTRE - 260, Y_CENTRE , 125, 280);
//     /* ?????? */
//        Line_Draw(&Chassis_Line[0], "CKL", UI_Graph_Change,  1, UI_Color_Yellow, 1, X_CENTRE - (80 - Offset_Angle_Rx.Pitch * 5), Y_CENTRE - (150 + Offset_Angle_Rx.Pitch * 10), X_CENTRE - (375 - Offset_Angle_Rx.Pitch * 10), Y_CENTRE -539);//X��??Y????
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
//            case 13: Char_ReFresh(ChassisMode_String[1]); break;
//            case 14: Char_ReFresh(ChassisMode_String[2]); break;
//            case 15: Char_ReFresh(ChassisMode_String[3]); break;
            case 13: Char_ReFresh(GimbalMode_String[0]); break;
//            case 17: Char_ReFresh(GimbalMode_String[1]); break;
//            case 18: Char_ReFresh(GimbalMode_String[2]); break;
            case 14: Char_ReFresh(ShootMode_String[0]); break;
            case 15: Char_ReFresh(Gimbal_status_String[0]); break;
            case 16: Char_ReFresh(Gimbal_status_String[1]); break;
//            case 17: Graph_ReFresh(ChassisMode_String[0]); break;
//            case 18: Char_ReFresh(Vision_status_String[0]);  break;
//            case 24: Char_ReFresh(AimMode_String[1]);  break;
//            case 25: Char_ReFresh(AimMode_String[2]);  break;
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
            case 17: Graph_ReFresh(1, SuperCap_Rectangle); break;
            case 18: Char_ReFresh(Shoot_String[0]); break;
//            case 17: Char_ReFresh(Shoot_String[1]); break;
//            case 29: Char_ReFresh(Shoot_String[2]); break;
//            case 18: Char_ReFresh(Shoot_String[3]); break;
//            case 19: Char_ReFresh(Shoot_String[4]); break;

            case 19: Graph_ReFresh(7, enemy_HP_line, Camera_Centre,Chassis_status,Gimbal_status[0],Shoot_status, Gimbal_status[1] ,Gimbal_status[2]); break;            
            case 20: Graph_ReFresh(5, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Camera_Rectangle);  break;
            case 21: Graph_ReFresh(1, Acr_42mmAmmo); break;
//            case 35: Graph_ReFresh(1, Acr_17mmAmmo); break;
            case 22: Graph_ReFresh(1, Acr_life); break;
            case 23: Graph_ReFresh(1, Vision_DIS); break;
            case 24: Graph_ReFresh(1, Vision_NUM); break;
            case 25: Graph_ReFresh(1, Ammo_NUM); break;
//            case 27: Graph_ReFresh(1, enemy_HP_line); break;
//            case 28: Graph_ReFresh(1, Acr_life); break;
            case 26: Graph_ReFresh(5, Shoot_Line[0], Shoot_Line[1], Shoot_Line[2], Shoot_Line[3], Shoot_Line[4]); break;
            case 27: UI_STATE = MOVEING; cnt=0; break;
#endif
            default: break;
        }
    }
    if(UI_STATE == MOVEING){//???
        switch(cnt++){
#if ROBOT_ID == 3
            Graph_ReFresh(7, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Mode_Rectangle[4], Chassis_Line[0], Chassis_Line[1]);
#else
//            Graph_ReFresh(7, Mode_Rectangle[0], Mode_Rectangle[1], Mode_Rectangle[2], Mode_Rectangle[3], Chassis_Line[0], Chassis_Line[1], Camera_Centre);
#endif
            case 0: Graph_ReFresh(7, Chassis_status, Gimbal_status[0], Shoot_status, Camera_Rectangle, Gimbal_status[1], Gimbal_status[2], Camera_Centre);break;
            case 1: Graph_ReFresh(7, Ammo_NUM, SuperCap_Rectangle, SuperCap_Line, Armor_Circle[0], Armor_Circle[1], Armor_Circle[2], Armor_Circle[3] );break;
            case 2: Graph_ReFresh(5, Vision_DIS, Vision_NUM, Acr_life,Fric_NUM,Acr_42mmAmmo);  cnt = 0;break;
//            case 3: Graph_ReFresh(1, enemy_HP_line); cnt = 0;break;
//            case 2: Graph_ReFresh(7, Vision_DIS, Vision_NUM, Acr_life, Fric_NUM, Acr_42mmAmmo, Ammo_remain, enemy_HP_line);  cnt = 0;break;
//            case 3: Graph_ReFresh(1, Vision_NUM); break;
//            case 4: Graph_ReFresh(1, Acr_life); break;
//            case 3: Char_ReFresh(Gimbal_status_String[0]); break;
//            case 4: Char_ReFresh(Gimbal_status_String[1]); break;
//            case 5: Char_ReFresh(Chassis_status_String[0]); break;  
//            case 7: Char_ReFresh(Vision_status_String[0]); cnt = 0; break;
//            case 9: Graph_ReFresh(1, Fric_NUM);cnt = 0;break;
            default: break;
        }
    }
}