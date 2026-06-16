#include "user.h"

void oled_ShowTask(void *pvPara)
{
   uint8_t string[30];
   OLED_Init();
   OLED_Clear();
   while(1)
   {
        sprintf((char*)string, "test");
        OLED_ShowString(0, 0, string);
        vTaskDelay(100);
   }

}
