//--------------------------------------------------------------
// File     : stm32_ub_touch_ads7843.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef STM32_UB_TOUCH_ADS7843_H
#define STM32_UB_TOUCH_ADS7843_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "SSD1963_api.h"
#include "SSD1963.h"
#include "SSD1963_Configuration.h"
#include "misc.h"
#include <stdbool.h>


// Touch-Status
typedef enum {
  TOUCH_PRESSED  = 0,   // Touch ist betaetigt
  TOUCH_RELEASED = 1	// Touch ist nicht betaetigt
}Touch_Status_t;

// Pixelkoordinaten
typedef struct {
  uint16_t xp;
  uint16_t yp;
}Touch_Pixel_t;

// Konstantenwerte zum Umrechnen der Rohdaten in Display-Daten
typedef struct {
  float A;
  float B;
  float C;
  float D;
  float E;
  float F;
  uint16_t xp1;
  uint16_t yp1;
  uint16_t xp2;
  uint16_t yp2;
  uint16_t xp3;
  uint16_t yp3;
}Touch_Konstanten_t;


//-----------------------------------------
// interne Struktur für die Kalibration
//-----------------------------------------
typedef struct {
  uint32_t timer_cnt;      // Counter vom Timer
  Touch_Pixel_t raw;       // Rohdaten der Pixelkoordinaten
  Touch_Pixel_t temp;      // Temp Pixelwerte
  Touch_Konstanten_t wert; // Konstantenwerte
}Touch_Kal_t;
Touch_Kal_t Touch_Kal;


//-----------------------------------------
// Globale Struktur der Touch-Daten
//-----------------------------------------
typedef struct {
  Touch_Status_t status;   // Status (gedrückt, losgelassen)
  Touch_Pixel_t pos;       // Display Pixelkoordinate
}Touch_Data_t;
Touch_Data_t Touch_Data;


//--------------------------------------------------------------
// Kommandos an den Touch-Controller
//--------------------------------------------------------------
#define  UB_TOUCH_CMD_CH3  0x90   // CH=X+, Mode=12bit
#define  UB_TOUCH_CMD_CH4  0xD0   // CH=Y+, Mode=12bit


//--------------------------------------------------------------
// Defines
// Hinweis : die Pausenzeiten sind nicht per Timer gemacht
//           Vorsicht bei anderen Optimierungsstufen als -O0
//--------------------------------------------------------------
#define  UB_TOUCH_MESS_DELAY   100    // Pause bis Messung startet
#define  UB_TOUCH_CLK_DELAY    1000   // pause für das SPI-Clock-Signal


//--------------------------------------------------------------
// mit MAX_DIFF wird festgelegt wie weit zwei benachbarte Pixel
// maximal entfernt sein dürfen um als gütlig erkannt zu werden
//--------------------------------------------------------------
#define  UB_TOUCH_MAX_DIFF     30     // max Pixel differenz


//--------------------------------------------------------------
// Touch Kalibrationspunkte
//--------------------------------------------------------------
#define   UB_TOUCH_KAL_XP1     240     // 50% von MAXX
#define   UB_TOUCH_KAL_YP1     255     // 90% von MAXY
#define   UB_TOUCH_KAL_XP2      48     // 10% von MAXX
#define   UB_TOUCH_KAL_YP2     136     // 50% von MAXY
#define   UB_TOUCH_KAL_XP3     432     // 90% von MAXX
#define   UB_TOUCH_KAL_YP3      27     // 10% von MAXY

#define   UB_TOUCH_KAL_CBG    LCD_COLOR_WHITE  // Hintergrundfarbe bei Kalibration
#define   UB_TOUCH_KAL_CPKT   LCD_COLOR_BLACK  // Farbe vom Zielkreuz
#define   UB_TOUCH_KAL_COK    LCD_COLOR_GREEN  // Hintergrund wenn OK
#define   UB_TOUCH_KAL_CERR   LCD_COLOR_RED    // Hintergrund wenn Error

#define   UB_TOUCH_KAL_DELAY   10     // Pause beim kalibrieren (10 = 500ms)

//--------------------------------------------------------------
#define   UB_TOUCH_DC_TIME     10     // DoubleClick-Zeit (10=500ms)
#define   UB_TOUCH_DC_TIMEOUT  20     // DoubleClick-Timeout (20=1sec)


#define TOUCH_CS_PORT    GPIOB
#define TOUCH_CS_PIN    GPIO_Pin_12

#define TOUCH_IRQ_PORT    GPIOD
#define TOUCH_IRQ_PIN    GPIO_Pin_6

#define T_CS()   GPIO_ResetBits(TOUCH_CS_PORT, TOUCH_CS_PIN);
#define T_DCS()  GPIO_SetBits(TOUCH_CS_PORT, TOUCH_CS_PIN);

//LCD Informationen
#define LCD_MAXX DISP_HOR_RESOLUTION
#define LCD_MAXY DISP_VER_RESOLUTION


//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------
ErrorStatus UB_Touch_Init(void);
ErrorStatus UB_Touch_Calibrate(void);
bool UB_Touch_OnPressed(void);
bool UB_Touch_OnClick(void);
bool UB_Touch_OnRelease(void);
bool UB_Touch_OnDoubleClick(void);
void ub_touch_handler_50ms(void);

//--------------------------------------------------------------
#endif // STM32_UB_TOUCH_ADS7843_H
