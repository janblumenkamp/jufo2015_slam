
//--------------------------------------------------------------
// File     : stm32_ub_touch_ads7843.c
// Datum    : 21.08.2013
// Version  : 1.7
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : GPIO, TIM, MISC, STM32_UB_LCD_ST7783
// Funktion : Touch-Funktionen
//            (Chip = ADS7843, oder XPT2046, oder UH7843)
//
// Hinweis  : Der Touch benutzt die CPU-Pins :
//            PA15 -> ADS7843_ChipSelect
//            PB1  -> ADS7843_DIN
//            PB4  -> ADS7843_Clk
//            PB5  -> ADS7843_DOUT
//            PB15 -> ADS7843_PEN
//            PC2  -> ADS7843_BUSY (wird im Code nicht benutzt)
//
//  Der Touch wird per Timer5 (TIM5) zyklisch abgefragt (50ms)
//  und die Daten werden in einer Struktur gespeichert :
//
//            Touch_Data.pos.xp = X-Position [0...239]
//            Touch_Data.pos.yp = Y-Position [0...319]
//            Touch_Data.status = [TOUCH_PRESSED, TOUCH_RELEASED]
//              -> Koordinaten nur gültig bei "TOUCH_PRESSED"
//--------------------------------------------------------------




//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_touch_ADS7843.h"
#include "printf.h"

//--------------------------------------------------------------
// interne Enumerationen
//--------------------------------------------------------------
typedef enum {
  KAL_PKT1 =0,
  KAL_PKT2,
  KAL_PKT3,
  KAL_READY,
  KAL_OK,
  KAL_ERR
}TOUCH_KAL_PKT_t;

typedef enum {
  MW_NONE =0,
  MW_4
}TOUCH_MW_t;

//--------------------------------------------------------------
// interne Funktionen
//--------------------------------------------------------------
void P_Touch_InitIO(void);
void P_Touch_InitTIM(void);
void P_Touch_InitNVIC(void);
void P_Touch_Read(void);
void P_Touch_ReadRaw(TOUCH_MW_t mw);
void P_Touch_CalcPos(void);
ErrorStatus P_Touch_CheckPos(void);
void P_Touch_KalDrawPoint(TOUCH_KAL_PKT_t pkt);
void P_Touch_DrawKreuz(uint16_t x, uint16_t y);
void P_Touch_KalSetData(void);
uint32_t P_Touch_PEN(void);
u8 P_Touch_WriteByte(u8 byte);
uint16_t P_Touch_Frame(u8 adr);


//--------------------------------------------------------------
// Init vom Touch
// Return_wert :
//  -> ERROR   , wenn Touch nicht gefunden wurde
//  -> SUCCESS , wenn Touch OK
//--------------------------------------------------------------
ErrorStatus UB_Touch_Init(void)
{
  ErrorStatus ret_wert=ERROR;

  // Variablen init
  Touch_Data.status=TOUCH_RELEASED;
  Touch_Data.pos.xp=0;
  Touch_Data.pos.yp=0;


  // Kalibrations Werte
  Touch_Kal.timer_cnt=0;
  Touch_Kal.raw.xp=0x8000;
  Touch_Kal.raw.yp=0x8000;
  Touch_Kal.temp.xp=0;
  Touch_Kal.temp.yp=0;

  //---------------------------------------------
  // Standard Kalibrationsdaten von UB
  // diese Daten koennen durch die Funktion
  // "UB_Toch_Calibrate" ermittelt werden
  // nach der Kalibration einfach per Debugger aulesen
  // und als Konstantenwerte eintragen
  //---------------------------------------------
  Touch_Kal.wert.A = 0.0;
  Touch_Kal.wert.B = 0.0;
  Touch_Kal.wert.C = 0.0;
  Touch_Kal.wert.D = 0.0;
  Touch_Kal.wert.E = 0.0;
  Touch_Kal.wert.F = 0.0;
  Touch_Kal.wert.xp1=996;
  Touch_Kal.wert.yp1=294;
  Touch_Kal.wert.xp2=1750;
  Touch_Kal.wert.yp2=1058;
  Touch_Kal.wert.xp3=271;
  Touch_Kal.wert.yp3=1679;

  P_Touch_KalSetData(); //Berechne A bis F basierend auf xp....

  // IO-Lines initialisieren
  P_Touch_InitIO();

  //-----------------------------------------
  // Test ob Touch-Controller vorhanden ist
  //-----------------------------------------
  P_Touch_ReadRaw(MW_4);

  // Test ob Messung erfolgreich
  if((Touch_Kal.raw.xp<=0x0FFF) && (Touch_Kal.raw.yp<=0x0FFF)) {
	ret_wert=SUCCESS;
  }

  return(ret_wert);
}




//--------------------------------------------------------------
// Kalibration des Touch
// ueber 3 Punkte die vom User gedrueckt werden muessen
// Return_wert :
//  -> ERROR   , wenn Abgleich ok war
//  -> SUCCESS , wenn Abgleich nicht ok war
//--------------------------------------------------------------
ErrorStatus UB_Touch_Calibrate(void)
{
	u8 ret_wert=1;
	uint16_t xp1 = 0,yp1 = 0,xp2 = 0,yp2 = 0,xp3 = 0,yp3 = 0,ok;
	static u8 sm_cal = 2;

	while(sm_cal < 9)
	{
		switch (sm_cal) {
		case 2:
			if(Touch_Data.status == TOUCH_RELEASED) //Touchscreen wird nicht gedrückt
			{
				P_Touch_KalDrawPoint(KAL_PKT1);

				sm_cal = 3;
			}
			break;
		case 3:
			if(Touch_Data.status == TOUCH_PRESSED)
			{
				// werte speichern
				xp1=Touch_Kal.raw.xp;
				yp1=Touch_Kal.raw.yp;
				sm_cal = 4;
			}
			break;
		case 4:
			if(Touch_Data.status == TOUCH_RELEASED) //Touchscreen wird nicht gedrückt
			{
				P_Touch_KalDrawPoint(KAL_PKT2);

				sm_cal = 5;
			}
			break;
		case 5:
			if(Touch_Data.status == TOUCH_PRESSED)
			{
				// werte speichern
				xp2=Touch_Kal.raw.xp;
				yp2=Touch_Kal.raw.yp;
				sm_cal = 6;
			}
			break;
		case 6:
			if(Touch_Data.status == TOUCH_RELEASED) //Touchscreen wird nicht gedrückt
			{
				P_Touch_KalDrawPoint(KAL_PKT3);

				sm_cal = 7;
			}
			break;
		case 7:
			if(Touch_Data.status == TOUCH_PRESSED)
			{
				// werte speichern
				xp3=Touch_Kal.raw.xp;
				yp3=Touch_Kal.raw.yp;
				sm_cal = 8;
			}
			break;
		case 8:
			if(Touch_Data.status == TOUCH_RELEASED) //Touchscreen wird nicht gedrückt
			{
				P_Touch_KalDrawPoint(KAL_READY);
				// kalibration fertig
				// Test ob werte ok sind

				ok=0;
				if((yp3>yp2) && (yp2>yp1)) ok++;
				if((yp3<yp2) && (yp2<yp1)) ok++;
				if((xp2>xp1) && (xp1>xp3)) ok++;
				if((xp2<xp1) && (xp1<xp3)) ok++;
				if(ok>=2) {
					// werte sind ok
					// Daten speichern und umrechnen
					Touch_Kal.wert.xp1=xp1;
					Touch_Kal.wert.yp1=yp1;
					Touch_Kal.wert.xp2=xp2;
					Touch_Kal.wert.yp2=yp2;
					Touch_Kal.wert.xp3=xp3;
					Touch_Kal.wert.yp3=yp3;

					fprintf(&debug, "xp1: %i\n\r", xp1);
					fprintf(&debug, "yp1: %i\n\r", yp1);
					fprintf(&debug, "xp2: %i\n\r", xp2);
					fprintf(&debug, "yp2: %i\n\r", yp2);
					fprintf(&debug, "xp3: %i\n\r", xp3);
					fprintf(&debug, "yp3: %i\n\r", yp3);

					P_Touch_KalSetData();
					//P_Touch_KalDrawPoint(KAL_OK);
					ret_wert=1;
					sm_cal = 9;
				}
				else {
					P_Touch_KalDrawPoint(KAL_ERR);
					ret_wert=2;
					sm_cal = 2;
				}
			}
			break;
		case 9: break;
		default:
			break;
		}
	}
	sm_cal = 2;

	return(ret_wert);
}


//--------------------------------------------------------------
// Touch OnPressed Auswertung
// ret_wert, ist solange true wie der Touch betätigt ist
//--------------------------------------------------------------
bool UB_Touch_OnPressed(void)
{
  bool ret_wert=false;

  if(Touch_Data.status==TOUCH_PRESSED) {
	ret_wert=true;
  }

  return(ret_wert);
}


//--------------------------------------------------------------
// Touch OnClick Auswertung
// ret_wert, ist nur einmal true wenn der Touch betätigt wurde
//--------------------------------------------------------------
bool UB_Touch_OnClick(void)
{
  bool ret_wert=false;
  static bool old_wert=false;

  if(Touch_Data.status==TOUCH_PRESSED) {
	if(old_wert==false) {
	  ret_wert=true;
	}
	old_wert=true;
  }
  else {
	old_wert=false;
  }

  return(ret_wert);
}


//--------------------------------------------------------------
// Touch OnClick Auswertung
// ret_wert, ist nur einmal true wenn der Touch losgelassen wurde
//--------------------------------------------------------------
bool UB_Touch_OnRelease(void)
{
  bool ret_wert=false;
  static bool old_wert=false;

  if(Touch_Data.status==TOUCH_RELEASED) {
	if(old_wert==true) {
	  ret_wert=true;
	}
	old_wert=false;
  }
  else {
	old_wert=true;
  }

  return(ret_wert);
}


//--------------------------------------------------------------
// Touch OnDoubleClick Auswertung
// ret_wert, ist nur einmal true wenn der Touch zweimal betätigt wurde
//--------------------------------------------------------------
bool UB_Touch_OnDoubleClick(void)
{
  bool ret_wert=false;
  static uint8_t mode=0;

  if(mode==0) {
	// warte auf ersten Click
	if(Touch_Data.status==TOUCH_PRESSED) {
	  mode=1;
	  Touch_Kal.timer_cnt=0;
	}
  }
  else if(mode==1) {
	// warten auf erstes loslassen
	if(Touch_Data.status==TOUCH_RELEASED) {
	  mode=2;
	}
	else {
	  // test ob zu lange gedrückt
	  if(Touch_Kal.timer_cnt>UB_TOUCH_DC_TIMEOUT) {
		mode=3;
	  }
	}
  }
  else if(mode==2) {
	// warten auf zweiten Click
	if(Touch_Data.status==TOUCH_PRESSED) {
	  mode=3;
	  if(Touch_Kal.timer_cnt<UB_TOUCH_DC_TIME) {
		ret_wert=true;
	  }
	}
	else {
	  // test ob zu lange losgelassen
	  if(Touch_Kal.timer_cnt>UB_TOUCH_DC_TIMEOUT) {
		mode=0;
	  }
	}
  }
  else {
	// warten auf zweites loslassen
	if(Touch_Data.status==TOUCH_RELEASED) {
	  mode=0;
	}
  }

  return(ret_wert);
}


//--------------------------------------------------------------
// interne Funktion
// Init aller IO-Pins fuer den Touch-Controller
//--------------------------------------------------------------
void P_Touch_InitIO(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;

	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_15|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);      //sclk	10	 13
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);	//miso	11	 14
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);	//mosi	12	 15

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//SPI_CPOL_Low 	 SPI_CPOL_High
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   //SPI_NSS_Hard	 //SPI_NSS_Soft
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 	//16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2,&SPI_InitStructure);
	SPI_Cmd(SPI2,ENABLE);
	//CS
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin=TOUCH_CS_PIN;
	GPIO_Init(TOUCH_CS_PORT,&GPIO_InitStruct);
	T_DCS();

	//IRQ
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Pin=TOUCH_IRQ_PIN;
	GPIO_Init(TOUCH_IRQ_PORT, &GPIO_InitStruct);
}


//--------------------------------------------------------------
// Diese Funktion wird zyklisch per Timer aufgerufen
//
// Auslesen vom Touch und fuellen der Struktur "Touch_Data"
//    Touch_Data.status => [TOUCH_RELEASED, TOUCH_PRESSED]
//    Touch_Data.pos.xp => Xpos der Koordinate
//    Touch_Data.pos.yp => YPos der Koordinate
//
// -> Falls Touch nicht betaetigt,
//    sind die Koordinatenwerte nicht gueltig
//--------------------------------------------------------------
void P_Touch_Read(void)
{
  Touch_Status_t akt_status=TOUCH_RELEASED;

  // Pen einlesen
  if(P_Touch_PEN()==Bit_RESET) {
	akt_status=TOUCH_PRESSED;
  }
  else {
	akt_status=TOUCH_RELEASED;
  }

  if(akt_status==TOUCH_PRESSED) {
	// wenn Touch betätigt, Touch auswerten
	// Touch-Daten einlesen (4 Mittelwerte)
	P_Touch_ReadRaw(MW_4);
	// Touch-Daten in LCD-Koordinaten umrechnen
	P_Touch_CalcPos();
	// Check ob Daten gueltig
	if(P_Touch_CheckPos()==SUCCESS) {
	  Touch_Data.status=TOUCH_PRESSED;
	}
  }
  else {
	Touch_Data.status=TOUCH_RELEASED;
	// Touch Daten muessen nicht gelesen werden
  }
}

//--------------------------------------------------------------
// interne Funktion
// Einlesen der Roh-Touch-Daten
//  mw => [MW_NONE, MW_4]
//--------------------------------------------------------------
void P_Touch_ReadRaw(TOUCH_MW_t mw)
{
  uint16_t n,wert;
  uint32_t xp=0,yp=0;

  if(mw==MW_NONE) {
	// keine Mittelwerte
	// CH-4 (X) einlesen
	Touch_Kal.raw.xp=P_Touch_Frame(UB_TOUCH_CMD_CH4);
	Touch_Kal.raw.yp=P_Touch_Frame(UB_TOUCH_CMD_CH3);
  }
  else {
	// erstem Messwert verwerfen
	P_Touch_Frame(UB_TOUCH_CMD_CH4);
	// Mittelwert aus 4 Messungen bilden
	for(n=0;n<4;n++) {
	  // CH-4 (X) einlesen
	  wert=P_Touch_Frame(UB_TOUCH_CMD_CH4);
	  xp+=wert;
	  // CH-3 (Y) einlesen
	  wert=P_Touch_Frame(UB_TOUCH_CMD_CH3);
	  yp+=wert;
	}
	// Werte speichern
	Touch_Kal.raw.xp=(xp>>2);
	Touch_Kal.raw.yp=(yp>>2);
  }
}

//--------------------------------------------------------------
// interne Funktion
// Umrechnen der Touch-Werte in Pixel-Koordinaten
//--------------------------------------------------------------
void P_Touch_CalcPos(void)
{
  float f1,f2,XTf,YTf;
  int32_t i1,i2,i3;
  int32_t XD,YD;

  // Rohdaten
  XTf=(float)(Touch_Kal.raw.xp);
  YTf=(float)(Touch_Kal.raw.yp);

  // X-Position ausrechnen
  f1=Touch_Kal.wert.A*XTf;
  i1=(int)(f1);
  f2=Touch_Kal.wert.B*YTf;
  i2=(int)(f2);
  i3=(int)(Touch_Kal.wert.C);
  XD=i1+i2+i3;

  // Y-Position ausrechnen
  f1=Touch_Kal.wert.D*XTf;
  i1=(int)(f1);
  f2=Touch_Kal.wert.E*YTf;
  i2=(int)(f2);
  i3=(int)(Touch_Kal.wert.F);
  YD=i1+i2+i3;

  // Werte auf Display-Grenzen prüfen
  if(XD>=LCD_MAXX) XD=(LCD_MAXX-1);
  if(XD<0) XD=0;

  if(YD>=LCD_MAXY) YD=(LCD_MAXY-1);
  if(YD<0) YD=0;

  Touch_Kal.temp.xp=XD;
  Touch_Kal.temp.yp=YD;
}


//--------------------------------------------------------------
// interne Funktion
// Check ob Positions-Daten ok sind
// Return_wert :
//  -> ERROR   , wenn Daten ok
//  -> SUCCESS , wenn Daten  nicht ok
//--------------------------------------------------------------
ErrorStatus P_Touch_CheckPos(void)
{
  ErrorStatus ret_wert=SUCCESS;
  static uint16_t old_x=0,old_y=0;
  uint16_t diff;

  if(Touch_Kal.temp.xp>=old_x) {
	diff=Touch_Kal.temp.xp-old_x;
  }
  else {
	diff=old_x-Touch_Kal.temp.xp;
  }
  if(diff>UB_TOUCH_MAX_DIFF) ret_wert=ERROR;

  if(Touch_Kal.temp.yp>=old_y) {
	diff=Touch_Kal.temp.yp-old_y;
  }
  else {
	diff=old_y-Touch_Kal.temp.yp;
  }
  if(diff>UB_TOUCH_MAX_DIFF) ret_wert=ERROR;

  old_x=Touch_Kal.temp.xp;
  old_y=Touch_Kal.temp.yp;

  if(ret_wert==SUCCESS) {
	// wenn Daten ok, dann speichern
	Touch_Data.pos.xp=Touch_Kal.temp.xp;
	Touch_Data.pos.yp=Touch_Kal.temp.yp;
  }

  return(ret_wert);
}

//--------------------------------------------------------------
// interne Funktion
// zeichnet ein Kalibrationsbild
//  pkt => [KAL_PKT1,KAL_PKT2,KAL_PKT3,KAL_READY,KAL_OK,KAL_ERR]
//--------------------------------------------------------------
void P_Touch_KalDrawPoint(TOUCH_KAL_PKT_t pkt)
{
	if(pkt==KAL_PKT1) {
		P_Touch_DrawKreuz(UB_TOUCH_KAL_XP1,UB_TOUCH_KAL_YP1);
	}
	else if(pkt==KAL_PKT2) {
		P_Touch_DrawKreuz(UB_TOUCH_KAL_XP2,UB_TOUCH_KAL_YP2);
	}
	else if(pkt==KAL_PKT3) {
		P_Touch_DrawKreuz(UB_TOUCH_KAL_XP3,UB_TOUCH_KAL_YP3);
	}
	else if(pkt==KAL_READY) {
		LCD_Fill(UB_TOUCH_KAL_CBG);
	}
	else if(pkt==KAL_OK) {
		LCD_Fill(UB_TOUCH_KAL_COK);
	}
	else {
		LCD_Fill(UB_TOUCH_KAL_CERR);
	}
}

//--------------------------------------------------------------
// interne Funktion
// zeichnet einen Kalibrationskreuz an x,y
//--------------------------------------------------------------
void P_Touch_DrawKreuz(uint16_t x, uint16_t y)
{
  LCD_Fill(UB_TOUCH_KAL_CBG);
  LCD_Line(x-12,y,x+13,y, UB_TOUCH_KAL_CPKT);
  LCD_Line(x,y-12,x,y+13, UB_TOUCH_KAL_CPKT);
  LCD_PutPixel(x+1,y+1,UB_TOUCH_KAL_CPKT);
  LCD_PutPixel(x-1,y+1,UB_TOUCH_KAL_CPKT);
  LCD_PutPixel(x+1,y-1,UB_TOUCH_KAL_CPKT);
  LCD_PutPixel(x-1,y-1,UB_TOUCH_KAL_CPKT);
  LCD_Circle(x,y,6, UB_TOUCH_KAL_CPKT, 0);
}

//--------------------------------------------------------------
// interne Funktion
// berechnet die Kalibrationsdaten
//--------------------------------------------------------------
void P_Touch_KalSetData(void)
{
  float XD1,YD1,XD2,YD2,XD3,YD3;
  float XT1f,YT1f,XT2f,YT2f,XT3f,YT3f;
  float f1,f2,f3,f4,f5,f6,fz,fn;

  XD1=(float)(UB_TOUCH_KAL_XP1);
  YD1=(float)(UB_TOUCH_KAL_YP1);
  XD2=(float)(UB_TOUCH_KAL_XP2);
  YD2=(float)(UB_TOUCH_KAL_YP2);
  XD3=(float)(UB_TOUCH_KAL_XP3);
  YD3=(float)(UB_TOUCH_KAL_YP3);

  XT1f=(float)(Touch_Kal.wert.xp1);
  YT1f=(float)(Touch_Kal.wert.yp1);
  XT2f=(float)(Touch_Kal.wert.xp2);
  YT2f=(float)(Touch_Kal.wert.yp2);
  XT3f=(float)(Touch_Kal.wert.xp3);
  YT3f=(float)(Touch_Kal.wert.yp3);

  // A ausrechnen
  f1=XD1*(YT2f-YT3f);
  f2=XD2*(YT3f-YT1f);
  f3=XD3*(YT1f-YT2f);
  f4=XT1f*(YT2f-YT3f);
  f5=XT2f*(YT3f-YT1f);
  f6=XT3f*(YT1f-YT2f);
  fz=f1+f2+f3;
  fn=f4+f5+f6;
  if(fn==0) fn=0.1;
  Touch_Kal.wert.A=fz/fn;

  // B ausrechnen
  f1=Touch_Kal.wert.A*(XT3f-XT2f);
  f2=XD2;
  f3=XD3;
  fz=f1+f2-f3;
  fn=(YT2f-YT3f);
  if(fn==0) fn=0.1;
  Touch_Kal.wert.B=fz/fn;

  // C ausrechnen
  f1=XD3;
  f2=Touch_Kal.wert.A*XT3f;
  f3=Touch_Kal.wert.B*YT3f;
  Touch_Kal.wert.C=f1-f2-f3;

  // D ausrechnen
  f1=YD1*(YT2f-YT3f);
  f2=YD2*(YT3f-YT1f);
  f3=YD3*(YT1f-YT2f);
  f4=XT1f*(YT2f-YT3f);
  f5=XT2f*(YT3f-YT1f);
  f6=XT3f*(YT1f-YT2f);
  fz=f1+f2+f3;
  fn=f4+f5+f6;
  if(fn==0) fn=0.1;
  Touch_Kal.wert.D=fz/fn;

  // E ausrechnen
  f1=Touch_Kal.wert.D*(XT3f-XT2f);
  f2=YD2;
  f3=YD3;
  fz=f1+f2-f3;
  fn=(YT2f-YT3f);
  if(fn==0) fn=0.1;
  Touch_Kal.wert.E=fz/fn;

  // F ausrechnen
  f1=YD3;
  f2=Touch_Kal.wert.D*XT3f;
  f3=Touch_Kal.wert.E*YT3f;
  Touch_Kal.wert.F=f1-f2-f3;
}

//--------------------------------------------------------------
// interne Funktion
// Pegel von PEN einlesen
//--------------------------------------------------------------
uint32_t P_Touch_PEN(void)
{
  return GPIO_ReadInputDataBit(TOUCH_IRQ_PORT, TOUCH_IRQ_PIN);
}

//--------------------------------------------------------------
// interne Funktion
// Sendet ein byte per SPI2 an den ADS7843
// und empfangt ein byte
//--------------------------------------------------------------
u8 P_Touch_WriteByte(u8 byte)
{
	unsigned char Data = 0;
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI2,byte);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	Data = SPI_I2S_ReceiveData(SPI2);

	return Data;
}

//--------------------------------------------------------------
// interne Funktion
// Sendet einen 24bit Frame an den ADS7843
// und empfangt einen 12Bit Datenwert
//--------------------------------------------------------------
uint16_t P_Touch_Frame(u8 adr)
{
	u16 data = 0;
	T_CS();
	P_Touch_WriteByte(adr);
	data = (((P_Touch_WriteByte(0) << 8) | P_Touch_WriteByte(0)) >> 4);
	T_DCS();
	return (data);
}

//--------------------------------------------------------------
// Timer des Touchscreens, muss alle 50ms aufgerufen werden!
//--------------------------------------------------------------
void ub_touch_handler_50ms(void)
{
  Touch_Kal.timer_cnt++;

  // Touch auswerten
  P_Touch_Read();
}
