//////////////////////////////////////////////////////////////////////////////////////
/// gui_graphics.c - Handling the raw GUI functions (touch events, drawing elements...
///
/// ADDING NEW ELEMENTS: see gui.h
//////////////////////////////////////////////////////////////////////////////////////

#include "gui_graphics.h"
#include "SSD1963_api.h"
#include "SSD1963.h"
#include "stm32_ub_font.h"
#include "stm32_ub_touch_ADS7843.h"
#include "printf.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////////
/// Private Prototypes (for a detailed description see each function)

void gui_redrawElement(GUI_ELEMENT *element);

///////////////////////////////////////////////////////////////////////////////////////
/// \brief graphics_init
/// \param element
/// Standard initialisation of gui_elemens (setting values to standard values, can be
/// overwritten in gui_init().

void graphics_init(GUI_ELEMENT element[])
{
	for(u8 i = 0; i < GUI_ELEMENTS_CNT; i++)
	{
		element[i].action = NULL;
		element[i].x = 0;
		element[i].y = 0;
		element[i].length = GetMaxX();
		element[i].heigth = GetMaxY();
		element[i].label = (char *)"-";
		element[i].font = &pArial_16;
		element[i].event.clicked = 0;
		element[i].event.doubleclick = 0;
		element[i].event.pressed = 0;
		element[i].event.released = 0;
		element[i].state = GUI_EL_INTOUCHABLE;

		switch (element[i].id) {
		case EL_ID_AREA:
			//No predefined settings...
			break;
		case EL_ID_MBTN:
			element[i].heigth = MBTN_STD_HEIGHT;
			element[i].length = MBTN_STD_LENGTH;
			break;
		case EL_ID_BTN:
			element[i].heigth = BTN_STD_HEIGHT;
			element[i].length = BTN_STD_LENGTH;
			break;
		case EL_ID_SW:
			element[i].heigth = SW_STD_HEIGHT;
			element[i].length = SW_STD_LENGTH;
			break;
		case EL_ID_SLI:
			element[i].heigth = SLI_STD_HEIGHT;
			element[i].length = SLI_STD_LENGTH;
			break;
		default:
			break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////
/// \brief gui_redrawElement
/// \param element
/// private helperfunction. Redraws any GUI_ELEMENT corresponding to it’s id.

void gui_redrawElement(GUI_ELEMENT *element)
{
	switch (element->id) {
	case EL_ID_AREA:
		//Do nothing, has to be decided in areas event function
		break;
	case EL_ID_MBTN:	gui_drawMBTN(element);	break;
	case EL_ID_BTN:		gui_drawBTN(element);	break;
	case EL_ID_SW:		gui_drawSW(element);	break;
	case EL_ID_SLI:		gui_drawSLI(element);	break;
	default:									break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////
/// \brief gui_handler
/// \param element
/// Heart of the touchscreen interface. Handles touch queries, checks if any element is
/// touched and possibly sends event to the given element’s event function.

void gui_handler(GUI_ELEMENT element[])
{
	for(u8 i = 0; i < GUI_ELEMENTS_CNT; i++)
	{
		if((element[i].state != GUI_EL_INVISIBLE) &&
		   (element[i].state != GUI_EL_INTOUCHABLE))
		{
			if((Touch_Data.pos.xp > element[i].x) &&
				(Touch_Data.pos.xp < (element[i].x + element[i].length)) &&
				(Touch_Data.pos.yp > element[i].y) &&
				(Touch_Data.pos.yp < (element[i].y + element[i].heigth)))
			{
				if(UB_Touch_OnPressed())
				{
					element[i].event.pressed = 1;
					element[i].action(&element[i].event);
					gui_redrawElement(&element[i]);
					element[i].event.pressed = 0;
				}
				if(UB_Touch_OnDoubleClick())
				{
					element[i].event.doubleclick = 1;
					element[i].action(&element[i].event);
					gui_redrawElement(&element[i]);
					element[i].event.doubleclick = 0;
				}
				if(UB_Touch_OnClick())
				{
					element[i].event.clicked = 1;
					element[i].action(&element[i].event);
					gui_redrawElement(&element[i]);
					element[i].event.clicked = 0;
				}
				if(UB_Touch_OnRelease())
				{
					element[i].event.released = 1;
					element[i].action(&element[i].event);
					gui_redrawElement(&element[i]);
					element[i].event.released = 0;
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
/// \brief gui_clearAREA
/// \param element
/// Clears (sets to background color) the size of the given element.

void gui_clearAREA(GUI_ELEMENT *element)
{
	LCD_Rectangle(element->x,
				  element->y,
				  element->x + element->length,
				  element->y + element->heigth,
				  GUI_COLOR_BACKGROUND, 1);
}

//////////////////////////////////////////////////////////////////////////////////
/// \brief gui_drawMBTN
/// \param element
/// Draws given menubutton.

void gui_drawMBTN(GUI_ELEMENT *element)
{
	if(element->state != GUI_EL_INVISIBLE)
	{
		LCD_Line(element->x,
				 element->y + element->heigth,
				 element->x + element->length,
				 element->y + element->heigth,
				 GUI_COLOR_FONT);

		u16 mbtn_color;
		if(element->state == MBTN_ACTIVE)
			mbtn_color = GUI_COLOR_EL_ACTIVE;
		else
			mbtn_color = GUI_COLOR_EL_UNACTIVE;

		for(u8 i = 1; i < element->heigth + 1; i++)
		{
			LCD_Line(element->x + (i/2),
					 element->y + element->heigth - i,
					 element->x + element->length + (i/2) - 1,
					 element->y + element->heigth - i,
					 mbtn_color);

			LCD_PutPixel(element->x + element->length + (i/2), element->y + element->heigth - i, LCD_COLOR_BLACK);
		}

		UB_Font_DrawPString(element->x + (element->heigth / 2) + 5,
							element->y + element->heigth - (element->font->height/2) - (element->heigth / 2),
							element->label,
							element->font,
							GUI_COLOR_FONT,
							mbtn_color);
	}
	else gui_clearAREA(element);
}

//////////////////////////////////////////////////////////////////////////////////
/// \brief gui_drawBTN
/// \param element
/// Draws given button.

void gui_drawBTN(GUI_ELEMENT *element)
{
	u16 btn_color;
	switch (element->state) {
	case BTN_NOT_ACTIVE:	btn_color = GUI_COLOR_EL_UNACTIVE;	break;
	case BTN_HOVERED:		btn_color = GUI_COLOR_EL_UNACTIVE;	break;
	case BTN_OK:			btn_color = LCD_COLOR_BRIGHTGREEN;	break;
	case BTN_BUSY:			btn_color = LCD_COLOR_BRIGHTYELLOW;	break;
	case BTN_ACTIVE:		btn_color = GUI_COLOR_EL_ACTIVE;	break;
	default:				btn_color = GUI_COLOR_EL_UNACTIVE;	break;
	}

	LCD_Rectangle(element->x,
				  element->y,
				  element->x + element->length - 2,
				  element->y + element->heigth - 2,
				  GUI_COLOR_FONT, 0);

	LCD_Rectangle(element->x + 1,
				  element->y + 1,
				  element->x + element->length - 3,
				  element->y + element->heigth - 3,
				  btn_color, 1);

	LCD_Rectangle(element->x + 2,
				  element->y + element->heigth - 1,
				  element->x + element->length,
				  element->y + element->heigth,
				  GUI_COLOR_FONT, 1);
	LCD_Rectangle(element->x + element->length - 1,
				  element->y + 2,
				  element->x + element->length,
				  element->y + element->heigth - 2,
				  GUI_COLOR_FONT, 1);

	UB_Font_DrawPString(element->x + 5,
						element->y + (element->heigth / 2) - (element->font->height/2),
						element->label, element->font, GUI_COLOR_FONT, btn_color);
}

//////////////////////////////////////////////////////////////////////////////////
/// \brief gui_drawSW
/// \param element
/// Draws given switch.

void gui_drawSW(GUI_ELEMENT *element)
{
	gui_clearAREA(element);

	if(element->state != GUI_EL_INVISIBLE)
	{
		UB_Font_DrawPString(element->x,
							element->y + (element->heigth / 2) - (element->font->height/2),
							element->label, element->font, GUI_COLOR_FONT, GUI_COLOR_BACKGROUND);

		LCD_Rectangle(element->x + (element->length / 2),
					  element->y,
					  element->x + element->length,
					  element->y + element->heigth,
					  GUI_COLOR_FONT, 0);

		if(element->state == SW_OFF)
		{
			LCD_Rectangle(element->x + (element->length / 2) + 2,
						  element->y + 2,
						  element->x + (element->length / 2) + (element->length / 3),
						  element->y + element->heigth - 2,
						  GUI_COLOR_FONT, 0);

			LCD_Rectangle(element->x + (element->length / 2) + 3,
						  element->y + 3,
						  element->x + (element->length / 2) + (element->length / 3) - 1,
						  element->y + element->heigth - 3,
						  LCD_COLOR_BRIGHTRED, 1);

			UB_Font_DrawPString(element->x + element->length - 25,
								element->y + (element->heigth / 2) - (element->font->height/2),
								"Off", element->font, GUI_COLOR_FONT, GUI_COLOR_BACKGROUND);
		}
		else
		{
			LCD_Rectangle(element->x + element->length - (element->length / 3),
						  element->y + 2,
						  element->x + element->length - 2,
						  element->y + element->heigth - 2,
						  GUI_COLOR_FONT, 0);

			if(element->state == SW_BUSY)
			{
				LCD_Rectangle(element->x + element->length - (element->length / 3) + 1,
							  element->y + 3,
							  element->x + element->length - 3,
							  element->y + element->heigth - 3,
							  LCD_COLOR_BRIGHTYELLOW, 1);
			}
			else //On
			{
				LCD_Rectangle(element->x + element->length - (element->length / 3) + 1,
							  element->y + 3,
							  element->x + element->length - 3,
							  element->y + element->heigth - 3,
							  LCD_COLOR_BRIGHTGREEN, 1);

				UB_Font_DrawPString(element->x + (element->length/2) + 5,
									element->y + (element->heigth / 2) - (element->font->height/2),
									"On", element->font, GUI_COLOR_FONT, GUI_COLOR_BACKGROUND);
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
/// \brief gui_drawSLI
/// \param element
/// Draws given slider (TO DO!).

void gui_drawSLI(GUI_ELEMENT *element)
{

}
