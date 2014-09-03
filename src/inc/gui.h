#ifndef GUI_H
#define GUI_H

enum GUI_ELEMENTS {
	GUI_EL_AREA_STATUSBAR_TOP, //Statusbar in the top of the display
	GUI_EL_AREA_STATUSBAR_DROPPED, //Only the space needed if dropped - this area is cleared if dropped

	GUI_EL_AREA_CONTENT, //Area below the menu buttons - only for content like map or buttons

	GUI_EL_MBTN_MAP, //Menubutton Map
		GUI_EL_SW_STARTMAPPING, //Switch - start/stop robot/mapping
		//GUI_EL_SLI_MAP_SCALE, //Scale of the map on the display
		GUI_EL_AREA_MAP, //Area of the map

	GUI_EL_MBTN_VIEW, //Menubutton View/Info (Sensordata/debugging)

	GUI_EL_MBTN_SETTINGS, //Menubutton for settings like lidar on/off etc.
		GUI_EL_BTN_CALTOUCH, //Calibrate Touchscreen
		GUI_EL_BTN_RESET, //Reset device - doubleclick nessesary!
		GUI_EL_SW_LIDAR, //Lidar on/off
	//////////DO NOT ADD ANYTHING BELOW THIS LINE!!!////////////
	GUI_ELEMENTS_CNT
};

enum GUI_MENU {
	MENU_INIT, MENU_INIT_IDLE, //Draw whole menu (also the buttons) and, corresponding which button is pressed, draw button and menu
	MENU_MAP_INIT, MENU_MAP_IDLE,
	MENU_VIEW_INIT, MENU_VIEW_IDLE,
	MENU_SETTINGS_INIT, MENU_SETTINGS_IDLE,
	MENU_CALIBRATION
};

extern void gui_init(void);

#endif // GUI_H
