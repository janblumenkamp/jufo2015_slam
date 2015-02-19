////////////////////////////////////////////////////////////////////////////////
/// gui.h - Definitions of Data structures etc. of the graphical user interface
///
/// - ALL elements of the GUI are enumerated in GUI_ELEMENTS
/// - The states of the menu/page statemachine are enumerated in GUI_MENU
///
/// ADDING NEW ELEMENTS:
/// 1)	Add the element in the given format (see description of GUI_ELEMENTS) in
///		the list and in the structure they are going to be placed later (only
///		important due to the clarity)
/// 2)	In gui.c: Add the element’s ID (is it a button, a switch...) to the
///		List at the very beginning of gui_init (in gui_init before graphics_init,
///		here the default values are stored in the gui_element structure)
/// 3)	Create the event function of the new element in gui.c (on the top in
///		the near and due to the clarity in the same structure like defined in
///		GUI_ELEMENTS). This function is called if the element is touched or
///		any other event occurs. The occured event is transmitted via the
///		ELEMENT_EVENT *event struct, so every function has to be in the format
///		void event_name(ELEMENT_EVENT *event).
///		Also add the private function prototype in the beginning of gui.c!
/// 4)	Make user defined settings: For every element you have to select the
///		x/y position and the length. Corresponding to the selected ID you also
///		have to select the other given variables like the font, the label etc.
///		If existing, don’t forget to add the event function in GUI_ELEMENT.event
///		(pointer to the function)
/// 5)	At last, the elements state has to be set to GUI_EL_INVISIBLE in the
///		gui_el_pages_putInvisible() function in gui.c, if the element is a element
///		of a page (if it is a sub-element of one of the MBTN Buttons). This is
///		important to disable the event functions of the not-active elements
///		(if the page is changed or the statusbar is dropped, all elements are
///		are firstly set as inactive and then in the menu statemachine corresponding
///		init state of the new page resetted to the active state).
/// 5)	Now the settings are done. The next step are the initialisation in the
///		menu state machine. The state of the element has to be initialized (the
///		MENU_INIT states are always called if the page was switched to the new,
///		corresponding page). If one element (especially areas!) are waiting for
///		a not-touch-event, they has to be redrawn in the page-IDLE state of the menu
///		statemachine. The elements (except for area-elements!) are automatically
///		redrawn in the gui_handler if they were touched (if the event functions
///		were executed).
//////////////////////////////////////////////////////////////////////////////////////

#ifndef GUI_H
#define GUI_H

extern u8 mapping; //Is the robot running and mapping or is it waiting for the start?
extern u8 setWaypoints;
extern u8 processedView;
///////////////////////////////////////////////////
/// \brief The GUI_ELEMENTS enum
/// All GUI elements
/// are defined here.

enum GUI_ELEMENTS {
	GUI_EL_AREA_STATUSBAR_TOP, //Statusbar in the top of the display
	GUI_EL_AREA_STATUSBAR_DROPPED, //Only the space needed if dropped - this area is cleared if dropped

	GUI_EL_AREA_CONTENT, //Area below the menu buttons - only for content like map or buttons

	GUI_EL_MBTN_MAP, //Menubutton Map
		GUI_EL_SW_STARTMAPPING, //start/stop robot/mapping
		GUI_EL_SW_SHOWSCAN, //Show the live-scan of the lidar in the map
		//GUI_EL_SLI_MAP_SCALE, //Scale of the map on the display
		GUI_EL_SW_PROCESSEDVIEW, //Processed or raw view of the map?
		GUI_EL_BTN_CLEARMAP, //Delete the map
		GUI_EL_BTN_SETWP, //Setting new waypoints by touching map?
		GUI_EL_AREA_MAP, //Area of the map

	GUI_EL_MBTN_VIEW, //Menubutton View/Info (Sensordata/debugging)

	GUI_EL_MBTN_SETTINGS, //Menubutton for settings like lidar on/off etc.
		GUI_EL_BTN_CALTOUCH, //Calibrate Touchscreen
		GUI_EL_BTN_RESET, //Reset device - doubleclick nessesary!
		GUI_EL_SW_LIDAR, //Lidar on/off
		GUI_EL_SW_STRLIDAR, //Stream lidar raw data via bluetooth
		GUI_EL_SW_STRSLAMUI, //SlamUI stream on/off
		GUI_EL_SW_STRDEBUGOS, //debugOS stream on/off
		GUI_EL_SW_STRDEBUG, //debug stream on/off
		GUI_EL_SW_STRERR, //error stream on/off


	//////////DO NOT ADD ANYTHING BELOW THIS LINE!!!////////////
	GUI_ELEMENTS_CNT
};

#define MAP_REFRESHTIME 4 //*50ms. Refreshrate of Map visualization.

///////////////////////////////////////////////////
/// \brief The GUI_MENU enum
/// All menu statemachine
/// states are defined here.

enum GUI_MENU {
	MENU_INIT, MENU_INIT_IDLE, //Draw whole menu (also the buttons) and, corresponding which button is pressed, draw button and menu
	MENU_MAP_INIT, MENU_MAP_IDLE,
	MENU_VIEW_INIT, MENU_VIEW_IDLE,
	MENU_SETTINGS_INIT, MENU_SETTINGS_IDLE,
	MENU_CALIBRATION
};

///Initialisation of the gui (has to be called while booting)
extern void gui_init(void);

#endif // GUI_H
