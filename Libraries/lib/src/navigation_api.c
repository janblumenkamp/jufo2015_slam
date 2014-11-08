/////////////////////////////////////////////////////////////////////////////////
/// Navigation API - all the waypoint stack management and waypoint management functions
/////////////////////////////////////////////////////////////////////////////////


#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "navigation.h"
#include "navigation_api.h"
#include "printf.h"

nav_waypoint_t nav_waypoints[WP_STACKSIZE]; //Waypoint stack
nav_waypoint_t *nav_wpStart, *nav_wpEnd; //Pointer to first and last waypoint
int16_t nav_lastID = 0; //Always incrementing ID - increments after every attached element
u_int16_t nav_wpAmount = 0; //Contains the amount of the current used waypoints

//////////////////////////////////////////////////////////////////////
/// \brief nav_resetWp
///			Resets the given waypoint to the uninitalized start values
/// \param wp
///			Given waypoint
///
void nav_resetWp(nav_waypoint_t *wp)
{
	if(wp != NULL)
	{
		wp->x = -1;
		wp->y = -1;
		wp->z = -1;
		wp->id = -1;
		wp->previous = NULL;
		wp->next = NULL;
	}
	//else: To do: ERROR...
}

////////////////////////////////////////////////////////////////////
/// \brief nav_initWaypointArray
///			Initializes the waypoint stack
///
void nav_initWaypointStack(void)
{
	for(u_int16_t i = 0; i < WP_STACKSIZE; i++) //Init struct
	{
		nav_resetWp(&nav_waypoints[i]);
	}
	nav_wpStart = NULL;
	nav_wpEnd = NULL;
	nav_wpAmount = 0;
	nav_lastID = 0;
}

//////////////////////////////////////////////////////////////////////
/// \brief nav_stackFindEmptyElement
///			Returns pointer to first unused element on stack. This function
///			is element of the navigation api hardware abstraction layer/
///			the memory management. You could also replace the content by
///			a dynamical reservation (malloc(sizeof(nav_waypoint_t)), but
///			dynamical reservation makes the program non-deterministical...
/// \return
///			Pointer to free stack element
///
nav_waypoint_t *nav_stackFindEmptyElement(void)
{
	for(u_int16_t i = 0; i < WP_STACKSIZE; i++) //Find first unused element in wapoint list
	{
		if(nav_waypoints[i].id == -1)
		{
			nav_wpAmount++;
			return &nav_waypoints[i];
		}
	}

	return NULL; //Usually, program should NEVER come until here - if it does, there is no more space available*/
}

///////////////////////////////////////////////////////////////////////
/// \brief nav_stackFree
///			Opposite to nav_stackFindEmptyElement. If you use dynamical
///			reservation, you would have to replace reset_wp by free(wp).
/// \param wp
///			Waypoint to free
///
void nav_stackFree(nav_waypoint_t *wp)
{
	nav_resetWp(wp);

	nav_wpAmount--;
}

////////////////////////////////////////////////////////////////////////
/// \brief nav_attachWaypoint
///			Adds new waypoint to the list
/// \param wp
///			Waypoint to add
///

///Helperfunction; Copies data (AND ONLY DATA!) from a to b
void attWp_copy(nav_waypoint_t *wp_a, nav_waypoint_t *wp_b)
{
	wp_b->x = wp_a->x;
	wp_b->y = wp_a->y;
	wp_b->z = wp_a->z;
	wp_b->id = nav_lastID++; //Add id, then increment global id
}

void nav_attachWaypoint(nav_waypoint_t *wp)
{
	nav_waypoint_t *penultimate_el; //Pointer to acces penultimate element in the list

	if(nav_wpStart == NULL) //There are no elements in the list, because the start pointer points to NULL
	{
		nav_wpStart = nav_stackFindEmptyElement(); //Reserve first element from stack
		if(nav_wpStart == NULL) //couldn’t reserve element. Stack too small?
		{
			//fprintf(stderr,"Kein Speicherplatz vorhanden für start\n");
			//return;
		}
		else
		{
			attWp_copy(wp, nav_wpStart); //Copy data from given waypoint

			nav_wpStart->next = NULL; //Next pointer points to next element in list, but the start element is now the first and last element, so it points to NULL
			nav_wpEnd = nav_wpStart; //The first element is also the last element
			nav_wpEnd->previous = NULL; //Just like start->next
		}
	}
	else //There is at least one element in the list
	{
		nav_wpEnd->next = nav_stackFindEmptyElement(); //Reserve element from stack for the element after end
		if(nav_wpEnd->next == NULL)
		{
			//fprintf(stderr,"Kein Speicherplatz für letztes Element\n");
			//return;
		}
		else
		{
			penultimate_el = nav_wpEnd; //Now, the last element became the penultimate element (because we attached one more element to the end of the list)
			nav_wpEnd = nav_wpEnd->next; //And the ultimate element points to the just reserved stack element
			nav_wpEnd->next = NULL; //The next pointer of the ultimate element (end element) has to point to NULL again
			nav_wpEnd->previous = penultimate_el; //The previous pointer of the last element has to point to the penultimate element
			penultimate_el->next = nav_wpEnd; //And inversed...

			attWp_copy(wp, nav_wpEnd); //Copy data from given waypoint
		}
	}
}

////////////////////////////////////////////////////////////////
/// \brief nav_deleteWaypoint
///			Deletes waypoint with the given id from the list
/// \param id
///			ID of the waypoint to delete
void nav_deleteWaypoint(int16_t id)
{
	nav_waypoint_t *del_el; //Element to delete

	if(nav_wpStart != NULL) //Are elements in the List?
	{
		if(nav_wpStart->id == id) //Is the start id the element we want to delete?
		{
			if(nav_wpStart->next == NULL) //Are not more than one elements in the list?
			{
				nav_stackFree(nav_wpStart);
				nav_wpStart = NULL;
				nav_wpEnd = NULL;
			}
			else
			{
				nav_wpStart->next->previous = NULL;
				nav_stackFree(nav_wpStart);
				nav_wpStart = nav_wpStart->next;
			}
		}
		else if(nav_wpEnd->id == id) //Do we want to delete the last element?
		{
			nav_wpEnd->previous->next=NULL;
			nav_stackFree(nav_wpEnd);
			nav_wpEnd = nav_wpEnd->previous;
		}
		else //We want to delete an element between end and start.
		{
			del_el = nav_wpStart;

			for(u_int16_t i = 0; i < WP_STACKSIZE; i++)
			{
				if(del_el->id == id)
				{
					del_el->previous->next = del_el->next;
					del_el->next->previous = del_el->previous;

					nav_stackFree(del_el);
					break; //Found ID - stop for-loop
				}
				else
				{
					del_el = del_el->next;
					if(del_el == NULL) //End of list - stop loop (Element not found).
						break;
				}
			}
		}
	}
	else
	{
		//printf("Es sind keine Daten zum löschen vorhanden!!!\n");
		//To do: Debug...
	}
}

//////////////////////////////////////////////////////////////////////////////
/// \brief nav_getWaypoint
///			returns the waypoint with the given id
/// \param id
///			ID of the desired waypoint
/// \return Pointer to waypoint with the given ID
///
nav_waypoint_t *nav_getWaypoint(int16_t id)
{
	nav_waypoint_t *get_el = nav_wpStart; //Element to find (index)

	if(nav_wpStart != NULL) //Are elements in the List?
	{
		for(u_int16_t i = 0; i < WP_STACKSIZE; i++)
		{
			if(get_el->id == id)
			{
				return get_el;
			}
			else
			{
				get_el = get_el->next;
				if(get_el == NULL) //End of list - stop loop (Element not found).
					return NULL;
			}
		}
	}
	else
	{
		//printf("Es sind keine Daten zum löschen vorhanden!!!\n");
		//To do: Debug...
	}

	return NULL;
}
