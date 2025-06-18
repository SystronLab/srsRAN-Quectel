/*
 ============================================================================
 Filename    : gamepad_interface.c

 Author(s)   : A Moulds

 Version     : Dev: 0.05

 Copyright   : University of York

 Description :

 Revisions:

 Notes:
 Port Number: specified for the robot (RosBot) UE. The robot listens on this port.

 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <linux/input.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <arpa/inet.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include "5g_demo1_consts.h"


#define DEFAULT_IP_ADDR "127.0.0.1"
#define DEMO_SEND_INTERVAL 3 // secs
//#define GAMEPAD_DRIVER_DEVICE_NAME "usb-D_TGZ_Controller_3E529610-event-joystick"
#define GAMEPAD_DRIVER_DEVICE_NAME "usb-D_TGZ_Controller_3E5296C0-event-joystick"
#define GAMEPAD_DRIVER_LOC "/dev/input/by-id/"


typedef struct
{
    int button_A_down;
    int button_B_down;
    int button_X_down;
    int button_Y_down;
    int stick_left_x;
    int stick_left_y;
    int stick_right_x;
    int stick_right_y;
} gamepadStruct, *gamepadStructptr;

int gamepad = -1;
char gamepadDevname[] = GAMEPAD_DRIVER_DEVICE_NAME;
char gamepadDevLoc[] = GAMEPAD_DRIVER_LOC;
char gamepadDataJson[GAMEPAD_JSON_PKTSIZE];

/*
*
*/
void jsonAppend(char* str)
{
	static int idx;
	if(str == NULL)
	{
		memset(gamepadDataJson, 0, sizeof(gamepadDataJson));
		idx = 0;
	}
	else
	{
		int len = strlen(str);
		strncat(&gamepadDataJson[idx], str, len);
		idx += len;
	}
}

/*
*
*/
char *intToStr(int x)
{
	static char buf[10];
	sprintf(buf, "%d", x);
	return &buf[0];
}

/*
*
*/
void writeGamepadDataJson(gamepadStructptr gamepad)
{
	char str[10];
	jsonAppend(NULL);
	jsonAppend("{");
	jsonAppend("\n\t\"ButtonA\":"); jsonAppend(intToStr(gamepad->button_A_down));
	jsonAppend(",\n\t\"ButtonB\":"); jsonAppend(intToStr(gamepad->button_B_down));
	jsonAppend(",\n\t\"ButtonX\":"); jsonAppend(intToStr(gamepad->button_X_down));
	jsonAppend(",\n\t\"ButtonY\":"); jsonAppend(intToStr(gamepad->button_Y_down));
	jsonAppend(",\n\t\"Left_X\":"); 
	sprintf(str, "\"%+06i\"", gamepad->stick_left_x);
	jsonAppend(str);
	jsonAppend(",\n\t\"Left_Y\":"); 
	sprintf(str, "\"%+06i\"", gamepad->stick_left_y);
	jsonAppend(str);
	jsonAppend(",\n\t\"Right_X\":"); 
	sprintf(str, "\"%+06i\"", gamepad->stick_right_x);
	jsonAppend(str);
	jsonAppend(",\n\t\"Right_Y\":"); 
	sprintf(str, "\"%+06i\"", gamepad->stick_right_y);
	jsonAppend(str);
	jsonAppend("\n}");
}


/*
*
*/
void printGamepadDataJson(void)
{
	system("clear");
	puts("*** Gamepad Interface - Test Program ***");
	printf("%s\n",gamepadDataJson);
}


/*
*
*/
void sigHandler(int sigNo)
{
   if(sigNo == SIGINT)
   {
	   if(gamepad >= 0)
		   close(gamepad);
	   exit(EXIT_SUCCESS);
   }
}


/* ==================================================================================== */
int main(int argc, char **argv)
{

	int robot_port_number = DEFAULT_PORT_NO;
	char robot_ip_addr_str[16];

	int test_loop_count = 0;

	struct sockaddr_in robot_addr;

	signal(SIGINT, sigHandler);

	// Get Options //
	int opt;
	char mode[5];
	strncpy(mode,"norm",4);
	strcpy(robot_ip_addr_str,DEFAULT_IP_ADDR);
	while((opt = getopt(argc, argv, "hi:p:m:")) != -1)
	{
	   switch (opt)
	   {
	     case 'h':
	    	 printf("Usage: gamepad_test [OPTION]\n");
	    	 printf("Sends gamepad control signals to robot over 5G network.\n\n");
	    	 printf("  -i: robot ip address - format xxx.xxx.xxx.xxx\n");
	    	 printf("  -p: robot port number integer\n\n");
			 printf("  -h: help\n");
			 printf("  -m: set mode to norm (default) or demo\n");
	    	 printf("Example:\ngamepad_test -i 192.100.0.1 -p 99700\n\n");
	    	 exit(EXIT_SUCCESS);
	    	 break;
		 case 'm':
	         strncpy(mode,optarg,4);	
			 if(strncmp(mode, "demo",4) && strncmp(mode,"norm",4))
			 	strncpy(mode,"norm",4);
			break;
	     case 'i':
	    	 strcpy(robot_ip_addr_str, optarg);
	         break;
         case 'p':
	         robot_port_number = atoi(optarg);
	         break;
	     default:
	    	 printf("error: unknown argument!\n");
	    	 exit(EXIT_FAILURE);
	   }
   }

   	bzero(&robot_addr, sizeof(struct sockaddr_in));
	robot_addr.sin_family = AF_INET;
	robot_addr.sin_addr.s_addr = inet_addr(robot_ip_addr_str);
	robot_addr.sin_port = htons(robot_port_number);

	int sockfd;
	if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
	    printf("Error while creating the socket\n");
	    exit(EXIT_FAILURE);
	}

	if(connect(sockfd, (struct sockaddr *) &robot_addr, sizeof(robot_addr)) < 0)
	{
		printf("error: socket connection failed!\n");
		close(sockfd);
		exit(EXIT_FAILURE);
	}


	puts("*** Gamepad Interface for 5G ORAN Demo1 **");

	if(!strncmp(mode,"norm",4))
	{
		gamepad = open(strcat(gamepadDevLoc,gamepadDevname), O_RDONLY | O_NONBLOCK);
		if(gamepad == -1)
		{
			printf("error: unable to open gamepad device!\n");
			goto FINISH;
		}
	}

    gamepadStruct gamepadVal = {0};
    gamepadStruct gamepadValOld = {0};

    while(1)
    {
		if(!strncmp(mode,"norm",4))
		{
			struct input_event events[8]; // 8 events per frame is pretty extreme, more like 1 or 2, sometimes as high as 4
			int r1 = read(gamepad, events, sizeof events);
			if (r1 != -1)
			{
				int new_event_count = r1/sizeof(struct input_event);
				for (int evi=0; evi<new_event_count; evi++)
				{
					struct input_event ev = events[evi];
					switch (ev.type)
					{
						case EV_ABS:
						{
							switch (ev.code)
							{
								case ABS_X: { gamepadVal.stick_left_x = ev.value; } break;
								case ABS_Y: { gamepadVal.stick_left_y = ev.value; } break;
								case ABS_RX: { gamepadVal.stick_right_x = ev.value; } break;
								case ABS_RY: { gamepadVal.stick_right_y = ev.value; } break;
							}
						} break;
						case EV_KEY:
						{
							switch (ev.code)
							{
								case BTN_A: { gamepadVal.button_A_down = ev.value; } break;
								case BTN_B: { gamepadVal.button_B_down = ev.value; } break;
								case BTN_X: { gamepadVal.button_X_down = ev.value; } break;
								case BTN_Y: { gamepadVal.button_Y_down = ev.value; } break;

							}
						} break;
					}
				}
				// Test if any changes in gamepad axis values or button states //
				if(gamepadValOld.button_A_down != gamepadVal.button_A_down ||
						gamepadValOld.button_B_down != gamepadVal.button_B_down ||
						gamepadValOld.button_X_down != gamepadVal.button_X_down ||
						gamepadValOld.button_Y_down != gamepadVal.button_Y_down ||
						gamepadValOld.stick_left_x != gamepadVal.stick_left_x ||
						gamepadValOld.stick_left_y != gamepadVal.stick_left_y ||
						gamepadValOld.stick_right_x != gamepadVal.stick_right_x ||
						gamepadValOld.stick_right_y != gamepadVal.stick_right_y)
				{
					writeGamepadDataJson(&gamepadVal);
					printGamepadDataJson();
					if((sendto(sockfd, 
						gamepadDataJson, 
						sizeof(gamepadDataJson), 
						0, // flags
						NULL,
						NULL)) < 0)
					{
						printf("error: unable to send data!n");
						perror("- ");
					}
				}
			}

			gamepadValOld = gamepadVal;
		}
		else if(!strncmp(mode,"demo",4))
		{
			gamepadVal.button_A_down = 0;
			gamepadVal.button_B_down = 0;
			gamepadVal.button_X_down = 0;
			gamepadVal.button_Y_down = 0;
			gamepadVal.stick_left_x = 0;
			gamepadVal.stick_left_y = 0;
			gamepadVal.stick_right_x = 0;
			gamepadVal.stick_right_y = 0;

			sleep(DEMO_SEND_INTERVAL);
			writeGamepadDataJson(&gamepadVal);
			printGamepadDataJson();
			if((sendto(sockfd, gamepadDataJson, sizeof(gamepadDataJson), 0,
				(struct sockaddr *) &robot_addr,
				sizeof(robot_addr))) < sizeof(gamepadDataJson))
			{
				printf("error: unable to send data!n");
				perror("- ");
			}
			test_loop_count++;
			printf("loop count = %d\n", test_loop_count);
		}

    }

FINISH:
	if(gamepad >= 0)
		close(gamepad);
	close(sockfd);
	return EXIT_SUCCESS;
}
