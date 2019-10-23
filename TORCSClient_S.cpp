/*
 * TORCS Client for Simulink
 * By: Ahmed M. Moustafa
 *
*/

#define S_FUNCTION_NAME TORCSClient_S
#define S_FUNCTION_LEVEL 2

#define nonblockingsocket(s) {unsigned long ctl = 1;ioctlsocket( s, FIONBIO, &ctl );}
#define _WINSOCK_DEPRECATED_NO_WARNINGS 1

#define UDP_MSGLEN 1000
#define UDP_CLIENT_TIMEUOT 1000000

//#include <windows.h>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <winsock2.h>

#include "simstruc.h"
#include "matrix.h"

#include "CarState.h"
#include "CarControl.h"
#include "SimpleParser.h"


// Type enumator for socket connection state
typedef enum {csError, csConnected} ConnectionState;

// Type struct for socket socket_information
typedef struct SocketStruct_
{
	SOCKET socketDescriptor;
	ConnectionState connState;
	struct sockaddr_in sockAddrIn;
	int addrLen;
} SocketStruct;



/* Initialize PWork pointers array for our global variables
	PWork[0] (Type: char*)         = Remote IP address
	PWork[1] (Type: unsigned int*) = Port number
	PWork[2] (Type: SocketStruct*) = Socket structure socket_information
	PWork[3] (Type: char*)         = Message buffer
*/
void InitPWorkPointers(SimStruct *S)
{
	void **PWork = ssGetPWork(S);

	char *ip_addr = (char *)malloc(16 * sizeof(unsigned char));
	unsigned short* port_num = (unsigned short *)malloc(sizeof(unsigned short));
	SocketStruct *sock_struct = (SocketStruct *) malloc(sizeof(SocketStruct));
	char *buffer = (char *) malloc(UDP_MSGLEN * sizeof(unsigned char));
	CarControl *car_control = new CarControl();
	CarState *car_state = new CarState();
	
	PWork[0] = (void *) ip_addr;
	PWork[1] = (void *) port_num;
    PWork[2] = (void *) sock_struct;
	PWork[3] = (void *) buffer;


	
}

/* Delete and free PWork pointers array for our global variables */
void DeletePWorkPointers(SimStruct *S)
{
	void **PWork = ssGetPWork(S);
	
	char *ip_addr = (char *) PWork[0];
	unsigned short* port_num = (unsigned short *) PWork[1];
	SocketStruct *sock_struct = (SocketStruct *) PWork[2];
	char *buffer = (char *) PWork[3];
	CarControl *car_control = (CarControl *) PWork[4];
	CarState *car_state = (CarState *) PWork[5];
	
	closesocket(sock_struct->socketDescriptor);

	free(ip_addr);
	free(port_num);
	free(sock_struct);
	free(buffer);
}


/* mdlCheckParameters: Verifies new parameter settings whenever parameters change or are reevaluated during a simulation */
#define MDL_CHECK_PARAMETERS   // Change to #undef to remove function 
#if defined(MDL_CHECK_PARAMETERS)
static void mdlCheckParameters(SimStruct *S)
{
    char ip_addr[16];
	unsigned long addr_stat = INADDR_NONE;
	unsigned short port_num;

	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
	{
		ssSetErrorStatus(S, "Error in S-function block parameters");
		return;
	}

	// Check first parameter (IP address) format
	mxGetString(ssGetSFcnParam(S,0), ip_addr, 16);
	addr_stat = inet_addr(ip_addr);
	if (addr_stat == INADDR_NONE)
	{
		ssSetErrorStatus(S, "Invalid IP address format"); 
		return;
	}

    // Check port number dimension
    if ( mxGetNumberOfElements(ssGetSFcnParam(S,1)) != 1 )
    { 
		ssSetErrorStatus(S,"Port number must be a scalar value"); 
		return; 
	}

    // Check port number range
    port_num = (unsigned short) (*mxGetPr(ssGetSFcnParam(S,1)));
	if ( (port_num < 1025) || (port_num > 65535) )
    { 
		ssSetErrorStatus(S,"Port number must range from 1025 to 65535"); 
		return; 
	}
}
#endif

/* mdlInitializeSizes: Initialize sizes for states, ports, and work arrays */
static void mdlInitializeSizes(SimStruct *S)
{
	// Number of S-function parameters (IP address + Port Num + Sample Time = 3)
	ssSetNumSFcnParams(S,3);                          

	// Check S-Function block parameters	
    mdlCheckParameters(S);	
	if (ssGetErrorStatus(S) != NULL)
		return; 

	
	ssSetNumContStates(S, 0); // No continuous states
	ssSetNumDiscStates(S, 0); // No discrete states   


	// Number of input ports       
    if (!ssSetNumInputPorts(S,7)) 
		return;            

	ssSetInputPortWidth(S, 0, 1); // accel input port width 
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDataType(S, 0, SS_SINGLE);	// Accel input port data type 

	ssSetInputPortWidth(S, 1, 1); // brake input port width              
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDataType(S, 1, SS_SINGLE);	// Brake input port data type 

	ssSetInputPortWidth(S, 2, 1); // gear input port width              
	ssSetInputPortDirectFeedThrough(S, 2, 1);
	ssSetInputPortDataType(S, 2, SS_INT32);	// Gear input port data type 

	ssSetInputPortWidth(S, 3, 1); // steer input port width              
	ssSetInputPortDirectFeedThrough(S, 3, 1);
	ssSetInputPortDataType(S, 3, SS_SINGLE);	// Steer input port data type 

	ssSetInputPortWidth(S, 4, 1); // clutch input port width              
	ssSetInputPortDirectFeedThrough(S, 4, 1);
	ssSetInputPortDataType(S, 4, SS_SINGLE);	// Clutch input port data type 

	ssSetInputPortWidth(S, 5, 1); // focus input port width              
	ssSetInputPortDirectFeedThrough(S, 5, 1);
	ssSetInputPortDataType(S, 5, SS_INT32);	// Focus input port data type 

	ssSetInputPortWidth(S, 6, 1); // meta input port width
	ssSetInputPortDirectFeedThrough(S, 6, 1);
	ssSetInputPortDataType(S, 6, SS_INT32);	// meta input port data type 


	// Number of output ports
	if (!ssSetNumOutputPorts(S, 19)) 
		return;  
	
	ssSetOutputPortWidth(S, 0, 1); // angle output port width              
	ssSetOutputPortDataType(S, 0, SS_SINGLE);// angle output port data type

	ssSetOutputPortWidth(S, 1, 1); // curLapTime output port width              
	ssSetOutputPortDataType(S, 1, SS_SINGLE);// curLapTime output port data type  

	ssSetOutputPortWidth(S, 2, 1); // damage output port width              
	ssSetOutputPortDataType(S, 2, SS_SINGLE);// damage output port data type  

	ssSetOutputPortWidth(S, 3, 1); // distFromStart output port width              
	ssSetOutputPortDataType(S, 3, SS_SINGLE);// distFromStart output port data type  

	ssSetOutputPortWidth(S, 4, 1); // distRaced output port width              
	ssSetOutputPortDataType(S, 4, SS_SINGLE);// distRaced output port data type  

	ssSetOutputPortWidth(S, 5, FOCUS_SENSORS_NUM); // focus output port width              
	ssSetOutputPortDataType(S, 5, SS_SINGLE);// focus output port data type  

	ssSetOutputPortWidth(S, 6, 1); // fuel output port width              
	ssSetOutputPortDataType(S, 6, SS_SINGLE);// fuel output port data type  

	ssSetOutputPortWidth(S, 7, 1); // gear output port width              
	ssSetOutputPortDataType(S, 7, SS_INT32);// gear output port data type  

	ssSetOutputPortWidth(S, 8, 1); // lastLapTime output port width              
	ssSetOutputPortDataType(S, 8, SS_SINGLE);// lastLapTime output port data type  

	ssSetOutputPortWidth(S, 9, OPPONENTS_SENSORS_NUM); // opponents output port width              
	ssSetOutputPortDataType(S, 9, SS_SINGLE);// opponents output port data type  

	ssSetOutputPortWidth(S, 10, 1); // racePos output port width              
	ssSetOutputPortDataType(S, 10, SS_INT32);// racePos output port data type  

	ssSetOutputPortWidth(S, 11, 1); // rpm output port width              
	ssSetOutputPortDataType(S, 11, SS_INT32);// rpm output port data type  

	ssSetOutputPortWidth(S, 12, 1); // speedX output port width              
	ssSetOutputPortDataType(S, 12, SS_SINGLE);// speedX output port data type  

	ssSetOutputPortWidth(S, 13, 1); // speedY output port width              
	ssSetOutputPortDataType(S, 13, SS_SINGLE);// speedY output port data type  

	ssSetOutputPortWidth(S, 14, 1); // speedZ output port width              
	ssSetOutputPortDataType(S, 14, SS_SINGLE);// speedZ output port data type  

	ssSetOutputPortWidth(S, 15, TRACK_SENSORS_NUM); // track output port width              
	ssSetOutputPortDataType(S, 15, SS_SINGLE);// track output port data type  

	ssSetOutputPortWidth(S, 16, 1); // trackPos output port width              
	ssSetOutputPortDataType(S, 16, SS_SINGLE);// trackPos output port data type  

	ssSetOutputPortWidth(S, 17, 4); // wheelSpinVel output port width              
	ssSetOutputPortDataType(S, 17, SS_SINGLE);// wheelSpinVel output port data type  

	ssSetOutputPortWidth(S, 18, 1); // z output port width              
	ssSetOutputPortDataType(S, 18, SS_SINGLE);// z output port data type  


    ssSetNumSampleTimes(S,0);  // number of sample times             

    ssSetNumRWork(S,0); // number real work vector elements 
    ssSetNumIWork(S,0); // number int_T work vector elements  
    ssSetNumPWork(S,4); // number ptr work vector elements
    ssSetNumModes(S,0); // number mode work vector elements
    
	ssSetNumNonsampledZCs(S,0); // number of nonsampled zero crossing

	


}

/* mdlInitializeSampleTimes: Initialize the sample times array */
static void mdlInitializeSampleTimes(SimStruct *S)
{

	double *sample_time = mxGetDoubles(ssGetSFcnParam(S,2));
    
	// Set sample time
    ssSetSampleTime(S, 0, sample_time[0]);
    ssSetOffsetTime(S, 0, 0);
}


/* Init_TORCS_Client: Initialize UDP client for TORCS, called from mdlStart */
void Init_TORCS_Client(SimStruct *S, SocketStruct *socket_info)
{
	void **PWork = ssGetPWork(S);

	SOCKET client_sock;
    char* ip_addr;
	unsigned short port_num;

	
	ip_addr = (char *)PWork[0];
	
	port_num = *((unsigned short *)PWork[1]);

	socket_info->connState = csError;
	
	socket_info->addrLen = sizeof(socket_info->sockAddrIn);
	memset((void *)&(socket_info->sockAddrIn), 0, socket_info->addrLen); // Initialize socket structure

	if ((socket_info->sockAddrIn.sin_addr.s_addr = inet_addr(ip_addr)) == INADDR_NONE)
	{
		ssSetErrorStatus(S, "Error in Init_TORCS_Client: Invalid IP address\n");
		socket_info->connState = csError;
		return;
	}

	// Address Family: Same as Protocol Family 
	socket_info->sockAddrIn.sin_family = AF_INET;

	// Insert port number in address (Check byte-ordering)
	socket_info->sockAddrIn.sin_port = htons(port_num);

	client_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (client_sock == INVALID_SOCKET)
	{
		ssSetErrorStatus(S, "Error in Init_TORCS_Client: Invalid socket\n");
		return;
	}
	

	socket_info->socketDescriptor = client_sock;

	socket_info->connState = csConnected;

	if(socket_info->connState==csConnected)
		nonblockingsocket(socket_info->socketDescriptor);
}

/* mdlStart: Initialize work vectors and start identification for connection */
#define MDL_START
static void mdlStart(SimStruct *S)
{
	WSADATA wsa_data;
	int status;
	char status_str[50];
    int sock_error;
    int num_read;
	
    fd_set readSet;
    struct timeval time_val;
    
	// Initialize work pointers array
	InitPWorkPointers(S);
	void **PWork = ssGetPWork(S);

	char ip_addr[16];
	mxGetString(ssGetSFcnParam(S, 0), ip_addr, 15);
	strcpy((char *)PWork[0], ip_addr);
	
	unsigned short port_num = (unsigned short)(*mxGetPr(ssGetSFcnParam(S, 1)));
	unsigned short* port_num_ptr = (unsigned short*)PWork[1];
	*port_num_ptr = port_num;
	
	SocketStruct *socket_info = (SocketStruct *) PWork[2];

	char *buffer = (char *) PWork[3];

	CarControl *car_control = (CarControl *)PWork[4];
	CarState *car_state = (CarState *)PWork[5];

	// Activate Winsock DLL 
	if ((status = WSAStartup(MAKEWORD(2, 2), &wsa_data)) != 0)
	{ 
		sprintf(status_str, "%d is the WSA startup error\n", status);
		ssSetErrorStatus(S, status_str);
	}
	
	
	Init_TORCS_Client(S, socket_info);
   
    
    
    /***********************************************************************************
     ************************* UDP client identification *******************************
     ***********************************************************************************/
	int i = 1000; // Counter for number of trials
	do
    {
        // Initialize the angles of range finders
		float angles[19] = {-90.0, -63.0, -45.0, -35.0, -23.0, -15.0, -11.0, -6.0, -3.0, 0.0, 
							3.0, 6.0, 11.0, 15.0, 23.0, 35.0, 45.0, 63.0, 90.0};
        
		string initString = SimpleParser::stringify(string("init"),angles,19);
        cout << "Sending id to server: " << "SCR" << endl;
        initString.insert(0,"SCR");
        cout << "Sending init string to the server: " << initString << endl;
        
        if (sendto(socket_info->socketDescriptor, initString.c_str(), initString.length(), 0,
                (const sockaddr *) &(socket_info->sockAddrIn), socket_info->addrLen)<0)
        {
            sock_error = WSAGetLastError();
            if(sock_error == WSAESHUTDOWN || sock_error == WSAENOTCONN ||
                    sock_error == WSAEHOSTUNREACH || sock_error == WSAECONNABORTED ||
                    sock_error == WSAECONNRESET || sock_error == WSAETIMEDOUT)
            {
                socket_info->connState = csError;
            }
			sprintf(status_str, "Error code: %d\n", WSAGetLastError());
			ssSetErrorStatus(S, status_str);
        }
        
        // Wait until answer comes back, for up to UDP_CLIENT_TIMEUOT micro sec
        FD_ZERO(&readSet);
        FD_SET(socket_info->socketDescriptor, &readSet);
        time_val.tv_sec = 0;
        time_val.tv_usec = UDP_CLIENT_TIMEUOT;
        
        if (select(socket_info->socketDescriptor+1, &readSet, NULL, NULL, &time_val))
        {
            // Read data sent by the TORCS server
            memset(buffer, 0x0, UDP_MSGLEN);  // Zero out the buffer.
            num_read = recv(socket_info->socketDescriptor, buffer, UDP_MSGLEN, 0);
            if (num_read < 0)
            {
                cerr << "didn't get response from server...\n";
            }
            else
            {
                cout << "Received: " << buffer << endl;
                
                if (strcmp(buffer,"***identified***")==0)
				{ 
					//Sleep(200); // Wait for not to crash
					break;
				}
                
            }
        }
    } while(i--);
}


real32_T **accel; 
real32_T **brake;
int32_T  **gear;  
real32_T **steer; 
real32_T **clutch;
int32_T  **focus;
int32_T  **meta; 

real32_T *angle;        
real32_T *curLapTime;   
real32_T *damage;       
real32_T *distFromStart;
real32_T *distRaced;    
real32_T *focus_out;    
real32_T *fuel;         
int32_T  *gear_out;     
real32_T *lastLapTime;  
real32_T *opponents;    
int32_T  *racePos;      
int32_T  *rpm;          
real32_T *speedX;       
real32_T *speedY;       
real32_T *speedZ;       
real32_T *track;        
real32_T *trackPos;     
real32_T *wheelSpinVel; 
real32_T *z;            

void *my_y0; 
void *my_y1; 
void *my_y2; 
void *my_y3; 
void *my_y4; 
void *my_y5; 
void *my_y6; 
void *my_y7;
void *my_y8; 
void *my_y9; 
void *my_y10;
void *my_y11;
void *my_y12;
void *my_y13;
void *my_y14;
void *my_y15;
void *my_y16;
void *my_y17;
void *my_y18;



/* mdlOutputs: Compute outputs */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	int i, retval;
    int num_read;
	int sock_error;
	char status_str[50];
    fd_set readSet;
    struct timeval time_val;
    
    static int timeout_counter = 100;

	// Get PWork array pointers
	void **PWork = ssGetPWork(S);
	char* ip_addr = (char *)PWork[0];
	unsigned short port_num = *((unsigned short*) PWork[1]);
	SocketStruct *socket_info = (SocketStruct *) PWork[2];
	char *buffer = (char * )PWork[3];
	
	
	unsigned long currentStep = 0;
	bool shutdownClient = false;
	
	// Get Inputs (Car controls) from Simulink
	accel  = (real32_T**) ssGetInputPortSignalPtrs(S, 0);
	brake  = (real32_T**) ssGetInputPortSignalPtrs(S, 1);
	gear   = (int32_T**)  ssGetInputPortSignalPtrs(S, 2);
	steer  = (real32_T**) ssGetInputPortSignalPtrs(S, 3);
	clutch = (real32_T**) ssGetInputPortSignalPtrs(S, 4);
	focus  = (int32_T**)  ssGetInputPortSignalPtrs(S, 5);
	meta   = (int32_T**)  ssGetInputPortSignalPtrs(S, 6);
	CarControl *car_control = new CarControl(**accel, **brake, **gear, **steer, **clutch, **focus, **meta);

	// Define Outputs (Car states) to Simulink
	/* my_y0 = ssGetOutputPortSignal(S, 0);
	my_y1 = ssGetOutputPortSignal(S, 1);
	my_y2 = ssGetOutputPortSignal(S, 2);
	my_y3 = ssGetOutputPortSignal(S, 3);
	my_y4 = ssGetOutputPortSignal(S, 4);
	my_y5 = ssGetOutputPortSignal(S, 5);
	my_y6 = ssGetOutputPortSignal(S, 6);
	my_y7 = ssGetOutputPortSignal(S, 7);
	my_y8 = ssGetOutputPortSignal(S, 8);
	my_y9 = ssGetOutputPortSignal(S, 9);
	my_y10 = ssGetOutputPortSignal(S, 10);
	my_y11 = ssGetOutputPortSignal(S, 11);
	my_y12 = ssGetOutputPortSignal(S, 12);
	my_y13 = ssGetOutputPortSignal(S, 13);
	my_y14 = ssGetOutputPortSignal(S, 14);
	my_y15 = ssGetOutputPortSignal(S, 15);
	my_y16 = ssGetOutputPortSignal(S, 16);
	my_y17 = ssGetOutputPortSignal(S, 17);
	my_y18 = ssGetOutputPortSignal(S, 18);

	angle         = (real32_T*) my_y0;
	curLapTime    = (real32_T*) my_y1;
	damage        = (real32_T*) my_y2;
	distFromStart = (real32_T*) my_y3;
	distRaced     = (real32_T*) my_y4;
	focus_out     = (real32_T*) my_y5;
	fuel          = (real32_T*) my_y6;
	gear_out      = (int32_T*)  my_y7;
	lastLapTime   = (real32_T*) my_y8;
	opponents     = (real32_T*) my_y9;
	racePos       = (int32_T*)  my_y10;
	rpm           = (int32_T*)  my_y11;
	speedX        = (real32_T*) my_y12;
	speedY        = (real32_T*) my_y13;
	speedZ        = (real32_T*) my_y14;
	track         = (real32_T*) my_y15;
	trackPos      = (real32_T*) my_y16;
	wheelSpinVel  = (real32_T*) my_y17;
	z             = (real32_T*) my_y18; */

	angle         = (real32_T*) ssGetOutputPortSignal(S, 0);
	curLapTime    = (real32_T*) ssGetOutputPortSignal(S, 1);
	damage        = (real32_T*) ssGetOutputPortSignal(S, 2);
	distFromStart = (real32_T*) ssGetOutputPortSignal(S, 3);
	distRaced     = (real32_T*) ssGetOutputPortSignal(S, 4);
	focus_out     = (real32_T*) ssGetOutputPortSignal(S, 5);
	fuel          = (real32_T*) ssGetOutputPortSignal(S, 6);
	gear_out      = (int32_T*)  ssGetOutputPortSignal(S, 7);
	lastLapTime   = (real32_T*) ssGetOutputPortSignal(S, 8);
	opponents     = (real32_T*) ssGetOutputPortSignal(S, 9);
	racePos       = (int32_T*)  ssGetOutputPortSignal(S, 10);
	rpm           = (int32_T*)  ssGetOutputPortSignal(S, 11);
	speedX        = (real32_T*) ssGetOutputPortSignal(S, 12);
	speedY        = (real32_T*) ssGetOutputPortSignal(S, 13);
	speedZ        = (real32_T*) ssGetOutputPortSignal(S, 14);
	track         = (real32_T*) ssGetOutputPortSignal(S, 15);
	trackPos      = (real32_T*) ssGetOutputPortSignal(S, 16);
	wheelSpinVel  = (real32_T*) ssGetOutputPortSignal(S, 17);
	z             = (real32_T*) ssGetOutputPortSignal(S, 18);



	// Recieve sensor readings and Send control actions over socket
	switch(socket_info->connState)
	{
	case csConnected:
		
        FD_ZERO(&readSet);
        FD_SET(socket_info->socketDescriptor, &readSet);
		time_val.tv_sec = 0;
		time_val.tv_usec = UDP_CLIENT_TIMEUOT;

		if (select(socket_info->socketDescriptor + 1, &readSet, NULL, NULL, &time_val))
		{
			// Read data sent by the TORCS server
			memset(buffer, 0x0, UDP_MSGLEN);  // Zero out the buffer.
			num_read = recv(socket_info->socketDescriptor, buffer, UDP_MSGLEN, 0);
			if (num_read < 0)
			{
				cerr << "didn't get response from server?" << endl;
				return;
			}

			if (strcmp(buffer, "***shutdown***") == 0)
			{
				//d->onShutdown();
				shutdownClient = true;
				cout << "Client Shutdown" << endl;
				ssSetStopRequested(S, 1);;
			}

			if (strcmp(buffer, "***restart***") == 0)
			{
				//d->onRestart();
				cout << "Client Restart" << endl;
				ssSetStopRequested(S, 1);
			}
			CarState *car_state = new CarState(string(buffer));
			*angle = car_state->getAngle();
			*curLapTime = car_state->getCurLapTime();
			*damage = car_state->getDamage();
			*distFromStart = car_state->getDistFromStart();
			*distRaced = car_state->getDistRaced();
			
			for (int i = 0; i<FOCUS_SENSORS_NUM; i++)
				focus_out[i] = car_state->getFocus(i);
			
			*fuel = car_state->getFuel();
			*gear_out = car_state->getGear();
			*lastLapTime = car_state->getLastLapTime();
			
			for (int i = 0; i < OPPONENTS_SENSORS_NUM; i++)
				opponents[i] = car_state->getOpponents(i);
			
			*racePos = car_state->getRacePos();
			*rpm = car_state->getRpm();
			*speedX = car_state->getSpeedX();
			*speedY = car_state->getSpeedY();
			*speedZ = car_state->getSpeedZ();
			
			for (int i = 0; i < TRACK_SENSORS_NUM; i++)
				track[i] = car_state->getTrack(i);
			
			*trackPos = car_state->getTrackPos();
			
			for (int i = 0; i < 4; i++)
				wheelSpinVel[i] = car_state->getWheelSpinVel(i);
			
			*z = car_state->getZ();

			memset(buffer, 0x0, UDP_MSGLEN);
			sprintf(buffer, "%s", car_control->toString().c_str());

			if (sendto(socket_info->socketDescriptor, buffer, strlen(buffer) + 1, 0,
				(const sockaddr *) &(socket_info->sockAddrIn), socket_info->addrLen) < 0)
			{
				ssSetErrorStatus(S, "Error in sending data\n");
			}

		}
		else
		{
            if (timeout_counter-- <= 0)
                ssSetErrorStatus(S, "** Server did not respond in 1 second.\n");

		}
		break;

	case csError:
		sprintf(status_str, "TORCS not connected, Error code: %d\n", WSAGetLastError());
		ssSetErrorStatus(S, status_str);
		break;

	}
	
}

/* mdlTerminate - called when the simulation is terminated ***********************************/
static void mdlTerminate(SimStruct *S) 
{
	DeletePWorkPointers(S);
    WSACleanup();
}

/* Trailer socket_information to set everything up for simulink usage *******************************/
#ifdef  MATLAB_MEX_FILE                      /* Is this file being compiled as a MEX-file?   */
#include "simulink.c"                        /* MEX-file interface mechanism                 */
#else
#include "cg_sfun.h"                         /* Code generation registration function        */
#endif
