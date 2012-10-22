/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
/*
This version is modified for control of the stalker II robot.
Features added:
-	Get coordinates from the selected XnSceletonPosition
-	Translates the coordinates and prepares them for transmission to the robot
-	Sends the directions to the robot for execution (using a simple Serial protocol for robotic communication)
-	Receives and processes feedback data from the robotic platform and displays this in the GUI

11-10-2012 <d.vollmar@fontys.nl>
*/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "SceneDrawer.h"
#include <XnPropNames.h>
#include <atlstr.h>
#include "Serial.h"

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
XnSkeletonJointPosition headPosition;
int oldX;
int oldY;
int oldZ;

int playerId = 1;

int automatic_display_counter = -1;


xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Player g_Player;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;

XnBool g_bPrintFrameID = TRUE;
XnBool g_bMarkJoints = TRUE;

#ifndef USE_GLES
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#else
#include "opengles.h"
#endif

#ifdef USE_GLES
static EGLDisplay display = EGL_NO_DISPLAY;
static EGLSurface surface = EGL_NO_SURFACE;
static EGLContext context = EGL_NO_CONTEXT;
#endif

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

XnBool g_bPause = false;
XnBool g_bRecord = false;

XnBool g_bQuit = false;

void getPosition();
void selectPlayer(int player);

//communication
PSerial comm;

int move0(int speedx,int direction);
int move1(int speedx,int direction);//all wheels share equal angle


//speed and angle control
#define MIN_SPEED 10
#define MAX_SPEED 100

#define MAX_ANGLES 14
#define MIN_ANGLE -7 //minimum degree angle
#define MAX_ANGLE 7 //maximum degree angle


bool manual_control;
int speed;
int angle;
int anglefilter;
int speedfilter;

int remote_movemode;
int LOCK_POINT;//int lock point, help variable for lock point changes
bool mode_Perpendicular;//camera in line with width when false, in line with length when true
//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

void CleanupExit()
{
	g_scriptNode.Release();
	g_DepthGenerator.Release();
	g_UserGenerator.Release();
	g_Player.Release();
	g_Context.Release();

	exit (1);
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d New User %d\n", epochTime, nId);
	// New user found
	if (g_bNeedPose)
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& /*generator*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Lost user %d\n", epochTime, nId);	
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& /*capability*/, const XnChar* strPose, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& /*capability*/, XnUserID nId, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Calibration started for user %d\n", epochTime, nId);
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& /*capability*/, XnUserID nId, XnCalibrationStatus eStatus, void* /*pCookie*/)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		// Calibration succeeded
		printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);		
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		printf("%d Calibration failed for user %d\n", epochTime, nId);
		if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
		{
			printf("Manual abort occured, stop attempting to calibrate!");
			return;
		}
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"

// Save calibration to file
void SaveCalibration()
{
	XnUserID aUserIDs[20] = {0};
	XnUInt16 nUsers = 20;
	g_UserGenerator.GetUsers(aUserIDs, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		// Find a user who is already calibrated
		if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i]))
		{
			// Save user's calibration to file
			g_UserGenerator.GetSkeletonCap().SaveCalibrationDataToFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
			break;
		}
	}
}
// Load calibration from file
void LoadCalibration()
{
	XnUserID aUserIDs[20] = {0};
	XnUInt16 nUsers = 20;
	g_UserGenerator.GetUsers(aUserIDs, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		// Find a user who isn't calibrated or currently in pose
		if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) continue;
		if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUserIDs[i])) continue;

		// Load user's calibration from file
		XnStatus rc = g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
		if (rc == XN_STATUS_OK)
		{
			// Make sure state is coherent
			g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
			g_UserGenerator.GetSkeletonCap().StartTracking(aUserIDs[i]);
		}
		break;
	}
}

// this function is called each frame
void glutDisplay (void)
{
	automatic_display_counter++;
	if(automatic_display_counter == 5)
	{
		if(!manual_control)
		{
			getPosition();
		}
		else
		{
			if(!remote_movemode)
			{
				move0(speed,angle);
			}
			else
				move1(speed,angle);
		}
		automatic_display_counter=-1;
	}

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint

	glMatrixMode(GL_MODELVIEW);
	/*Modes:
	GL_MODELVIEW
	Applies subsequent matrix operations to the modelview matrix stack.

	GL_PROJECTION 'DEFAULT
	Applies subsequent matrix operations to the projection matrix stack.

	GL_TEXTURE
	Applies subsequent matrix operations to the texture matrix stack.

	GL_COLOR
	Applies subsequent matrix operations to the color matrix stack.
	*/
	glPushMatrix();
	glLoadIdentity();
	glScalef (-1.0, 1.0, 1.0);

	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);
#ifndef USE_GLES
	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
#else
	glOrthof(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
#endif

	glDisable(GL_TEXTURE_2D);

	if (!g_bPause)
	{
		// Read next available data
		g_Context.WaitOneUpdateAll(g_UserGenerator);
	}

	// Process the data
	g_DepthGenerator.GetMetaData(depthMD);
	g_UserGenerator.GetUserPixels(0, sceneMD);
	DrawDepthMap(depthMD, sceneMD);

#ifndef USE_GLES
	glutSwapBuffers();
#endif
}

#ifndef USE_GLES
void glutIdle (void)
{
	if (g_bQuit) {
		CleanupExit();
	}

	// Display the frame
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27: //esc key
		CleanupExit();

	case 'w':// w
		if(manual_control)
		{
			if(speed==0)
			{
				speed=MIN_SPEED;
			}
			else
				speed+=4;
			printf("new speed: %i\n",speed);
		}
		break;
	case 'a':// a
		if(manual_control)
		{
			angle-=1;
			if(angle<-7)angle=-7;
		}
		break;
	case 'd':
		if(manual_control)
		{
			angle+=1;
			if(angle>7)angle=7;
		}
		break;
	case 't':
		if(manual_control)
		{
			remote_movemode = !remote_movemode;				
		}
		break;
	case 'm':
		if(manual_control)
		{
			manual_control=FALSE;
			printf("Manual Mode de-activated.\n");
			printf("Keys:\nm - Press m to enable Manual Mode,\nb - draw background,\nx - draw pixels,\ns - draw skeleton,\ni - print label,\nl - toggle print (ID / ID + STATE),\nf - print frameID,\nj - Mark joints,\np - pause,\nr - start device,\ne - stop device,\n` - switch Perpendicular mode,\n1 - select player 1,\nn - selec...      n\n");
		}
		else
		{
			manual_control=TRUE;
			remote_movemode=FALSE;
			printf("Manual Mode activated. Keys:\na - turn left,\nd - turn right,\nw - speed control (forward),\ns - speed control (backward),\nSpace - Stop moving.\nn - Equal wheel angle steering (on/off).\n\nPress m again to disable Manual Mode.\n");
		}
		break;
	case 'b':
		// Draw background?
		g_bDrawBackground = !g_bDrawBackground;
		break;
	case 'x':
		// Draw pixels at all?
		g_bDrawPixels = !g_bDrawPixels;
		break;
	case 's':
		if(manual_control)
		{
			if(speed==0)
			{
				speed=-MIN_SPEED;
			}
			else
				speed-=5;
			printf("new speed: %i\n",speed);
		}
		else// Draw Skeleton?
		{
			g_bDrawSkeleton = !g_bDrawSkeleton;
		}
		break;
	case 'i':
		// Print label?
		g_bPrintID = !g_bPrintID;
		break;
	case 'l':
		// Print ID & state as label, or only ID?
		g_bPrintState = !g_bPrintState;
		break;
	case 'f':
		// Print FrameID
		g_bPrintFrameID = !g_bPrintFrameID;
		break;
	case 'j':
		// Mark joints
		g_bMarkJoints = !g_bMarkJoints;
		break;
	case'p':
		g_bPause = !g_bPause;
		break;
	case'r':
		comm.serial_send("$1");
		speed=0;
		angle=0;
		break;
	case'e':
		comm.serial_send("$0");
		break;
	case 'q':
		mode_Perpendicular=!mode_Perpendicular;
		printf("Set mode_Perpendicular switched\n");
		break;
	case 'S':
		SaveCalibration();
		break;
	case 'L':
		LoadCalibration();
		break;
	case 32: //space key
		if(!remote_movemode)
		{
			move0(0,angle);
		}
		else
			move1(0,angle);
		speed=0;//reset speed variable
		automatic_display_counter=-5;//prevent loop from triggering
		//showPosition();
		break;
	case '1':
		selectPlayer(1);
		break;
	case '2':
		selectPlayer(2);
		break;
	case '3':
		selectPlayer(3);
		break;
	case '4':
		selectPlayer(4);
		break;
	case '5':
		selectPlayer(5);
		break;
	case '6':
		selectPlayer(6);
		break;
	case '7':
		selectPlayer(7);
		break;
	case '8':
		selectPlayer(8);
		break;
	case '9':
		selectPlayer(9);
		break;
	}
}

void selectPlayer(int player){
	playerId = player;
	printf("Selected player %d\n", player);
}

void getPosition(){	
	LOCK_POINT = XN_SKEL_NECK;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(playerId,(XnSkeletonJoint) LOCK_POINT,headPosition);
	int x = headPosition.position.X;
	int y = headPosition.position.Y;
	int z = headPosition.position.Z;

	int confidence = headPosition.fConfidence;

	//printf("speed: %d, angle: %d,confidence %d, x:%i, y:%i, z:%i\n",speed,angle,confidence,x,y,z);

	if(x == oldX && y == oldY && z == oldZ){
		return;//tracked person is probably gone
	}

	oldX = x;
	oldY = y;
	oldZ = z;

	int max_z;
	int max_x;
	int help;

	switch(LOCK_POINT)
	{
	case XN_SKEL_HEAD:
		{
			x+=510;
			max_x=1050;
			z-=1000;
			max_z=3700;
			break;
		}
	case XN_SKEL_NECK:
		{
			x+=1050;
			max_x=1485;
			z-=1000;
			max_z=3391;
			break;
		}
	}
	if(mode_Perpendicular)
	{
		help=x;
		x=z;
		z=help;
		help=max_x;
		max_x=max_z;
		max_z=help;
	}

	if(z<0){
		speed=0;
		speedfilter=0;
	}else if(z>max_z)
	{
		speed=0;//failsafe
	}
	else//if z is bigger than zero and z and smaller than 3700
	{
		if(z<300)
		{
			speed=-15;
		}
		else
			speed=((100.00/(max_z-500)) * (z-300));//automatic cast to integer from float, round
	}
	if(speedfilter!=speed)//there is a speed change!
	{
		if(speed-speedfilter>15)//if delta speed is bigger than maximum allowed
		{//increase in speed
			speedfilter+=15;
			speed=speedfilter;
		}
		else if(speedfilter-speed>30)//if delta speed is bigger than maximum allowed
		{//decrease in speed
			speedfilter-=30;
			speed=speedfilter;
		}
		else //if delta speed is smaller than maximum allowed
			speedfilter=speed;//smaller increase or decrease in speed
	}
	if(x<0)//out of screen, left side, fail-safe value
	{
		angle=MAX_ANGLE;//MAXIMUM
	}
	else
		if(x>max_x)//out of screen, right side, fail-safe value
		{
			angle=MIN_ANGLE;//MINIMUM
		}
		else//if x is larger than zero and smaller than 1050
		{
			//calculate new angle
			angle=(MAX_ANGLE - (((MAX_ANGLE-MIN_ANGLE)/(float)max_x) * x));//automatic cast to integer from float, round
		}

		if(confidence == 0)
		{
			speed=0;
			speedfilter=0;
			//printf("confidence-triggered reset");
			//move0(0,angle);//stop moving
			//return;//not confident enough to act on it, stop moving!
		}
		else
			//printf("speed: %d, angle: %d,confidence %d, x:%i, y:%i, z:%i\n",speed,angle,confidence,x,y,z);

			move0(speed,angle);

}

void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB |  GLUT_MULTISAMPLE);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("User Tracker Viewer");

	comm.Connect( TEXT("COM9"),CBR_115200,8, NOPARITY, ONESTOPBIT);
	Sleep(100);//give time to connect and create window
	printf("Keys:\nm - Press m to enable Manual Mode,\nb - draw background,\nx - draw pixels,\ns - draw skeleton,\ni - print label,\nl - toggle print (ID / ID + STATE),\nf - print frameID,\nj - Mark joints,\np - pause,\nr - start device,\ne - stop device,\n` - switch Perpendicular mode,\n1 - select player 1,\nn - selec...      n\n");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

}
#endif // USE_GLES

#define SAMPLE_XML_PATH "../../../Data/SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
{																\
	printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
	return nRetVal;												\
}

int main(int argc, char **argv)
{
	XnStatus nRetVal = XN_STATUS_OK;


	if (argc > 1)
	{
		nRetVal = g_Context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = g_Context.OpenFileRecording(argv[1], g_Player);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
			return 1;
		}
	}
	else
	{
		xn::EnumerationErrors errors;
		nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
		if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return (nRetVal);
		}
		else if (nRetVal != XN_STATUS_OK)
		{
			printf("Open failed: %s\n", xnGetStatusString(nRetVal));
			return (nRetVal);
		}
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		printf("No depth generator found. Using a default one...");
		xn::MockDepthGenerator mockDepth;
		nRetVal = mockDepth.Create(g_Context);
		CHECK_RC(nRetVal, "Create mock depth");

		// set some defaults
		XnMapOutputMode defaultMode;
		defaultMode.nXRes = 320;
		defaultMode.nYRes = 240;
		defaultMode.nFPS = 48;
		nRetVal = mockDepth.SetMapOutputMode(defaultMode);
		CHECK_RC(nRetVal, "set default mode");

		// set FOV
		XnFieldOfView fov;
		fov.fHFOV = 1.0225999419141749;
		fov.fVFOV = 0.79661567681716894;
		nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
		CHECK_RC(nRetVal, "set FOV");

		XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
		XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

		nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
		CHECK_RC(nRetVal, "set empty depth map");

		g_DepthGenerator = mockDepth;
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	CHECK_RC(nRetVal, "Register to user callbacks");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	CHECK_RC(nRetVal, "Register to calibration start");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
	CHECK_RC(nRetVal, "Register to calibration complete");

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
		CHECK_RC(nRetVal, "Register to Pose Detected");
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);

		nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
		CHECK_RC(nRetVal, "Register to pose in progress");
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
	CHECK_RC(nRetVal, "Register to calibration in progress");

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

#ifndef USE_GLES
	glInit(&argc, argv);
	glutMainLoop();
#else
	if (!opengles_init(GL_WIN_SIZE_X, GL_WIN_SIZE_Y, &display, &surface, &context))
	{
		printf("Error initializing opengles\n");
		CleanupExit();
	}

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	while (!g_bQuit)
	{
		glutDisplay();
		eglSwapBuffers(display, surface);
	}
	opengles_shutdown(display, surface, context);

	CleanupExit();
#endif
}




//COMMUNICATION FUNCTIONS

PSerial::PSerial(){

}
int PSerial::Connect( TCHAR * commport, long baudrate, BYTE bytesize, BYTE parity, BYTE stopbits)
{
	TCHAR * pcCommPort = commport; //  Most systems have a COM1 port

	//  Open a handle to the specified com port.
	hCom = CreateFile( pcCommPort,
		GENERIC_READ | GENERIC_WRITE,
		0,      //  must be opened with exclusive-access
		NULL,   //  default security attributes
		OPEN_EXISTING, //  must use OPEN_EXISTING
		0,      //  not overlapped I/O
		NULL ); //  hTemplate must be NULL for comm devices

	if (hCom == INVALID_HANDLE_VALUE) 
	{
		//  Handle the error.
		printf ("CreateFile failed with error %d.\n", GetLastError());
		return 4;
	}

	//  Initialize the DCB structure.
	SecureZeroMemory(&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(DCB);

	//  Build on the current configuration by first retrieving all current
	//  settings.
	fSuccess = GetCommState(hCom, &dcb);

	if (!fSuccess) 
	{
		//  Handle the error.
		printf ("GetCommState failed with error %d.\n", GetLastError());
		return 3;
	}

	//PrintCommState(dcb);       //  Output to console

	//  Fill in some DCB values and set the com state: 
	//  57,500 bps, 8 data bits, no parity, and 1 stop bit.
	dcb.BaudRate = baudrate;     //  baud rate
	dcb.ByteSize = bytesize;             //  data size, xmit and rcv
	dcb.Parity   = parity;      //  parity bit
	dcb.StopBits = stopbits;    //  stop bit

	fSuccess = SetCommState(hCom, &dcb);

	if (!fSuccess) 
	{
		//  Handle the error.
		printf ("SetCommState failed with error %d.\n", GetLastError());
		return 2;
	}

	//  Get the comm config again.
	fSuccess = GetCommState(hCom, &dcb);

	if (!fSuccess) 
	{
		//  Handle the error.
		printf ("GetCommState failed with error %d.\n", GetLastError());
		return 1;
	}

	//PrintCommState(dcb);       //  Output to console

	_tprintf (TEXT("Serial port %s successfully reconfigured.\n"), pcCommPort);

	return (0);
}

PSerial::~PSerial(){
	CloseHandle(hCom);
}
int PSerial::Close()
{
	return CloseHandle(hCom);
}

int PSerial::serial_send(CString data)
{
	DWORD received;
	for (int i = 0; i < data.GetLength(); ++i)
	{
		//printf("%c",data.GetAt(i));
		WriteFile(PSerial::hCom, (CString) data.GetAt(i),1,&received,NULL);
	}
	WriteFile(PSerial::hCom,"\r",1,&received,NULL);
	return 1;
}

// Reads a single byte from the serial buffer. Returns true if a byte was read.
// the BYTE &b is set with the byte from the serial buffer. If the buffer was empty, the byte will be set to 0.
// todo: checken of het blocked-io is, eventueel in een aparte thread zetten
BOOL PSerial::serialRead(BYTE &b){
	BYTE rx;
	b=0;
	DWORD dwBytesTransferred=0;

	if (ReadFile (hCom, &rx, 1, &dwBytesTransferred, 0)){
		if (dwBytesTransferred == 1){
			b=rx;
			return true;
		}
	}
	return false;
}

char * PSerial::serialRead(){
	char r[64];
	int p = 0;
	byte b = 0;	
	while(serialRead((b))){
		if(p == 64){
			p--;//prevent index out of bounds
			break;
		}
		r[p] = (char) b;		 
		p++;
	}
	//mark eos
	r[p] = '\0';
	//return type char * instead of r[]
	char *ret;
	ret = r;
	return ret;
}



void PrintCommState(DCB dcb)
{
	//  Print some of the DCB structure values
	_tprintf( TEXT("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n"), 
		dcb.BaudRate, 
		dcb.ByteSize, 
		dcb.Parity,
		dcb.StopBits );
}

int move0(int speedx,int direction)
{
	char buffer [50];
	if(speedx>MAX_SPEED)speedx=MAX_SPEED;
	if(speedx<-MAX_SPEED)speedx=-MAX_SPEED;
	if(speedx<MIN_SPEED)//if speed is smaller than minimum speed
	{
		if(speedx>-1 || (speedx>MIN_SPEED && speedx<1))//but is bigger than -1, so 0+ OR (reversed)
		{
			speedx=0;
		}
	}
	sprintf_s(buffer, "$2,%i,%i\n",speedx,direction);
	//printf("%s",buffer);
	comm.serial_send(buffer);
	return 1;
}
int move1(int speedx,int direction)
{
	char buffer [50];
	if(speedx>100)speedx=100;
	if(speedx<-100)speedx=-100;
	if(speedx<MIN_SPEED && speed>0)
	{
		speedx=MIN_SPEED;//minimum speed
	}
	else
		if(speedx>-MIN_SPEED && speed<0)
			speedx=-MIN_SPEED;//minimum speed
	sprintf_s(buffer, "$3,%i,%i\n",speedx,direction);
	//printf("%s",buffer);
	comm.serial_send(buffer);
	return 1;
}