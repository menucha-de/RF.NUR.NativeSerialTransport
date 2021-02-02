/*
  Copyright © 2014-2016 Nordic ID
  NORDIC ID DEMO SOFTWARE DISCLAIMER

  You are about to use Nordic ID Demo Software ("Software").
  It is explicitly stated that Nordic ID does not give any kind of warranties,
  expressed or implied, for this Software. Software is provided "as is" and with
  all faults. Under no circumstances is Nordic ID liable for any direct, special,
  incidental or indirect damages or for any economic consequential damages to you
  or to any third party.

  The use of this software indicates your complete and unconditional understanding
  of the terms of this disclaimer.

  IF YOU DO NOT AGREE OF THE TERMS OF THIS DISCLAIMER, DO NOT USE THE SOFTWARE.
*/

#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"

#include "jni.h"
#include <stdio.h>
#include "com_nordicid_nativeserial_NativeSerialTransport.h"
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/time.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/* From Win sources, define some helpers. */
#define GetLastError() errno
#define SetLastError(a) errno = (a)
#define _countof(x) (sizeof(x)/sizeof((x)[0]))
#define Sleep(x) usleep((x)*1000)

#define IO_EXCEPTION "java/io/IOException"
#define NURAPI_EXCEPTION "com/nordicid/nurapi/NurApiException"
#define INVALID_HANDLE_VALUE -1
#define DWORD unsigned int
#define TRUE JNI_TRUE
#define FALSE JNI_FALSE
#define BOOL bool
#define BYTE int8_t

#define MAX_PORT_NAMELEN	256

/* Built-in string class. */
#define JAVA_STRING_CLASS	"Ljava/lang/String;"

/* Built-in I/O exception definition. */
#define IO_EXCEPTION "java/io/IOException"
/* NUR Java API exception. */
#define NURAPI_EXCEPTION "com/nordicid/nurapi/NurApiException"

/* The provided SerialPort class. */
#define SERPORT_CLASSNAME	"Lcom/nordicid/nativeserial/SerialPort;"
/* The SerialPort clas constructs's definition. */
#define SERPORT_INITPARAMS	"(Ljava/lang/String;Ljava/lang/String;I)V"

/* Enumeration(Ex) function. */
#define __ENUMFUNCEX__	"enumaratePortsNativeEx"

/*
	Throw an exception that can be handled as "NurApiException" in the Java application
*/
void ThrowNurApiException(JNIEnv *env, const char *name, const char *errorMessage);

// !!! This may, or may not, be required with some USB serial implementations.
#define ZLP_64_FIX	1

/*
	Serial port transport structure.
*/
struct SP_TRANSPORT
{
	char portName[MAX_PORT_NAMELEN];
	int baudrate;
	int fd;
	int pipeFd[2];
	BOOL connected;
	BOOL connecting;
};

/*
	"Windows-like" serial port definition.
*/
struct SP_INFO
{
	char openName[MAX_PORT_NAMELEN];
	char friendlyName[MAX_PORT_NAMELEN];
	int portNumber;
	struct SP_INFO *next;
};

/*
	A list defining serial port entries.
*/
struct SP_LIST
{	
	int count;
	struct SP_INFO *info;	
};

/*
	Create and initialize simple serial port list.
*/
static struct SP_LIST *createPortList()
{
	struct SP_LIST *list = NULL;

	list = (struct SP_LIST *)malloc(sizeof(struct SP_LIST));
	list->count = 0;
	list->info = NULL;

	return list;
}

/*
	Find the last entry in the given serial port list.
*/
static struct SP_INFO *findLastInfo(struct SP_LIST *pList)
{
	struct SP_INFO *pi = pList->info;
	while (pi && pi->next)
	{
		pi = pi->next;
	};

	return pi;
}

/*
	Append a new serial port entry to the given list.
*/
static struct SP_INFO *addNewSpInfo(struct SP_LIST *dest)
{
	struct SP_INFO *p;
	if (dest != NULL)
	{
		if (dest->info == NULL)
		{
			dest->info = (struct SP_INFO *)malloc(sizeof(struct SP_INFO));
			dest->info->next = NULL;
			p = dest->info;
		}
		else
		{
			p = findLastInfo(dest);
			p->next = (struct SP_INFO *)malloc(sizeof(struct SP_INFO));
			p = p->next;
		}
		memset((void *)p, 0, sizeof(struct SP_INFO));
		dest->count++;		
		return p;
	}

	return NULL;
}

/*
	Allocate a new serial port transport.
*/
static struct SP_TRANSPORT *newSerialTransport(void)
{
	struct SP_TRANSPORT *trptr = (struct SP_TRANSPORT *)malloc(sizeof(struct SP_TRANSPORT));

	memset((void *)trptr, 0, sizeof(struct SP_TRANSPORT));
	trptr->baudrate = 115200;
	trptr->fd = INVALID_HANDLE_VALUE;
	
	return trptr;
}

/*
	Free a previously created serial port list.
*/
static int freeSerialPortList(struct SP_LIST *pList)
{
	int rc = 0;
	struct SP_INFO *pInfo;
	struct SP_INFO *pNext;

	if (pList != NULL) {		
		pInfo = pList->info;
		
		while (pInfo) {						
			pNext = pInfo->next;
			free((void *)pInfo);
			pInfo = pNext;
			rc++;
		}		

		free((void *)pList);
	}

	return rc;
}

/*
	Linux equivalent.
*/
DWORD GetTickCount()
{
	struct timeval tv;
	if(gettimeofday(&tv, NULL) != 0)
		return 0;
	return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/*
	Translate a given baudrate to the system's corresponding one.
*/
static int TranslateBaudrate(DWORD baudrate)
{
	int baudr = 0;

	switch (baudrate)
	{
	case 50:
		baudr = B50;
		break;
	case 75:
		baudr = B75;
		break;
	case 110:
		baudr = B110;
		break;
	case 134:
		baudr = B134;
		break;
	case 150:
		baudr = B150;
		break;
	case 200:
		baudr = B200;
		break;
	case 300:
		baudr = B300;
		break;
	case 600:
		baudr = B600;
		break;
	case 1200:
		baudr = B1200;
		break;
	case 1800:
		baudr = B1800;
		break;
	case 2400:
		baudr = B2400;
		break;
	case 4800:
		baudr = B4800;
		break;
	case 9600:
		baudr = B9600;
		break;
	case 19200:
		baudr = B19200;
		break;
	case 38400:
		baudr = B38400;
		break;
	case 57600:
		baudr = B57600;
		break;
	case 115200:
		baudr = B115200;
		break;
	case 230400:
		baudr = B230400;
		break;
	case 460800:
		baudr = B460800;
		break;
	case 500000:
		baudr = B500000;
		break;
	case 576000:
		baudr = B576000;
		break;
	case 921600:
		baudr = B921600;
		break;
	case 1000000:
		baudr = B1000000;
		break;
	default:
		break;
	}

	return baudr;
}

BOOL SetBaudrate(int fd, int baudrate)
{
	struct termios portSettings;
	int br;
	int error;

	br = TranslateBaudrate(baudrate);
	if (br == 0) {
		return FALSE;
	}
	
	memset(&portSettings, 0, sizeof(struct termios));
	
	portSettings.c_cflag = CS8 | CLOCAL | CREAD;
	portSettings.c_iflag = 0;//IGNPAR;
	portSettings.c_oflag = 0;
	portSettings.c_lflag = 0;
	portSettings.c_cc[VMIN] = 0;  /* block until n bytes are received */
	portSettings.c_cc[VTIME] = 0; /* block until a timer expires (n * 100 mSec.) */

	cfsetispeed(&portSettings, br);
	cfsetospeed(&portSettings, br);

	error = tcsetattr(fd, TCSANOW, &portSettings);
	if(error == -1)
	{
		return FALSE;
	}

    return TRUE;
}

#define max(a, b) (((a) > (b)) ? (a) : (b))
/*
	Wait for incoming traffic specified by the given transport spec.
*/
static BOOL WaitFD(struct SP_TRANSPORT *tr, int timeout)
{
	struct timeval tv;
	struct timeval *tvPtr = &tv;
	int ret = 0;
	fd_set set;
	FD_ZERO(&set);
	FD_SET(tr->fd, &set);
	FD_SET(tr->pipeFd[0], &set);

	if (timeout == -1) {
		tvPtr = NULL;
	} 
	else {
		tv.tv_sec = timeout/1000;
		tv.tv_usec = timeout*1000;
	}

	ret = select(1 + max(tr->fd, tr->pipeFd[0]), &set, NULL, NULL, tvPtr);

	return (ret > 0) ? TRUE : FALSE;
}

/*
	Disconnect a valid transport spec.
*/
BOOL Disconnect(struct SP_TRANSPORT *tr)
{
	if(tr == NULL || tr->connected == false) {
		return FALSE;
	}
	else
	{	
		tr->connected = FALSE;
		tr->connecting = FALSE;

		if (tr->pipeFd[1] != INVALID_HANDLE_VALUE) {
			tcflush(tr->fd, TCIOFLUSH);
			close(tr->pipeFd[0]);
			close(tr->pipeFd[1]);
			tr->pipeFd[0] = INVALID_HANDLE_VALUE;
			tr->pipeFd[1] = INVALID_HANDLE_VALUE;
		}

		if (tr->fd != INVALID_HANDLE_VALUE) {
			close(tr->fd);
			tr->fd = INVALID_HANDLE_VALUE;
		}
		
		free(tr);
	}
	
	return TRUE;
}

/*
	Build a serial port list.
	Used by the serial port enumeration functions.
	There is probably a better way to do this but this works.
*/

static struct SP_LIST *buildSerialPortList()
{
	/* Extended serial port information. */
	struct SP_LIST *ret = NULL;
	/* Single serial port information. */
	struct SP_INFO *spDest;

	int i;
	/* Single device's open name. */
	char path[256];
	/* Descriptior for a file. */
	int fd;

	/* Create a base list. Needs to freed later. */
	ret = createPortList();
	for(i = 0; i < 64; i++) {
		/* More: "S", "USB" etc.? */
		sprintf(path, "/dev/ttyACM%d", i);

		fd = open(path, O_RDWR | O_NONBLOCK);

		if(fd > 0) {
			spDest = addNewSpInfo(ret);
			sprintf(spDest->friendlyName, "USB serial port %d", i);
			spDest->portNumber = i;
			strcpy(spDest->openName, path);
		}
		close(fd);
	}

	return ret;
}

/*
	A serial port enumeration function that can be called from the Java application through the JNI.
	Returns simply an array of port open names.
*/
JNIEXPORT jobjectArray JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_enumeratePortsNative(JNIEnv *env, jclass clazz)
{
	jobjectArray ret = NULL;
	jclass strClass = (*env)->FindClass(env, JAVA_STRING_CLASS);
	jstring str = NULL;
	
	struct SP_LIST *ptrPorts = NULL;
	struct SP_INFO *aPort;
	int i;

	ptrPorts = buildSerialPortList();
	
	if (ptrPorts != NULL && ptrPorts->count > 0) {
		ret = (*env)->NewObjectArray(env, ptrPorts->count, strClass, NULL);

		if (ret == NULL) {
			(*env)->DeleteLocalRef(env, strClass);
			return NULL;
		}
	
		aPort = ptrPorts->info;
		for(i = 0; i < ptrPorts->count; i++) {
			str = (*env)->NewStringUTF(env, aPort->openName);
			(*env)->SetObjectArrayElement(env, ret, i, str);
			(*env)->DeleteLocalRef(env, str);
			aPort = aPort->next;
		}
		(*env)->DeleteLocalRef(env, strClass);
		freeSerialPortList(ptrPorts);
	}

	return ret;
}

/*
	A serial port enumeration function that can be called from the Java application through the JNI.
	Returns extended serial port information w/ friendy name (as "friendly" as they can get in this environment).
*/
JNIEXPORT jobjectArray JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_enumaratePortsNativeEx(JNIEnv *env, jclass cls)
{		
	/* Returned port stuff. */
	jobjectArray ret = 0;
	/* Single SerialPort Java object/class */
	jobject portObj;
	jclass thePortClass;
	/* For constructor call. */
	jstring strFriendly;
	jstring strOpen;
	jint portNumber;
	/* CSerialPort constructor's method ID. */
	jmethodID initMethod;
	
	char tmpStr[MAX_PORT_NAMELEN];
	/* The list. */
	struct SP_LIST *theList;
	struct SP_INFO *pInfo;
	int i;

	theList = buildSerialPortList();
	/* Process if there were some interesting ports. */
	if (theList->count > 0) {
		pInfo = theList->info;

		/* Get the Class of interest, SerialPort. */
		thePortClass = (*env)->FindClass(env, SERPORT_CLASSNAME);
		/* What we return. */
		ret = (*env)->NewObjectArray(env, theList->count, thePortClass, NULL);
		/* Constructor initialization method  */
		initMethod = (*env)->GetMethodID(env, thePortClass, "<init>", SERPORT_INITPARAMS);

		/* Classes */
		strFriendly = (*env)->FindClass(env, JAVA_STRING_CLASS);
		strOpen = (*env)->FindClass(env, JAVA_STRING_CLASS);		

		for (i=0;i<theList->count && pInfo;i++) {
			sprintf(tmpStr, "%s", pInfo->friendlyName);
			strFriendly =  (*env)->NewStringUTF(env, tmpStr);			

			sprintf(tmpStr, "%s", pInfo->openName);
			strOpen = (*env)->NewStringUTF(env, tmpStr);		
			
			portNumber = (jint)pInfo->portNumber;
			
			/* Call the newly created object constructor. */
			portObj = (*env)->NewObject(env, thePortClass, initMethod, strOpen, strFriendly, portNumber);

			/* Add the newly created SerialPort object to the array.*/
			(*env)->SetObjectArrayElement(env, ret, i, portObj);
			/* Free local references. */
			(*env)->DeleteLocalRef(env, strFriendly);
			(*env)->DeleteLocalRef(env, strOpen);
			(*env)->DeleteLocalRef(env, portObj);

			/* Get next.*/
			pInfo = pInfo->next;
		}
	}
	else {
		//printf("Throwing exception; no ports.\n");
		ThrowNurApiException(env, NURAPI_EXCEPTION, __ENUMFUNCEX__": no ports to be listed.");
		freeSerialPortList(theList);
		return 0;
	}

	freeSerialPortList(theList);
	return ret;
}

/*
	Connect to the given serial port using the given baudrate.
	Return "handle" to the serial transport that is used with other calls (R/W, disconnect etc.).
*/
JNIEXPORT jlong JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_connectNative(JNIEnv *env, jobject obj, jstring portName, jint baudrate)
{
	struct SP_TRANSPORT *tr;	
	int error;	
	const jbyte *str;

    tr = newSerialTransport();
    
    /*
    	The parameter is the "openName".
    */
    str = (jbyte*)((*env)->GetStringUTFChars(env, portName, NULL));
	strcpy(tr->portName, (const char*)str);
	(*env)->ReleaseStringUTFChars(env, portName, (const char *)str);
	
	tr->baudrate = baudrate;
	tr->connecting = TRUE;	
	tr->fd = open(tr->portName, O_RDWR | O_NOCTTY | O_NDELAY);
	
	if(tr->fd == INVALID_HANDLE_VALUE) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Port in use.");
		Disconnect(tr);
		return 0;
	}
	
	if (!SetBaudrate(tr->fd, tr->baudrate)) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Transport error.");
		Disconnect(tr);
		return 0;
	}

	error = pipe(tr->pipeFd);
	
	if(error != 0) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Connection failure");
		Disconnect(tr);
		return 0;
	}

	/* All is well. */
	tcflush(tr->fd, TCIOFLUSH);

	tr->connecting = FALSE;
	tr->connected = TRUE;

	return (jlong)tr;
}

JNIEXPORT jint JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_getBaudrateNative(JNIEnv *env, jobject obj, jlong hTransport)
{
	struct SP_TRANSPORT *tr = (struct SP_TRANSPORT *)hTransport;
	if(!tr->connected) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Transport not connected.");
		return -1;	
	}
	
	return tr->baudrate;
}

JNIEXPORT void JNICALL Java_com_nordicid_NurApiSerialTransportNative_setBaudrate(JNIEnv *env, jobject obj, jint hTransport, jint baudrate)
{
	struct SP_TRANSPORT *tr = (struct SP_TRANSPORT *)hTransport;
	if(!tr->connected) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Transport not connected.");
		return;	
	}

	if(!SetBaudrate(tr->fd, baudrate)) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Failed to set baudrate.");
		return;
	}

	tr->baudrate = baudrate;

    return;
}

JNIEXPORT void JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_disconnectNative(JNIEnv *env, jobject obj, jlong hTransport)
{
	struct SP_TRANSPORT *tr = (struct SP_TRANSPORT *)hTransport;
	
	/* Would an exception be necessary? */
	/*
	if (tr == NULL || !tr->connected) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Transport not connected.");
		return;
	} */
	Disconnect(tr);
}

/*
	The NurApiTransport eventually calls this in order to fill its internal ring buffer.
*/
JNIEXPORT jint JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_readDataNative(JNIEnv *env, jobject obj, jlong hTransport, jbyteArray buffer, jint bufferLen)
{
	struct SP_TRANSPORT *tr = (struct SP_TRANSPORT *)hTransport;
	if(!tr->connected) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Transport not connected.");
		return -1;
	}
	
	jbyte *buf;
	DWORD iRead = 0;
	
	buf = (*env)->GetByteArrayElements(env, buffer, NULL);
	
	/* Timeout? */
	if (!WaitFD(tr, 10000)) {
		(*env)->ReleaseByteArrayElements(env, buffer, buf, 0);
		return 0;
	}
	
	if(!tr->connected) {
		/* Huh. */
		(*env)->ReleaseByteArrayElements(env, buffer, buf, 0);
		return -1;
	}
	
	iRead = read(tr->fd, buf, bufferLen);
	
	if (iRead == -1) {
		char errorMessage[256];
		sprintf(errorMessage, "Transport error. Error : %d.", GetLastError());
		ThrowNurApiException(env, NURAPI_EXCEPTION, errorMessage);
		(*env)->ReleaseByteArrayElements(env, buffer, buf, 0);
		return -1;
	}
	
	if (iRead == 0) {
		Disconnect(tr);
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Transport not connected.");
		(*env)->ReleaseByteArrayElements(env, buffer, buf, 0);
		return -1;
	}

	/* All is well. */
	(*env)->ReleaseByteArrayElements(env, buffer, buf, 0);
    return iRead;
}
/*
	The NurApiTransport eventually calls this in order to transmit its data.
*/
JNIEXPORT jint JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_writeDataNative(JNIEnv *env, jobject obj, jlong hTransport, jbyteArray buffer, jint offset, jint bufferLen)
{
	struct SP_TRANSPORT *tr = (struct SP_TRANSPORT *)hTransport;
	
	if(!tr->connected) {
		ThrowNurApiException(env, NURAPI_EXCEPTION, "Transport not connected.");
		return -1;
	}
	
	DWORD iWritten = 0;
	DWORD totalWritten = 0;
	DWORD timeout;
	jbyte *buf;

	buf = (*env)->GetByteArrayElements(env, buffer, 0);

	timeout = GetTickCount() + 500;

	while (totalWritten < bufferLen && tr->connected) {
		iWritten = write(tr->fd, buf + totalWritten, bufferLen - totalWritten);
		
		if (iWritten == -1) {
			break;
		}

		totalWritten += iWritten;

		if (iWritten == 0 || timeout < GetTickCount()) {
			(*env)->ReleaseByteArrayElements(env, buffer, buf, 0);
			return -1;
		}
	}

#ifdef ZLP_64_FIX
	if (bufferLen >0 && ((bufferLen % 64) == 0))
	{
		jbyte zlp = 0xFF;	// Module will ignore this.
		write(tr->fd, &zlp, 1);
	}
#endif

	(*env)->ReleaseByteArrayElements(env, buffer, buf, 0);
    return totalWritten;
}

/*

*/
JNIEXPORT jboolean JNICALL Java_com_nordicid_nativeserial_NativeSerialTransport_isConnectedNative(JNIEnv *env, jobject obj, jlong hTransport)
{
	struct SP_TRANSPORT *tr = (struct SP_TRANSPORT *)hTransport;
    if (tr != NULL)
    	return tr->connected;
    return JNI_FALSE;
}

/*
	NurApiException helper.
*/
void ThrowNurApiException(JNIEnv *env, const char *name, const char *errorMessage)
{
	jclass class = (*env)->FindClass(env, name);
	if (class != NULL)
		(*env)->ThrowNew(env, class, errorMessage);
	
	(*env)->DeleteLocalRef(env, class);
}
