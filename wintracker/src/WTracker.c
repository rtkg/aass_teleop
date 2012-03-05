#include <usb.h> /* libusb header */
#include <unistd.h> /* for geteuid */
#include <stdio.h>
#include <string.h>
#include "WTracker.h"

static const int vendorID=0x04b4;
static const int productID=0x64df;
static const int configuration=1; /*  Configuration 1*/
static const int interface=0;	/* Interface 0 */

static const int endpoint_in=1;
static const int endpoint_out=2;

static const int timeout=20000; /* timeout in ms */

/* max packet size is 64-bytes */
static const int reqLen=32;
typedef unsigned char byte;

void bad(const char *why) {
	fprintf(stderr,"Fatal error> %s\n",why);

	exit(17);
}

/****************** Internal I/O Commands *****************/

/** Send this binary string command. */
static void send_usb(struct usb_dev_handle * d, int len, const char * src)
{
   int r = usb_bulk_write(d, endpoint_out, (char *)src, len, timeout);
//   if( r != reqLen )
   if( r < 0 )
   {
	  perror("FS USB write"); bad("USB write failed"); 
   }
}

/** Read this many bytes from this device */
static void recv_usb(struct usb_dev_handle * d, int len, byte * dest)
{
//   int i;
   int r = usb_bulk_read(d, endpoint_in, (char*) dest, len, timeout);
   if( r != len )
   {
	  perror("FS USB read"); bad("USB read failed"); 
   }
}



/* debugging: enable debugging error messages in libusb */
//extern int usb_debug;

/* Find the first USB device with this vendor and product.
   Exits on errors, like if the device couldn't be found.
*/
struct usb_dev_handle *WG_fs_usb_open(void)
{
  struct usb_device * device;
  struct usb_bus * bus;

#ifndef WIN32  
  /**  it is not necessary to run it as sudo as long as the device is
   *   added to udev devices.
   */
  //  if( geteuid()!=0 )
  //	 bad("This program must be run as root, or made setuid root");
#endif  
#ifdef USB_DEBUG
  usb_debug=4; 
#endif

  printf("Locating WinTracker (vendor 0x%04x/product 0x%04x)\n", vendorID, productID);
  /* (libusb setup code stolen from John Fremlin's cool "usb-robot") */

  // added the two debug lines  
  usb_set_debug(255);
  printf("setting USB debug on by adding usb_set_debug(255) \n");
  //End of added codes

  usb_init();
  usb_find_busses();
  usb_find_devices();

/* change by Xiaofan */  
/* libusb-win32: not using global variable like usb_busses*/
/*  for (bus=usb_busses;bus!=NULL;bus=bus->next) */  
  for (bus=usb_get_busses();bus!=NULL;bus=bus->next) 
  {
	 struct usb_device * usb_devices = bus->devices;
	 for( device=usb_devices; device!=NULL; device=device->next )
	 {
		if( device->descriptor.idVendor == vendorID &&
			device->descriptor.idProduct == productID )
		{
		   struct usb_dev_handle *d;
		   printf("Found WinTracker as device '%s' on USB bus %s\n",
				   device->filename,
				   device->bus->dirname);
		   d = usb_open(device);
		   if( d )
		   { /* This is our device-- claim it */
//			  byte retData[reqLen];
			  if( usb_set_configuration(d, configuration) ) 
			  {
				 bad("Error setting USB configuration.\n");
			  }
			  if( usb_claim_interface(d, interface) ) 
			  {
				 bad("Claim failed-- the WinTracker is in use by another driver.\n");
			  }
			  printf("Communication established.\n");
//			  RdVersion(d);
			  return d;
		   }
		   else 
			  bad("Open failed for USB device");
		}
		/* else some other vendor's device-- keep looking... */
	 }
  }
  bad("Could not find WinTracker--\n"
      "you might try lsusb to see if it's actually there.");
  return NULL;
}

#ifdef STANDALONE
int main(int argc, char ** argv) 
{
   struct usb_dev_handle * WG_fs_usb = WG_fs_usb_open();

   byte receive_buf[reqLen];
   byte send_buf[reqLen];
   int intA, i;

   send_buf[0] = 'S';
   send_buf[1] = 'C';
   send_buf[2] = '\0';
   send_usb(WG_fs_usb, 32, send_buf);

   for(;;)
        {
	  /*usb_bulk_read(WG_fs_usb, 32, (char*)receive_buf, 32, 50);*/ /* wait up to 50ms for the data */
	  recv_usb(WG_fs_usb, 32, receive_buf);
	  
	  if( receive_buf[0] == 'D' )
	    {
	      printf("\nsensor %c:\n", receive_buf[1]);
	      
	      intA = receive_buf[3];
	      intA = intA * 256 + receive_buf[2];
	      printf("Sensor0  = %5d, ", intA);
	      intA = receive_buf[5];
	      intA = intA * 256 + receive_buf[4];
	      printf("Sensor1  = %5d, ", intA);
			intA = receive_buf[7];
			intA = intA * 256 + receive_buf[6];
			printf("Sensor2  = %5d, ", intA);
			intA = receive_buf[9];
			intA = intA * 256 + receive_buf[8];
			printf("Sensor3  = %5d\n", intA);

			intA = receive_buf[11];
			intA = intA * 256 + receive_buf[10];
			printf("Sensor4  = %5d, ", intA);
			intA = receive_buf[13];
			intA = intA * 256 + receive_buf[12];
			printf("Sensor5  = %5d, ", intA);
			intA = receive_buf[15];
			intA = intA * 256 + receive_buf[14];
			printf("Sensor6  = %5d, ", intA);
			intA = receive_buf[17];
			intA = intA * 256 + receive_buf[16];
			printf("Sensor7  = %5d\n", intA);


			intA = receive_buf[19];
			intA = intA * 256 + receive_buf[18];
			printf("Sensor8  = %5d, ", intA);
			intA = receive_buf[21];
			intA = intA * 256 + receive_buf[20];
			printf("Sensor9  = %5d, ", intA);
			intA = receive_buf[23];
			intA = intA * 256 + receive_buf[22];
			printf("Sensor10 = %5d,\n", intA);  
                    }
          }

//	usb_release_interface(WG_fs_usb, interface);
   usb_close(WG_fs_usb);
   return 0;
}
#endif

struct usb_dev_handle * WG_fs_usb;
WTrackerSensor wtrackerSensors[N_SENSORS];

void enable_cont_mode()
{
  /* Enable continous output mode */
  char send_buf[reqLen];
  send_buf[0] = 'S';
  send_buf[1] = 'C';
  send_buf[2] = '\0';
  send_usb(WG_fs_usb, 3, send_buf);
  usleep(5000); //rkg - not sure if this is acutally needed
}
void disable_cont_mode()
{
  /* Disable continous output mode (this has to be done when system parameters are changed)*/
  char send_buf[reqLen];
  send_buf[0] = 'S';
  send_buf[1] = 'c';
  send_buf[2] = '\0';
  send_usb(WG_fs_usb, 3, send_buf);
  usleep(5000);//rkg - not sure if this is acutally needed
}


int initialize_wtracker() {
  WG_fs_usb = WG_fs_usb_open();
  
  char send_buf[reqLen];
 
  /* Enable only first receiver */
  send_buf[0] = 'S';
  send_buf[1] = 'A';
  send_buf[2] = '1';
  send_buf[3] = '0';
  send_buf[4] = '0';
  send_buf[5] = '\0';
  send_usb(WG_fs_usb, 6, (char*) send_buf);
  usleep(5000);

  return 0;
}

void shutdown_wtracker() {
  /* Shutting down wtracker */
  
  printf("Shutting down wtracker\n");
  disable_cont_mode();
  usb_release_interface(WG_fs_usb, interface);
  usb_close(WG_fs_usb);  
}

short wtracker2short(char *buf) {
  short v;
  /* TODO - this does not work on smallendian systems */
  ((char*)&v)[0]=buf[0];
  ((char*)&v)[1]=buf[1];
  return v;
}

void setFrontHemisphere() {
  char send_buf[reqLen];
  send_buf[0] = 'S';
  send_buf[1] = 'H';
  send_buf[2] = 'F';
  send_buf[3] = '\0';
  send_usb(WG_fs_usb, 4, (char*) send_buf);
  usleep(5000);
}
void setUpHemisphere() {
  char send_buf[reqLen];
  send_buf[0] = 'S';
  send_buf[1] = 'H';
  send_buf[2] = 'U';
  send_buf[3] = '\0';
  send_usb(WG_fs_usb, 4, (char*) send_buf);
  usleep(5000);
}


void tick_wtracker() {
  char receive_buf[40], *sensorData;
  char send_buf[reqLen];
  //  int intA;
  int sensor;
  int ret,i;

  /* Ask for sensor data */
  send_buf[0] = 'S';
  send_buf[1] = 'P';
  send_buf[2] = '\0';
  send_usb(WG_fs_usb, 3, send_buf);

  for(i=0;i<32;i++) receive_buf[i]=0;
  ret=usb_bulk_read(WG_fs_usb, endpoint_in, (char*)&receive_buf[0], 32, 50); /* wait up to 50ms for the data */
  /*printf("%d bytes: ",ret);
  for(i=0;i<32;i++) printf("%02x ",receive_buf[i]);
  printf("\n");*/
  if(ret != 32) return;

  if(receive_buf[0] == 'D') sensorData = &receive_buf[0];
  else if(receive_buf[1] == 'D') sensorData = &receive_buf[1];
  else return;

  sensor = sensorData[1] - '0';
  /*printf("Got sensor %d ('%c')\n",sensor,sensorData[1]);*/
  

  if(sensor >= 0 && sensor < N_SENSORS) {
    
    /*if(sensor == 0) {
      printf("Received buf: "); for(i=0;i<32;i++) printf("%02x ",receive_buf[i]);
      printf("\n");
      }*/

    wtrackerSensors[sensor].x = wtracker2short(&sensorData[2]) * 0.0001;
    wtrackerSensors[sensor].y = wtracker2short(&sensorData[4]) * 0.0001;
    wtrackerSensors[sensor].z = wtracker2short(&sensorData[6]) * 0.0001;

    wtrackerSensors[sensor].a = wtracker2short(&sensorData[8]) * 0.01;
    wtrackerSensors[sensor].e = wtracker2short(&sensorData[10]) * 0.01;
    wtrackerSensors[sensor].r = wtracker2short(&sensorData[12]) * 0.01;

    wtrackerSensors[sensor].qw = wtracker2short(&sensorData[14]) * 0.01;
    wtrackerSensors[sensor].qx = wtracker2short(&sensorData[16]) * 0.01;
    wtrackerSensors[sensor].qy = wtracker2short(&sensorData[18]) * 0.01;
    wtrackerSensors[sensor].qz = wtracker2short(&sensorData[20]) * 0.01;

    wtrackerSensors[sensor].button = wtracker2short(&sensorData[20]);

      /*printf("sensors[%d].xyz = %3.3f %3.3f %3.3f\n",sensor,wtrackerSensors[sensor].x,wtrackerSensors[sensor].y,wtrackerSensors[sensor].z);*/

    } else {
      printf("Received invalid sensor: %d\n",sensor);
    }
}


