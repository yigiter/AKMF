/*
 * main.c
 *
 *  Created on: Nov 21, 2010
 *      Author: Yigiter YUKSEL (instk.org)
 */

#include <fcntl.h>
#include <linux/ioctl.h>
#include <pthread.h>


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <time.h>

#include "akm8973.h"
#include "bma150.h"

#include <math.h>

#define GRAV 	(9.81)
#define CHSIZE 	(10)
#define ETHRESH (0.08)	//Singularity threshold for euler
#define MTHRESH	(0.01)	//Singularity threshold for matrix inverse
#define ATHRESH	(0.1)	//Acc threshold for update cache
#define CACTHR	(0.05)
#define PI 		(3.14)
//#define DBG		1

typedef struct
{
	int *p_fakm;
	char *p_awake;
} thread_arg_t;

void *SuspendMonitor(void *arg);
void getcalib(float akmcalib[][4], float bmacalib[][4], char *dacoffset, char *fl_calib);
void init_sensors(int fakm, int fabm, char *dacoffset);
int comp_dacoffset(int fakm, char *offset, char *fl_awake);
int read_sensor_data(int fakm, int fbma, char *akmdat, short *bmadat, short *delay, char *fl_awake);
void calib_tem_data(char *akmdat, float *temp);
void calib_acc_data(short *bmadat, float bmacalib[][4], float *acc);
void calib_mag_data(char *akmdat, float akmcalib[][4], float *mfield);
void update_akmcalib(char *akmdat, float acc[3], float akmcalib[][4]);
void align_pl(float *acc, float dcm[][3]);
void *SliderDetectThread(void *arg) ;
void comp_angles(float *euler, float *mfield, float dcm_pb[][3]);
int fit_circle(float cache[][CHSIZE], float *cparam);
int comp_3x3inv(float A[][3], float Ainv[][3]);
void publish_data(int fakm, float *mfield, float *acc, float *euler, float temp);
void write_calib(float akmcalib[][4], float bmacalib[][4], char *dacoffset, char *fl_calib);


int main() {
	float akmcalib[3][4];	//magnetometer calibration matrix
	float bmacalib[3][4];	//accelerometer calibration matrix
	char dacoffset[3];		//magnetometers DAC offset
	int fakm;	//The file that the daemon talks to
	int fbma;	//accel file
	char fl_awake=0;	//sleep flag
	char fl_calib[3]={0,0,0};	//Calib computation flags
	short delay;
	char akmdat[RBUFF_SIZE+1];	//akm data
	short bmadat[7];		//accel data
	float dcm_pb[3][3];		//Platform the body dcm
	float acc[3];
	float mfield[3];
	float euler[3];
	float temp;



	//Auxilary Variables
	short clayout[4][3][3];
	short mode;
	int status;
	thread_arg_t thread_arg={
		.p_fakm=&fakm,
		.p_awake=&fl_awake
	};
	int rv;
	pthread_t suspend_tid;
	struct timespec sltime={0,0};		//sleep time


	//Open the device nodes
	fakm=open("/dev/akm8973_daemon", O_RDWR); //Open the drivers
	fbma=open("/dev/bma150", O_RDWR);

	if (fakm==-1 || fbma==-1) {
		fprintf(stderr,"Could not open driver interface");
		exit(1);
	}

	//Get the Layout	(for what??)
	rv=ioctl(fakm,ECS_IOCTL_GET_MATRIX, clayout);

	//Set akm mode (Why isn't the reset applied as dictated by the datasheet???)
	mode=AKECS_MODE_E2P_READ;	//EEPROM access mode	//dunno why!
	rv=ioctl(fakm,ECS_IOCTL_SET_MODE, &mode);
	mode=AKECS_MODE_POWERDOWN; //Power-down mode
	rv=ioctl(fakm,ECS_IOCTL_SET_MODE, &mode);

#ifdef DBG
	struct timespec prtime;		//previous sample time
	struct timespec cutime;		//current sample time
	long udelay;		//real delay between samples in usec
	rv=clock_gettime(CLOCK_MONOTONIC, &prtime);

	int count=1;
	FILE *flog;	//my log file
	flog=fopen("/data/misc/datlog.txt", "w");	//my log file
#endif

	//Start the ultimate loop
	while (1) {

		//Make everything sleep till the sensors get resumed by the Android power manager
		rv=ioctl(fakm, ECS_IOCTL_GET_OPEN_STATUS, &status);	//This make the main thread sleep inside the kernel
		if (status!=1) {
			fprintf(stderr,"undefined GET_OPEN_STATUS behaviour");
			exit(1);
		}

		//We woke up.
		fl_awake=1;

		//Create a thread which checks the suspend status and signal us as soon as the sensors sleep.
		pthread_create(&suspend_tid, NULL, SuspendMonitor, (void *) &thread_arg);

		//Start the keypad polling event - I am not going to implement
		//pthread_create(&sld_tid, NULL, SliderDetectThread, NULL);v

		//Get the calibration parameters
		getcalib(akmcalib, bmacalib, dacoffset, fl_calib);	//Read the calibration matrix elements. Each time the system resumes it reads the new calib parameters.

		//initialize the sensors
		if (fl_awake) init_sensors(fakm, fbma, dacoffset);

		if (fl_calib[2]==1) {	//Perform automatic DAC computation to prevent saturation
			rv=comp_dacoffset(fakm, dacoffset, &fl_awake);
		}

		//Start the main sensor reading loop
		while (fl_awake) {
			rv=read_sensor_data(fakm, fbma, akmdat, bmadat, &delay, &fl_awake);

#ifdef DBG
			//Just for curiosity
			rv=clock_gettime(CLOCK_MONOTONIC, &cutime);
			udelay=(cutime.tv_sec-prtime.tv_sec)*1000000+(cutime.tv_nsec-prtime.tv_nsec)/1000;
			prtime=cutime;
			fprintf(flog,"%ld\t%hd\t%d\t%d\t%d\t%d\t%hd\t%hd\t%hd\n",udelay,delay,akmdat[1],akmdat[2],akmdat[3],akmdat[4],bmadat[0],bmadat[1],bmadat[2]);
			//fprintf(flog,"aaa\n");
			if (count%100 == 0) {
				fflush(flog);
				count=1;
			}
#endif

			if (rv)	//Sensor data cannot be read (probably it was suspended somewhere between main thread execution)
				break;
			else { //We have complete sensor data. Process and publish them.
				//calibrate the temperature and accel data
				calib_tem_data(akmdat, &temp);
				calib_acc_data(bmadat, bmacalib, acc);

				//Compute the dcm_pb from acc
				align_pl(acc, dcm_pb);

				//Mag data automatic calib matrix computation
				if (fl_calib[0]==1)	//we want on the fly calib mat computation
					update_akmcalib(akmdat, acc, akmcalib);	//This updates the calib matrix only if certain conditions are satisfied

				calib_mag_data(akmdat, akmcalib, mfield);

				//Compute the roll pitch yaw
				comp_angles(euler, mfield, dcm_pb);

				//Publish the data over akm interface
				publish_data(fakm, mfield, acc, euler, temp);

#ifdef DBG
			//fprintf(flog,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",euler[0],euler[1],euler[2],acc[0],acc[1],acc[2],mfield[0],mfield[1],mfield[2],temp);
#endif

				//Put the system to sleep for the duration of delay
				sltime.tv_sec=delay/1000;
				sltime.tv_nsec=(delay%1000)*1000000;
				nanosleep(&sltime,NULL);
			}
		}

		//The main thread reaches here iff i)sensors are suspended (fl_awake==0) or ii)data cannot be read in read_sensor_data for some unknown reason
		//We force the main thread to suspend untill sensors are suspended by the framework
		pthread_join(suspend_tid, NULL);

		//make the accelerometers sleep (for some reason it was left as a user space operation)
		mode = BMA_MODE_SLEEP;
		ioctl(fbma, BMA_IOCTL_SET_MODE, &mode);

		//Write calib values for next use
		write_calib(akmcalib, bmacalib, dacoffset, fl_calib);
	}


	//Execution never comes to this point
	close(fakm);
	close(fbma);
#ifdef DBG
	fclose(flog);
#endif
	return 0;
}

void *SuspendMonitor(void *arg) {
	// As soon as the sensors are suspended this routine sets fl_sleep=1 and returns

	thread_arg_t *argp=(thread_arg_t *)arg;		//I hate passing arguments to threads!!!!

	int rv;
	short status;
	int fakm=*(argp->p_fakm);
	char *p_awake=argp->p_awake;

	//This makes this thread sleep until the sensors are suspended by the framework
	rv=ioctl(fakm, ECS_IOCTL_GET_CLOSE_STATUS, &status);

	//Sensors are suspended
	*p_awake=0;

	//Exit the thread
	pthread_exit(NULL);
	return NULL;
}

void getcalib(float akmcalib[][4], float bmacalib[][4], char *dacoffset, char *fl_calib) {
	// In fact all calib params should be in a single file with key-value pairs.
	// But I am too lazy to implement that. Maybe later.
	//

	//Default values
	//Calibration matrices
	akmcalib[0][0]=1; akmcalib[0][1]=0; akmcalib[0][2]=0;
	akmcalib[1][0]=0; akmcalib[1][1]=1; akmcalib[1][2]=0;
	akmcalib[2][0]=0; akmcalib[2][1]=0; akmcalib[2][2]=1;

	bmacalib[0][0]=1; bmacalib[0][1]=0; bmacalib[0][2]=0;
	bmacalib[1][0]=0; bmacalib[1][1]=1; bmacalib[1][2]=0;
	bmacalib[2][0]=0; bmacalib[2][1]=0; bmacalib[2][2]=1;

	//Biases
	akmcalib[0][3]=0;
	akmcalib[1][3]=0;
	akmcalib[2][3]=0;

	bmacalib[0][3]=0;
	bmacalib[1][3]=0;
	bmacalib[2][3]=0;

	//flags (perform implicit calib)
	fl_calib[0]=1;	//akm
	fl_calib[1]=1;	//bma
	fl_calib[2]=1;	//akm dac

	//return;

	//Open the calibration files
	FILE *akmfile=fopen("/data/misc/AKM8973Calib.txt","r");
	FILE *bmafile=fopen("/data/misc/BMA150Calib.txt","r");
	FILE *dacfile=fopen("/data/misc/AKM8973Dac.txt","r");

	fl_calib[0]=(akmfile==NULL ? 1 : 0);
	fl_calib[1]=(bmafile==NULL ? 1 : 0);	//I am not going to use this
	fl_calib[2]=(dacfile==NULL ? 1 : 0);

	if (!fl_calib[0]) {	//File exist, read the calib param
		fscanf(akmfile,"%f %f %f %f", &akmcalib[0][0],&akmcalib[0][1],&akmcalib[0][2],&akmcalib[0][3]);
		fscanf(akmfile,"%f %f %f %f", &akmcalib[1][0],&akmcalib[1][1],&akmcalib[1][2],&akmcalib[1][3]);
		fscanf(akmfile,"%f %f %f %f", &akmcalib[2][0],&akmcalib[2][1],&akmcalib[2][2],&akmcalib[2][3]);
		fclose(akmfile);
	}

	if (!fl_calib[1]) {	//bma file File exists
		fscanf(bmafile,"%f %f %f %f", &bmacalib[0][0],&bmacalib[0][1],&bmacalib[0][2],&bmacalib[0][3]);
		fscanf(bmafile,"%f %f %f %f", &bmacalib[1][0],&bmacalib[1][1],&bmacalib[1][2],&bmacalib[1][3]);
		fscanf(bmafile,"%f %f %f %f", &bmacalib[2][0],&bmacalib[2][1],&bmacalib[2][2],&bmacalib[2][3]);
		fclose(bmafile);
	}

	if (!fl_calib[2]) {	//dac file
		int vr_a[3];
		fscanf(dacfile,"%d %d %d", &vr_a[0],&vr_a[1],&vr_a[2]);
		dacoffset[0]=vr_a[0];	//I just don't like warnings
		dacoffset[1]=vr_a[1];
		dacoffset[2]=vr_a[2];
		fclose(dacfile);
	}

	return;
}

void init_sensors(int fakm, int fbma, char *dacoffset) {
	short mode;
	int rv;
	char rwbuf[5];
	char eeprom[5];

	//Accel init
	mode=BMA_MODE_NORMAL;
	rv=ioctl(fbma, BMA_IOCTL_SET_MODE, &mode);	//Set mode Normal mode
	rv=ioctl(fbma, BMA_IOCTL_INIT, NULL); //This sets acc to 25Hz, +-2g, 20ms wake up time, no interrupt operation.

	//AKM init
	rv=ioctl(fakm, ECS_IOCTL_RESET, NULL); //Reset the akm (sets the akm into power down mode automatically)

	mode=AKECS_MODE_E2P_READ;	//eeprom access mode
	rv=ioctl(fakm, ECS_IOCTL_SET_MODE, &mode);

	eeprom[0]=3; //Read the gain adjustment values
	eeprom[1]=0x66;	//address of the gains in eeprom
	rv=ioctl(fakm, ECS_IOCTL_READ, &eeprom);

	//Set the initial bias and the gain
	mode=AKECS_MODE_POWERDOWN;	//power down mode
	rv=ioctl(fakm, ECS_IOCTL_SET_MODE, &mode);
	rwbuf[0]=4;		//length
	rwbuf[1]=0xe1;	//adress
	rwbuf[2]=dacoffset[0];	//0x0;		//=0x80 bias to be removed by the sensor (HDAC value in the AKM8973Prms.txt)
	rwbuf[3]=dacoffset[1];	//0x0;		//Offset value in that file is also equal to this value multiplied by 14
	rwbuf[4]=dacoffset[2];	//0x0;		//See Spec. page 22
	rv=ioctl(fakm, ECS_IOCTL_WRITE, &rwbuf);		//Write the measurement conditions (DAC - No bias)
	rwbuf[0]=4;
	rwbuf[1]=0xe4;	//Address of the gain register
	rwbuf[2]=eeprom[1];// & 0x0f;		//See spec. page 23
	rwbuf[3]=eeprom[2];// & 0x0f;		//although only the last 4 bits ares used, spec says to transfer whole byte
	rwbuf[4]=eeprom[3];// & 0x0f;
	rv=ioctl(fakm, ECS_IOCTL_WRITE, &rwbuf);		//Write the measurement conditions (Gain)

	return;
}

int comp_dacoffset(int fakm, char *offset, char *fl_awake)
{
	//imitation of AKMD initial DAC offset computation routine
	char rwbuf[5];
	int rv;
	short mode;
	char akmdat[RBUFF_SIZE+1];
	char factor=0x40;
	int i,k;

	for (k=0;k<7;k++) {
		//Read the data
		if (*fl_awake) {
			mode=AKECS_MODE_MEASURE;
			rv=ioctl(fakm, ECS_IOCTL_SET_MODE, &mode);	//Set to measurement mode
			rv=ioctl(fakm, ECS_IOCTL_GETDATA, akmdat);
		}
		else
			return 1;		//Sensors are suspended. Exit the routine


		//Adjust the new offset
		if (k==0)		//initial offsets
			for (i=0;i<3;i++) {
					if (akmdat[i+2]>127)
						offset[i]=0x40;
					else
						offset[i]=0xc0;
			}
		else
			for (i=0;i<3;i++) {
				if (akmdat[i+2]>127)
					if (offset[i]>0x80)
						offset[i]=offset[i]-factor;
					else
						offset[i]=offset[i]+factor;
				else
					if (offset[i]>0x80)
						offset[i]=offset[i]+factor;
					else
						offset[i]=offset[i]-factor;
			}

		//Set the offset
		rwbuf[0]=4;
		rwbuf[1]=0xe1;
		rwbuf[2]=offset[0];
		rwbuf[3]=offset[1];
		rwbuf[4]=offset[2];
		rv=ioctl(fakm, ECS_IOCTL_WRITE, &rwbuf);


		////read the offset values
		//rwbuf[0]=3; //Read the gain adjustment values
		//rwbuf[1]=0xe1;	//address of the gains in eeprom
		//rv=ioctl(fakm, ECS_IOCTL_READ, &rwbuf);

		//divide factor by 2
		factor=factor >> 1;
		//if (factor==0) factor=1;
	}

	//Write the result to the file
	FILE *dacfile=fopen("/data/misc/AKM8973Dac.txt","w");
	fprintf(dacfile,"%d\t%d\t%d",offset[0],offset[1],offset[2]);
	fclose(dacfile);

	return 0;
}


int read_sensor_data(int fakm, int fbma, char *akmdat, short *bmadat, short *delay, char * fl_awake) {
	int rv;
	short mode;	//Read mode

	mode=AKECS_MODE_MEASURE;
	if (*fl_awake) rv=ioctl(fakm, ECS_IOCTL_SET_MODE, &mode);  else return 2;	//Set to measurement mode
	if (rv!=0) return 1;

	if (*fl_awake) rv=ioctl(fakm, ECS_IOCTL_GETDATA, akmdat); else return 2;	//Read the akm outputs
	if (rv!=0) return 1;
	rv=ioctl(fbma, BMA_IOCTL_READ_ACCELERATION, bmadat);	//Read the bma data (no need to check fl_awake, bma does not sleep automatically)
	if (rv!=0) return 1;

	rv=ioctl(fakm, ECS_IOCTL_GET_DELAY , delay);	//Read the delay value
	if (rv!=0) return 1;

	return 0;
}


void calib_tem_data(char *akmdat, float *temp)
{
	*temp=-0.625*((unsigned char) akmdat[1])+110;		//see spec
}

void calib_acc_data(short *bmadat, float bmacalib[][4], float *acc) {
	float vr_a[3];
	//Nominal Calib
	vr_a[0]=GRAV*bmadat[0]/256.0;vr_a[1]=GRAV*bmadat[1]/256.0;vr_a[2]=GRAV*bmadat[2]/256.0;

	//Bias
	vr_a[0]=vr_a[0]+bmacalib[0][3];
	vr_a[1]=vr_a[1]+bmacalib[1][3];
	vr_a[2]=vr_a[2]+bmacalib[2][3];

	//scale, misalign, cross etc
	acc[0]=bmacalib[0][0]*vr_a[0]+bmacalib[0][1]*vr_a[1]+bmacalib[0][2]*vr_a[2]+bmacalib[0][3];
	acc[1]=bmacalib[1][0]*vr_a[0]+bmacalib[1][1]*vr_a[1]+bmacalib[1][2]*vr_a[2]+bmacalib[1][3];
	acc[2]=bmacalib[2][0]*vr_a[0]+bmacalib[2][1]*vr_a[1]+bmacalib[2][2]*vr_a[2]+bmacalib[2][3];

	return;
}

void calib_mag_data(char *akmdat, float akmcalib[][4], float *mfield) {
	float vr_a[3];
	//Calibrate the data (nominal part)
	vr_a[0]=akmdat[2]-127;vr_a[1]=akmdat[3]-127;vr_a[2]=akmdat[4]-127;		//no scale

	//Bias correction
	vr_a[0]=vr_a[0]+akmcalib[0][3];
	vr_a[1]=vr_a[1]+akmcalib[1][3];
	vr_a[2]=vr_a[2]+akmcalib[2][3];
	//Scale, misaling, cross effect corr.
	mfield[0]=akmcalib[0][0]*vr_a[0]+akmcalib[0][1]*vr_a[1]+akmcalib[0][2]*vr_a[2];
	mfield[1]=akmcalib[1][0]*vr_a[0]+akmcalib[1][1]*vr_a[1]+akmcalib[1][2]*vr_a[2]+akmcalib[1][3];
	mfield[2]=akmcalib[2][0]*vr_a[0]+akmcalib[2][1]*vr_a[1]+akmcalib[2][2]*vr_a[2]+akmcalib[2][3];

	return;
}

void update_akmcalib(char *akmdat, float acc[3], float akmcalib[][4]) {
	/*
	 * I am not happy with the performance of this automatic calibration method at all.
	 * There is something wrong with it.
	 * Basically, this method assumes that when the system is horizontal, one should observe a circular output pattern.
	 * However, even though I perfectly align the device to the north, I always observe significant magnetism in all 3 axis.
	 * This might be because of:
	 *  i) external magnetic sources which I don't have any control on, or
	 *  ii)mismatch between the body frames of sensor and the device
	 *
	 * If the reason is the second one, then a spherical match should be used. However it will be computationally very expensive.
	 * Furthermore, even if the biases (and the scale) is estimated with spherical fit, still we need that transformation for heading estimate.
	 *
	 * Someone who has more experience about compass had better revise this part.
	 */


	static float cache[3][CHSIZE];
	static int ind=0;

	//Condition 1: Device must be on a horizontal plane
	if ((acc[0]*acc[0]+acc[1]*acc[1])>ATHRESH)
		return;

	float akm_pl[3];
	//Nominal calibration
	akm_pl[0]=akmdat[2]-127;akm_pl[1]=akmdat[3]-127;akm_pl[2]=akmdat[4]-127;

	//Add the new data to the cache
	if (ind==0) {
		cache[0][0]=akm_pl[0];
		cache[1][0]=akm_pl[1];
		cache[2][0]=akm_pl[2];
		ind=ind+1;
	}
	else {
		float ang;
		float mag=akm_pl[0]*akm_pl[0]+akm_pl[1]*akm_pl[1];
		//Condition 2: The angle between the current and the previous samples should be greater than a threshold
		ang=(cache[0][ind-1]*akm_pl[0]+cache[1][ind-1]*akm_pl[1])/mag;
		if ((1-ang*ang)>CACTHR) {	//add to cache only if the angle (sin) is greater than the threshold
			cache[0][ind]=akm_pl[0];
			cache[1][ind]=akm_pl[1];
			cache[2][ind]=akm_pl[2];
			ind=ind+1;
		}
	}

	//If cache is full compute the calibration parameters
	float zbias=0;
	float cparam[3];
	int i;
	int rv;
	if (ind==CHSIZE) {
		//First compute the z bias
		for (i=0;i<CHSIZE;i++)
			zbias=zbias+cache[2][i];
		zbias=zbias/CHSIZE;

		//Fit circle to x and y values
		rv=fit_circle(cache, cparam);

		if (!rv) { //Circle fit was successful
			//I will use only centre parameters. (no need for arbitrary scaling with radious)
			akmcalib[0][3]=-cparam[0];
			akmcalib[1][3]=-cparam[1];
			akmcalib[2][3]=-zbias;
		}
		else {	//Data is not suitable for circle fit
			ind=0; //Start data collection from the beginning;
			return;
		}

		ind=0;	//restart data caching
	}
	return;
}

void align_pl(float *acc, float dcm_pb[][3]){
	//Computes the platform frame to body frame dcm from the acc data

	float grav;
	grav=sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);

	//Normalise
	float nacc[3];
	nacc[0]=acc[0]/grav;
	nacc[1]=acc[1]/grav;
	nacc[2]=acc[2]/grav;

	float den;
	den=sqrt(1-nacc[0]*nacc[0]);    //Note:assumes pitch is not 90


	//When den~=0, the euler angle singularity arise. For such cases I simply assume roll==0, pitch=+-90
	if (den<ETHRESH)	{ //singularity
		dcm_pb[0][2]=-nacc[0]/fabs(nacc[0]);
		dcm_pb[1][2]=0;
		dcm_pb[2][2]=0;

		dcm_pb[0][1]=0;
		dcm_pb[1][1]=1;
		dcm_pb[2][1]=0;

		dcm_pb[0][0]=0;
		dcm_pb[0][0]=0;
		dcm_pb[0][0]=-dcm_pb[0][2];
	}
	else {
		dcm_pb[0][2]=-nacc[0];
		dcm_pb[1][2]=-nacc[1];
		dcm_pb[2][2]=-nacc[2];

		dcm_pb[0][1]=0;
		dcm_pb[1][1]=-nacc[2]/den;
		dcm_pb[2][1]=nacc[1]/den;

		dcm_pb[0][0]=den;
		dcm_pb[1][0]=-nacc[0]*nacc[1]/den;
		dcm_pb[2][0]=-nacc[0]*nacc[2]/den;
	}
}

//input event poll thread
void *SliderDetectThread(void *arg) {

	//This is to detect whether the slider is open or not.
	//When it is open, this routine changes the compass calibration parameters (akmcalib).
	//If this routine is going to be used, the calib parameters must be mutexed!

	//Also, this routine can be used to perform self calibration operations.
	//(e.g. iOS assumes the first 0.66 sec after the first keypad event is stationary, and uses the corresponding interval to eliminate gyro biases)


	return NULL;
}


//Compute the roll pitch yaw
void comp_angles(float *euler, float *mfield, float dcm_pb[][3]) {

	euler[0]=atan2(dcm_pb[1][2], dcm_pb[2][2]);		//roll
	euler[1]=asin(-dcm_pb[0][2]);		//pitch

	float vr_a[2];
	vr_a[0]=dcm_pb[0][0]*mfield[0]+dcm_pb[1][0]*mfield[1]+dcm_pb[2][0]*mfield[2];
	vr_a[1]=dcm_pb[0][1]*mfield[0]+dcm_pb[1][1]*mfield[1]+dcm_pb[2][1]*mfield[2];
	euler[2]=-atan2(vr_a[1],vr_a[2]);		//heading
}


int fit_circle(float cache[][CHSIZE], float *cparam) {
	//Kasa's method

	int i,k,m;
	float psmat[3][3];
	float psmat_inv[3][3];
	float y[CHSIZE];
	float vr_a[3],vr_b[3];
	int rv;


	//Replace 3rd row with ones
	for (i=0;i<CHSIZE;i++)
		cache[2][i]=1;

	//pseudo matrix (A'A)
	for (i=0;i<3;i++)
		for (k=0;k<3;k++) {
			psmat[i][k]=0;
			for (m=0;m<CHSIZE;m++)
				psmat[i][k]=psmat[i][k]+cache[i][m]*cache[k][m];
		}

	//y
	for (i=0;i<CHSIZE;i++)
		y[i]=cache[0][i]*cache[0][i]+cache[1][i]*cache[1][i];

	//vr_a=A'*y
	for (i=0;i<3;i++) {
		vr_a[i]=0;
		for (k=0;k<CHSIZE;k++)
			vr_a[i]=vr_a[i]+cache[i][k]*y[k];
	}

	//Pseudo inverse matrix
	rv=comp_3x3inv(psmat, psmat_inv);

	if (rv)
		//psmat is singular.
		return 1;

	//result=inv(A'A)*A'y;
	for (i=0;i<3;i++) {
		vr_b[i]=0;
		for (k=0;k<3;k++)
			vr_b[i]=vr_b[i]+psmat_inv[i][k]*vr_a[k];
	}

	//Convert result to the circular parameters
	cparam[0]=vr_b[0]/2;	//x bias
	cparam[1]=vr_b[1]/2;	//y bias
	cparam[2]=sqrt((vr_b[0]*vr_b[0]+vr_b[1]*vr_b[1])/4.0+vr_b[2]);	//Radius

	return 0;
}

int comp_3x3inv(float A[][3], float Ainv[][3]) {
	//hard coded 3x3 inverse
	float det=0;

	det=A[0][0]*A[1][1]*A[2][2];
	det=det+A[0][1]*A[1][2]*A[2][0];
	det=det+A[0][2]*A[1][0]*A[2][1];
	det=det-A[0][2]*A[1][1]*A[2][0];
	det=det-A[0][1]*A[1][0]*A[2][2];
	det=det-A[0][0]*A[1][2]*A[2][1];

	if (abs(det)<MTHRESH)
		return 1; //matrix is singular


	float detinv=1.0/det;

	Ainv[0][0]=(A[1][1]*A[2][2]-A[2][1]*A[1][2])*detinv;
	Ainv[0][1]=-(A[1][0]*A[2][2]-A[1][2]*A[2][0])*detinv;
	Ainv[0][2]=(A[1][0]*A[2][1]-A[2][0]*A[1][1])*detinv;
	Ainv[1][0]=-(A[0][1]*A[2][2]-A[0][2]*A[2][1])*detinv;
	Ainv[1][1]=(A[0][0]*A[2][2]-A[0][2]*A[2][0])*detinv;
	Ainv[1][2]=-(A[0][0]*A[2][1]-A[2][0]*A[0][1])*detinv;
	Ainv[2][0]=(A[0][1]*A[1][2]-A[0][2]*A[1][1])*detinv;
	Ainv[2][1]=-(A[0][0]*A[1][2]-A[1][0]*A[0][2])*detinv;
	Ainv[2][2]=(A[0][0]*A[1][1]-A[1][0]*A[0][1])*detinv;

	return 0;
}


void publish_data(int fakm, float *mfield, float *acc, float *euler, float temp) {
	//Sensor body frame

	int rv;
	short frmdata[12];

	frmdata[0]=(short) -(euler[2]*180.0/PI);	//rot around z (yaw)
	frmdata[1]=(short) -(euler[0]*180.0/PI);	//rot around x (pitch)
	frmdata[2]=(short) (euler[1]*180.0/PI);	//rot around y (roll)

	frmdata[3]=(short) temp;
	frmdata[4]=1;	//mstat		//low
	frmdata[5]=1;	//gstat	- not used

	//Devided by grav becouse htc's sensor.c assumes output in G and converts it m/s^2.
	frmdata[6]=acc[0]*720/GRAV; //android -x
	frmdata[7]=acc[2]*720/GRAV;	//android -z
	frmdata[8]=acc[1]*720/GRAV; //andorid y

	float mag=1; //sensor output is in uT
	frmdata[9]=mfield[0]*16/mag;
	frmdata[10]=mfield[1]*16/mag;
	frmdata[11]=mfield[2]*16/mag;

	//Note: 720 and 16 are the parameters used in sensors.c

	//Create the input event
	rv=ioctl(fakm, ECS_IOCTL_SET_YPR, frmdata);

	return;
}

void write_calib(float akmcalib[][4], float bmacalib[][4], char *dacoffset, char *fl_calib) {

	if (fl_calib[2]==1)	{ //dac values were re-computed
		FILE *fd=fopen("/data/misc/AKM8973Dac.txt","w");
		fprintf(fd, "%d\t%d\t%d\n", dacoffset[0], dacoffset[1], dacoffset[2]);
		fclose(fd);
	}

	if (fl_calib[1]==1)	{ //bma values were re-computed
		//Do nothing becuase there is no acc self calibration in this daemon (nor in akmd)
	}

	if (fl_calib[0]==1)	{ //akmcalib values were re-computed
		FILE *fd=fopen("/data/misc/AKM8973Calib.txt","w");
		fprintf(fd,"%f\t%f\t%f\t%f\n", akmcalib[0][0],akmcalib[0][1],akmcalib[0][2],akmcalib[0][3]);
		fprintf(fd,"%f\t%f\t%f\t%f\n", akmcalib[1][0],akmcalib[1][1],akmcalib[1][2],akmcalib[1][3]);
		fprintf(fd,"%f\t%f\t%f\t%f\n", akmcalib[2][0],akmcalib[2][1],akmcalib[2][2],akmcalib[2][3]);
		fclose(fd);
	}

	return;
}
