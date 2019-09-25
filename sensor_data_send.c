#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <time.h>

#include <linux/can.h>
#include <linux/can/raw.h>

double gettimeafterboot(){
	struct timespec time_after_boot;
	clock_gettime(CLOCK_MONOTONIC,&time_after_boot);
	return (time_after_boot.tv_sec*1000+time_after_boot.tv_nsec*0.000001);
}

int main(void){
	int i=0;
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct can_frame frame_64[64]={0};
	struct ifreq ifr;
	const char *ifname = "can0";

	char str_tmp[1024];
	char curr_timestamp[1024];
	char prev_timestamp[1024];
	char *p;
	float b[4]={0};
	int cnt;

	FILE *Radar_Amplitude_File=fopen("excel/Radar_Amplitude.csv","r");
	FILE *Radar_Angle_File=fopen("excel/Radar_Angle.csv","r");
	FILE *Radar_LateRate_File=fopen("excel/Radar_LateRate.csv","r");
	FILE *Radar_Range_File=fopen("excel/Radar_Range.csv","r");
	FILE *Radar_RangeAccel_File=fopen("excel/Radar_RangeAccel.csv","r");
	FILE *Radar_RangeRate_File=fopen("excel/Radar_RangeRate.csv","r");
	FILE *Radar_time_File=fopen("excel/Radar_time.csv","r");
	FILE *Radar_ValidLevel_File=fopen("excel/Radar_ValidLevel.csv","r");
	FILE *Radar_Width_File=fopen("excel/Radar_Width.csv","r");

	FILE *Chassis_time_File=fopen("excel/Chassis_time.csv","r");
	FILE *Steering_Angle_File=fopen("excel/Steering_Angle.csv","r");
	FILE *Vehicle_Speed_File=fopen("excel/Vehicle_Speed.csv","r");
	int First=1;
	double standard_time=gettimeafterboot();
	if(Chassis_time_File!=NULL){
		while(!feof(Chassis_time_File)){
			//About Vehicle Speed
			fgets(curr_timestamp,1024,Chassis_time_File);
			if(First) {memcpy(prev_timestamp,curr_timestamp,sizeof(curr_timestamp));First=0;}
			if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
				perror("Error while opening socket");
			}
		
			strcpy(ifr.ifr_name, ifname);
			ioctl(s, SIOCGIFINDEX, &ifr);
			
			addr.can_family  = AF_CAN;
			addr.can_ifindex = ifr.ifr_ifindex;
		
			if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
				perror("Error in socket bind");
			}
		
			frame.can_id  = 0x003;
			frame.can_dlc = 8;
			fgets(str_tmp,1024,Vehicle_Speed_File);
			p=strtok(str_tmp,",");
			cnt=0;
			while(p!=NULL){
				if(gettimeafterboot()>standard_time){
					p=strtok(NULL,",");
					p=strtok(NULL,",");
					p=strtok(NULL,",");
					frame.data[0]=atoi(p);
					//cnt++;
					p=strtok(NULL,",");
				}else {
usleep((standard_time-gettimeafterboot())*1000); continue;}
			}
			//About Steering Angle
		
			fgets(str_tmp,1024,Steering_Angle_File);
			p=strtok(str_tmp,",");
			printf("steer angel : %f\n",atof(p));
			frame.data[1]=((int)atof(p)*10+5400);
			frame.data[2]=(((int)atof(p)*10+5400)>>8);
			p=strtok(NULL,",");
			write(s, &frame, sizeof(struct can_frame));
			close(s);
			printf("gettimeafterboot() : %f\n",gettimeafterboot());
			standard_time+=(atoi(curr_timestamp)-atoi(prev_timestamp));
			memcpy(prev_timestamp,curr_timestamp,sizeof(curr_timestamp));
			
			i=(i+1)%5;
			if(i%5==0){//50ms period of Radar
				if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
					perror("Error while opening socket");
				}
			
				strcpy(ifr.ifr_name, ifname);
				ioctl(s, SIOCGIFINDEX, &ifr);
				
				addr.can_family  = AF_CAN;
				addr.can_ifindex = ifr.ifr_ifindex;
			
				if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
					perror("Error in socket bind");
				}
			
				for(int j=0;j<64;j++){
				frame_64[j].can_id  = j+4;
				frame_64[j].can_dlc = 8;
				}

				//Radar data 
				fgets(str_tmp,1024,Radar_Angle_File);
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[0]=(int)(atof(p)*10+512);
					frame_64[cnt].data[1]=(int)(atof(p)*10+512)>>8;
					cnt++;
					p=strtok(NULL,",");
				}
				
				fgets(str_tmp,1024,Radar_Range_File);//0~200
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[1]=frame_64[cnt].data[1]|(atoi(p)<<2);
					frame_64[cnt].data[2]=(atoi(p)>>8);
					cnt++;
					p=strtok(NULL,",");
				}

				fgets(str_tmp,1024,Radar_Width_File);
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[2]=frame_64[cnt].data[2]|(atoi(p)<<5);
					frame_64[cnt].data[3]=atoi(p)>>3;
					cnt++;
					p=strtok(NULL,",");
				}


				fgets(str_tmp,1024,Radar_RangeAccel_File);
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[3]=frame_64[cnt].data[3]|(atoi(p)<<1);
					frame_64[cnt].data[4]=atoi(p)>>7;
					cnt++;
					p=strtok(NULL,",");
				}

				fgets(str_tmp,1024,Radar_RangeRate_File);
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[4]=frame_64[cnt].data[4]|(atoi(p)<<3);
					frame_64[cnt].data[5]=atoi(p)>>5;
					frame_64[cnt].data[6]=atoi(p)>>13;
					cnt++;
					p=strtok(NULL,",");
				}


				for(int j=0;j<64;j++){
					write(s, &frame_64[j], sizeof(struct can_frame));
					usleep(1);
				}
				close(s);
				
			}	
		}
	}
	return 0;
}



