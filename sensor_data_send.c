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
	int i=0,j=0;
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct can_frame frame_64[64]={0};
	struct ifreq ifr;
	const char *ifname = "can0";

	char str_tmp[1024];
	int curr_timestamp;
	int next_timestamp;
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
			if(First){
				fgets(str_tmp,1024,Chassis_time_File);
				curr_timestamp=atoi(str_tmp);
				First=0;
			}
			fgets(str_tmp,1024,Chassis_time_File);
			next_timestamp=atoi(str_tmp);

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
					p=strtok(NULL,",");
					p=strtok(NULL,",");
					p=strtok(NULL,",");
					frame.data[0]=atoi(p);
					p=strtok(NULL,",");
			}
			//About Steering Angle
		
			fgets(str_tmp,1024,Steering_Angle_File);
			p=strtok(str_tmp,",");
		//	printf("steer angel : %f\n",atof(p));
			frame.data[1]=((int)(atof(p)*10+5400+0.5));
			frame.data[2]=(((int)(atof(p)*10+5400+0.5))>>8);
			p=strtok(NULL,",");
			while(1){
				if(gettimeafterboot()>standard_time){
					printf("time : %f\n",gettimeafterboot());
					write(s, &frame, sizeof(struct can_frame));
					close(s);
					break;
				}else {
					usleep((standard_time-gettimeafterboot())*1000);
					continue;
				}
			}
			//printf("gettimeafterboot() : %f\n",gettimeafterboot());
			standard_time+=(next_timestamp-curr_timestamp);
			curr_timestamp=next_timestamp;
			
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
					frame_64[cnt].data[0]=(int)(atof(p)*10+512+0.5);
					frame_64[cnt].data[1]=(int)(atof(p)*10+512+0.5)>>8;
					cnt++;
					p=strtok(NULL,",");
				}
				
				fgets(str_tmp,1024,Radar_Range_File);//0~200
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[1]=frame_64[cnt].data[1]|(((int)(atof(p)*10))<<2);
					frame_64[cnt].data[2]=((int)(atof(p)*10))>>6;
					cnt++;
					p=strtok(NULL,",");
				}

				fgets(str_tmp,1024,Radar_Width_File);
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[2]=frame_64[cnt].data[2]|(((int)(atof(p)*2))<<5);
					frame_64[cnt].data[3]=((int)(atof(p)*2))>>3;
					cnt++;
					p=strtok(NULL,",");
				}


				fgets(str_tmp,1024,Radar_RangeAccel_File);
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[3]=frame_64[cnt].data[3]|(((int)(atof(p)*20+512+0.5))<<1);
					frame_64[cnt].data[4]=((int)(atof(p)*20+512+0.5))>>7;
					cnt++;
					p=strtok(NULL,",");
				}

				fgets(str_tmp,1024,Radar_RangeRate_File);
				p=strtok(str_tmp,",");
				cnt=0;
				while(p!=NULL){
					frame_64[cnt].data[4]=frame_64[cnt].data[4]|(((int)(atof(p)*10+8192+0.5))<<3);
					frame_64[cnt].data[5]=((int)(atof(p)*10+8192+0.5))>>5;
					frame_64[cnt].data[6]=((int)(atof(p)*10+8192+0.5))>>13;
					cnt++;
					p=strtok(NULL,",");
				}


				for(int j=0;j<64;j++){
					write(s, &frame_64[j], sizeof(struct can_frame));
					usleep(2);
				}
				close(s);
				
			}	
			if(j<8) j++;
			else if (j==8){
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
			
				frame.can_id  = 0x7ff;
				frame.can_dlc = 8;
				for(int k=0;k<frame.can_dlc;k++)
					frame.data[k]=0;
				write(s, &frame, sizeof(struct can_frame));
				close(s);
				j++;
			}
			else{}
		}
	}
	return 0;
}



