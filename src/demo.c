#define _GNU_SOURCE
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include <sys/time.h>
#include <unistd.h>

#define DEMO 1

#ifdef OPENCV

#define iteration 100
#define start_log 10
#define cycle 4

static char **demo_names;
static image **demo_alphabet;
static int demo_classes;

static network *net;
static image buff [3];
static image buff_letter[3];
static int buff_index = 0;
static void * cap;
static float fps = 0;
static float demo_thresh = 0;
static float demo_hier = .5;
static int running = 0;

static int demo_frame = 3;
static int demo_index = 0;
static float **predictions;
static float *avg;
static int demo_done = 0;
static int demo_total = 0;
double demo_time;

static double image_waiting_array[iteration];
static double image_cycle_array[iteration];
static double fetch_array[iteration];
static double detect_array[iteration];
static double display_array[iteration];
static double slack[iteration];
static double fps_array[iteration];
static double latency[iteration];

static int count=0;

float camera_fps=0;
float *ptr_camera_fps=&camera_fps;
long int frame_number[3];
double frame_timestamp[3];
static double detect_start;
static double detect_end;
static double detect_time;
static double display_time;
static double fetch_start;
static double fetch_time;
static double image_waiting_time;
static double image_cycle;
static double before_image_waiting_time; //edit jang
static int sleep_time;
int apply_offset_=0;

double gettimeafterboot()
{
	struct timespec time_after_boot;
	clock_gettime(CLOCK_MONOTONIC,&time_after_boot);
	return (time_after_boot.tv_sec*1000+time_after_boot.tv_nsec*0.000001);
}

detection *get_network_boxes(network *net, int w, int h, float thresh, float hier, int *map, int relative, int *num);

int size_network(network *net)
{
    int i;
    int count = 0;
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            count += l.outputs;
        }
    }
    return count;
}

void remember_network(network *net)
{
    int i;
    int count = 0;
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(predictions[demo_index] + count, net->layers[i].output, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
}

detection *avg_predictions(network *net, int *nboxes)
{
    int i, j;
    int count = 0;
    fill_cpu(demo_total, 0, avg, 1);
    for(j = 0; j < demo_frame; ++j){
        axpy_cpu(demo_total, 1./demo_frame, predictions[j], 1, avg, 1);
    }
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(l.output, avg + count, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
    detection *dets = get_network_boxes(net, buff[0].w, buff[0].h, demo_thresh, demo_hier, 0, 1, nboxes);
    return dets;
}

void *detect_in_thread(void *ptr)
{
	detect_start=gettimeafterboot();
    running = 1;
    float nms = .4;

    layer l = net->layers[net->n-1];
    float *X = buff_letter[(buff_index+2)%3].data;
    network_predict(net, X);

    remember_network(net);
    detection *dets = 0;
    int nboxes = 0;
    dets = avg_predictions(net, &nboxes);

    if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.1f\n",fps);
    printf("Objects:\n\n");
    image display = buff[(buff_index+2) % 3];
    draw_detections(display, dets, nboxes, demo_thresh, demo_names, demo_alphabet, demo_classes);
    free_detections(dets, nboxes);

    demo_index = (demo_index + 1)%demo_frame;
    running = 0;
	detect_end=gettimeafterboot();
	detect_time=detect_end-detect_start;
	if(count>=start_log) detect_array[count-start_log]=detect_time;
    return 0;
}

void *fetch_in_thread(void *ptr)
{
	usleep(sleep_time*1000);
	fetch_start=gettimeafterboot();
    free_image(buff[buff_index]);
//    buff[buff_index] = get_image_from_stream(cap);
    buff[buff_index] = get_image_from_stream_timestamp(cap,frame_timestamp,buff_index);
	image_waiting_time=frame_timestamp[buff_index]-fetch_start;  //edit jang
	image_cycle=frame_timestamp[buff_index]-before_image_waiting_time;
	before_image_waiting_time = frame_timestamp[buff_index];

    if(buff[buff_index].data == 0) {
        demo_done = 1;
        return 0;
    }
    letterbox_image_into(buff[buff_index], net->w, net->h, buff_letter[buff_index]);
	fetch_time=gettimeafterboot()-fetch_start;
	if(count>=start_log){
		fetch_array[count-start_log]=fetch_time;
		image_waiting_array[count-start_log]=image_waiting_time;
		image_cycle_array[count-start_log]=image_cycle;
	}
    return 0;
}

void *display_in_thread(void *ptr)
{
	int c;
	if(!apply_offset_) c = show_image(buff[(buff_index + 1)%3], "Demo", 1); 
	else c = show_image(buff[(buff_index + 2)%3], "Demo", 1); 
    if (c != -1) c = c%256;
    if (c == 27) {
        demo_done = 1;
        return 0;
    } else if (c == 82) {
        demo_thresh += .02;
    } else if (c == 84) {
        demo_thresh -= .02;
        if(demo_thresh <= .02) demo_thresh = .02;
    } else if (c == 83) {
        demo_hier += .02;
    } else if (c == 81) {
        demo_hier -= .02;
        if(demo_hier <= .0) demo_hier = .0;
    }

	double now_time=gettimeafterboot();
	display_time=now_time-detect_end;
	if(count>=start_log){
		fps_array[count-start_log]=fps;
		if(!apply_offset_) latency[count-start_log]=now_time-frame_timestamp[(buff_index+1)%3];
		else latency[count-start_log]=now_time-frame_timestamp[(buff_index+2)%3];
		display_array[count-start_log]=display_time;
		printf("latency[%d]: %f\n",count-start_log,latency[count-start_log]);
		printf("count : %d\n",count);
	}
    return 0;
}

void *display_loop(void *ptr)
{
    while(1){
        display_in_thread(0);
    }
}

void *detect_loop(void *ptr)
{
    while(1){
        detect_in_thread(0);
    }
}

void demo_with_opencv(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen, int opencv_buffer_size, int apply_offset)
{
    //demo_frame = avg_frames;
    image **alphabet = load_alphabet();
    demo_names = names;
    demo_alphabet = alphabet;
    demo_classes = classes;
    demo_thresh = thresh;
    demo_hier = hier;
    printf("Demo\n");
    net = load_network(cfgfile, weightfile, 0);
    set_batch_network(net, 1);
    pthread_t detect_thread;
    pthread_t fetch_thread;
	apply_offset_=apply_offset;
    srand(2222222);

    int i;
    demo_total = size_network(net);
    predictions = calloc(demo_frame, sizeof(float*));
    for (i = 0; i < demo_frame; ++i){
        predictions[i] = calloc(demo_total, sizeof(float));
    }
    avg = calloc(demo_total, sizeof(float));

    if(filename){
        printf("video file: %s\n", filename);
        cap = open_video_stream(filename, 0, 0, 0, 0);
    }else{
        cap = open_video_stream_cam_fps(0, cam_index, w, h, frames, ptr_camera_fps,opencv_buffer_size);
    }

    if(!cap) error("Couldn't connect to webcam.\n");

    buff[0] = get_image_from_stream(cap);
    buff[1] = copy_image(buff[0]);
    buff[2] = copy_image(buff[0]);
    buff_letter[0] = letterbox_image(buff[0], net->w, net->h);
    buff_letter[1] = letterbox_image(buff[0], net->w, net->h);
    buff_letter[2] = letterbox_image(buff[0], net->w, net->h);


    if(!prefix){
        make_window("Demo", 1352, 1013, fullscreen);
    }

    //demo_time = what_time_is_it_now();
	demo_time=gettimeafterboot();
	
	double image_waiting_sum[cycle]={0};
	double fetch_sum[cycle]={0};
	double detect_sum[cycle]={0};
	double display_sum[cycle]={0};
	double slack_sum[cycle]={0};
	double fps_sum[cycle]={0};
	double latency_sum[cycle]={0};
	double image_cycle_sum[cycle]={0};

	sleep_time=0;

	for(int iter=0;iter<cycle;iter++){
		//if(iter==0) set_opencv_buffer_size(cap,1);
		while(!demo_done){
			if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
			if(!apply_offset_) if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");
			if(!prefix){
				fps=1./(gettimeafterboot()-demo_time)*1000;
				demo_time=gettimeafterboot();
				if(apply_offset_) detect_in_thread(0);
				display_in_thread(0);
	
	        }else{
	            char name[256];
	            sprintf(name, "%s_%08d", prefix, count);
	            save_image(buff[(buff_index + 2)%3], name);
	        }
	        pthread_join(fetch_thread, 0);
	        if(!apply_offset_) pthread_join(detect_thread, 0);
			if(count>=start_log)
				slack[count-start_log]=(detect_time+display_time)-(sleep_time+fetch_time);
			if(count==(iteration+start_log-1)){
				FILE *fp;
				char s1[35]="191008/offset_";
				char s2[4];
				sprintf(s2,"%d",sleep_time);
				char s3[5]=".csv";
				strcat(s1,s2);
				strcat(s1,s3);
				
				fp=fopen(s1,"w+");
				for(int i=0;i<iteration;i++){
					image_waiting_sum[iter]+=image_waiting_array[i];
					fetch_sum[iter]+=fetch_array[i];
					detect_sum[iter]+=detect_array[i];
					display_sum[iter]+=display_array[i];
					slack_sum[iter]+=slack[i];
					fps_sum[iter]+=fps_array[i];
					latency_sum[iter]+=latency[i];
					image_cycle_sum[iter]+=image_cycle_array[i];
					fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f\n",image_waiting_array[i],fetch_array[i],detect_array[i],	display_array[i],slack[i],fps_array[i],latency[i],image_cycle_array[i]);
				}
				fclose(fp);
				if(apply_offset){
					if(iter==0)
						sleep_time=(int)detect_sum[0]/iteration-1000./(2*(int)(camera_fps));
					else if (iter>0) 
						sleep_time=(int)detect_sum[1]/iteration-1000./(2*(int)(camera_fps));
					break;
				}
			}
			count++;
	    	buff_index = (buff_index + 1) %3;

		}
		count=0;
	}
	for(i=0; i<cycle;i++){
		printf("avg_image_waiting[%d] : %f\n",i,image_waiting_sum[i]/iteration);
		printf("avg_fetch[%d] : %f\n",i,fetch_sum[i]/iteration);
		printf("avg_detect[%d] : %f\n",i,detect_sum[i]/iteration);
		printf("avg_display[%d] : %f\n",i,display_sum[i]/iteration);
		printf("avg_slack[%d] : %f\n",i,slack_sum[i]/iteration);
		printf("avg_fps[%d] : %f\n",i,fps_sum[i]/iteration);
		printf("avg_latency[%d] : %f\n",i,latency_sum[i]/iteration);
		printf("avg_image_cycle[%d] : %f\n",i,image_cycle_sum[i]/iteration);
	}
}
/*
   void demo_compare(char *cfg1, char *weight1, char *cfg2, char *weight2, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
   {
   demo_frame = avg_frames;
   predictions = calloc(demo_frame, sizeof(float*));
   image **alphabet = load_alphabet();
   demo_names = names;
   demo_alphabet = alphabet;
   demo_classes = classes;
   demo_thresh = thresh;
   demo_hier = hier;
   printf("Demo\n");
   net = load_network(cfg1, weight1, 0);
   set_batch_network(net, 1);
   pthread_t detect_thread;
   pthread_t fetch_thread;

   srand(2222222);

   if(filename){
   printf("video file: %s\n", filename);
   cap = cvCaptureFromFile(filename);
   }else{
   cap = cvCaptureFromCAM(cam_index);

   if(w){
   cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH, w);
   }
   if(h){
   cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, h);
   }
   if(frames){
   cvSetCaptureProperty(cap, CV_CAP_PROP_FPS, frames);
   }
   }

   if(!cap) error("Couldn't connect to webcam.\n");

   layer l = net->layers[net->n-1];
   demo_detections = l.n*l.w*l.h;
   int j;

   avg = (float *) calloc(l.outputs, sizeof(float));
   for(j = 0; j < demo_frame; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));

   boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
   probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
   for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes+1, sizeof(float));

   buff[0] = get_image_from_stream(cap);
   buff[1] = copy_image(buff[0]);
   buff[2] = copy_image(buff[0]);
   buff_letter[0] = letterbox_image(buff[0], net->w, net->h);
   buff_letter[1] = letterbox_image(buff[0], net->w, net->h);
   buff_letter[2] = letterbox_image(buff[0], net->w, net->h);
   ipl = cvCreateImage(cvSize(buff[0].w,buff[0].h), IPL_DEPTH_8U, buff[0].c);

   int count = 0;
   if(!prefix){
   cvNamedWindow("Demo", CV_WINDOW_NORMAL); 
   if(fullscreen){
   cvSetWindowProperty("Demo", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
   } else {
   cvMoveWindow("Demo", 0, 0);
   cvResizeWindow("Demo", 1352, 1013);
   }
   }

   demo_time = what_time_is_it_now();

   while(!demo_done){
buff_index = (buff_index + 1) %3;
if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");
if(!prefix){
    fps = 1./(what_time_is_it_now() - demo_time);
    demo_time = what_time_is_it_now();
    display_in_thread(0);
}else{
    char name[256];
    sprintf(name, "%s_%08d", prefix, count);
    save_image(buff[(buff_index + 1)%3], name);
}
pthread_join(fetch_thread, 0);
pthread_join(detect_thread, 0);
++count;
}
}
*/
#else
void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg, float hier, int w, int h, int frames, int fullscreen)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}





#endif



