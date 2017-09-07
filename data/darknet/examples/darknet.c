#include "darknet.h"
#include "math.h"
#define pi 3.1415926
/*
detector_t make_detector_t()
{
    detector_t detector={0};
    return detector;
}

detector_t YOLO_Init(char *cfgfile, char *weightfile)
{
    //image **alphabet = load_alphabet();

    detector_t detector=make_detector_t();


    detector.net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&detector.net, weightfile);
    }
    detector.net.gpu_index=1;
    
    set_batch_network(&detector.net, 1);
    srand(2222222);

    layer l = detector.net.layers[detector.net.n-1];

    detector.boxes = (box*)calloc(l.w*l.h*l.n, sizeof(box));
    detector.probs = (float**)calloc(l.w*l.h*l.n, sizeof(float *));
    int j=0;
    for(j = 0; j < l.w*l.h*l.n; ++j) detector.probs[j] = (float*)calloc(l.classes + 1, sizeof(float *));
    
    return detector;
}

int YOLO_Clear(detector_t* detector)
{
   layer l = detector->net.layers[detector->net.n-1];
   free(detector->boxes);
   free_ptrs((void **)detector->probs, l.w*l.h*l.n);
   return 1;
}


bbox_v YOLO_Process(detector_t detector,image im,float thresh)
{

        float hier_thresh=0.5;
	float nms=.4;
	


	image sized = letterbox_image(im, detector.net.w, detector.net.h);
	float *X = sized.data;
	layer l = detector.net.layers[detector.net.n-1];
	network_predict(detector.net, X);

	get_region_boxes(l, im.w, im.h, detector.net.w, detector.net.h, thresh, detector.probs, detector.boxes, 0, 0, hier_thresh, 1);
	if (nms) do_nms_obj(detector.boxes, detector.probs, l.w*l.h*l.n, l.classes, nms);

        
        
	
	//list *options = read_data_cfg(datacfg);
	//char *name_list = option_find_str(options, "names", "data/names.list");
	//char **names = get_labels(name_list);

	//image **alphabet = load_alphabet();	

        //draw_detections(im, l.w*l.h*l.n, thresh, detector.boxes, detector.probs, names, alphabet, l.classes);
        size_t i;
        unsigned int bbox_count=0;
        for (i = 0; i < (l.w*l.h*l.n); ++i) {
		int const obj_id = max_index(detector.probs[i], l.classes);
		float const prob = detector.probs[i][obj_id];		
		if (prob > thresh&&obj_id==6) bbox_count++;
	}

        bbox_v bboxinfo;
        bboxinfo.bboxptr=(bbox_t*)calloc(bbox_count,sizeof(bbox_t));
        bboxinfo.bbox_count=bbox_count;
        bbox_t* tempptr=bboxinfo.bboxptr;
        for (i = 0; i < (l.w*l.h*l.n); ++i) {
		box b = detector.boxes[i];
		int const obj_id = max_index(detector.probs[i], l.classes);
		float const prob = detector.probs[i][obj_id];
		
		if (prob > thresh&&obj_id==6) 
		{
                        tempptr->x=(unsigned int)((b.x - b.w / 2.)*im.w>0?(b.x - b.w / 2.)*im.w:0);
                        tempptr->y=(unsigned int)((b.y - b.h / 2.)*im.h>0?(b.y - b.h / 2.)*im.h:0);
                        tempptr->w=(unsigned int)(b.w*im.w);
                        tempptr->h=(unsigned int)(b.h*im.h);
                        tempptr->prob=prob;
                        tempptr->obj_id=obj_id;
                        tempptr++;
		}
	}


        if(sized.data) free(sized.data);
	return bboxinfo;

}

ObsDetInstance make_ObsDetInstance()
{
    ObsDetInstance aInstance={0};
    return aInstance;
}



ObsDetInstance ObsDetInstance_Init(float  acamera_height,
	float  atheta_Init,
	float* acameraInternalParam,
	unsigned int awidth,
	unsigned int aheight,
	float ROI_h,
	float ROI_w)
{
        ObsDetInstance aInstance=make_ObsDetInstance();
	aInstance.camera_height = acamera_height;
	aInstance.theta_Init = atheta_Init;
	aInstance.cameraInternalParam_fx = acameraInternalParam[0];
	aInstance.cameraInternalParam_cx = acameraInternalParam[2];
	aInstance.cameraInternalParam_fy = acameraInternalParam[4];
	aInstance.cameraInternalParam_cy = acameraInternalParam[5];	
	aInstance.Img_width = awidth;
	aInstance.Img_height = aheight;


	float ROI_y = (1.0 - ROI_h);
	float ROI_x = (1.0 - ROI_w) / 2;
	aInstance.ROI.x=(unsigned int)(awidth *ROI_x);
        aInstance.ROI.y=(unsigned int)(aheight * ROI_y);
        aInstance.ROI.w=(unsigned int)(awidth * ROI_w);
	aInstance.ROI.h=(unsigned int)(aheight*ROI_h);
	float lowest_board = (float)(aheight - aInstance.cameraInternalParam_cy) / aInstance.cameraInternalParam_fy;
	float farest_board = (float)(floor(aheight * ROI_y) - aInstance.cameraInternalParam_cy) / aInstance.cameraInternalParam_fy;


	float near_theta = (float)(pi / 2 - atan((1.0 / lowest_board)) + aInstance.theta_Init);
	float far_theta = (float)(pi / 2 - atan((1.0 / farest_board)) + aInstance.theta_Init);



	float near_dist, far_dist;
	near_dist = aInstance.camera_height / tan((double)(near_theta));
	far_dist = aInstance.camera_height / tan((double)(far_theta));

	printf("near_dist(cm):%f,far_dist(cm):%f\n",near_dist,far_dist);
	aInstance.estimate_distance = 0;
	aInstance.estimate_objwidth = 0;
	return aInstance;
}

bbox_t overlapRoi(bbox_t a,bbox_t b)
{
	bbox_t roi={0};
	int x_tl=a.x>b.x?a.x:b.x;
	int y_tl=a.y>b.y?a.y:b.y;
	int x_br=(a.x+a.w)>(b.x+b.w)?(b.x+b.w):(a.x+a.w);
	int y_br=(a.y+a.h)>(b.y+b.h)?(b.y+b.h):(a.y+a.h);
	if(x_tl<x_br&&y_tl<y_br)
	{
		roi.x=x_tl;
		roi.y=y_tl;
		roi.w=x_br-x_tl;
		roi.h=y_br-y_tl;
	}
	return roi;
}

int ObsDetInstance_Process(ObsDetInstance* aInstance,bbox_v bbox_info)
{
    line_t board_line={0};
    if(bbox_info.bbox_count>0)
    {
	int i;
	for(i=0;i<bbox_info.bbox_count;i++)
	{
		bbox_t ObjRoi=overlapRoi(aInstance->ROI,bbox_info.bboxptr[i]);
		if(ObjRoi.w&&ObjRoi.h)
		{
			line_t candidate_board_line;
			candidate_board_line.u = ObjRoi.x;
			candidate_board_line.v = ObjRoi.y + ObjRoi.h;
			candidate_board_line.linelength = ObjRoi.w;
			
			if(candidate_board_line.v>board_line.v)
			{
				board_line.v=candidate_board_line.v;
				board_line.u=candidate_board_line.u;
				board_line.linelength=candidate_board_line.linelength;
			}
		}
	}
    }

    if(board_line.linelength>0)
    {
	    //printf("board_line:%d,%d,%d\n",board_line.u,board_line.v,board_line.linelength);

	    float board_line_height = (float)(board_line.v - aInstance->cameraInternalParam_cy) / aInstance->cameraInternalParam_fy;
	    float board_line_width = (float)(board_line.linelength) / aInstance->cameraInternalParam_fx;

	    float theta = (float)(pi / 2 - atan((double)(1.0 / board_line_height)) + aInstance->theta_Init);
	    float t1 = (float)(aInstance->camera_height / sin((double)theta) / sqrt((double)(1 + board_line_height*board_line_height)));

	    aInstance->estimate_distance = aInstance->camera_height / tan((double)theta);
	    aInstance->estimate_objwidth = t1*board_line_width;
	
	    return 1;
    }else
    {

	    return 0;
    }
}
detector_t detector;
void yolo_init_test(char *cfgfile,char *weifile)
{
	detector=YOLO_Init(cfgfile,weifile);
}
int yolo_detect_test(char *input,ObsDetInstance *aInstance)
{
	float cameraArray[9] = { 1.1234784457098569e+003f, 0.f, 6.4473815457954368e+002f, 0.f,
		1.1200599349358272e+003f, 4.7556164629827435e+002f, 0.f, 0.f, 1.f };//camera 内参
	float camera_h = 5.0;//camera height (mm)
	float theta0 = (float)0.0;
        float thresh=0.15;

        image im = load_image_color(input,0,0);

        bbox_v bbox_info=YOLO_Process(detector,im,thresh);

        float ROIh=(1.0 / 3);
        float ROIw=(3.0 / 4);
        float Img_width=im.w;
        float Img_height=im.h;
	aInstance[0]=ObsDetInstance_Init(camera_h,theta0,cameraArray,Img_width,Img_height,ROIh,ROIw);
	int isObsDet=ObsDetInstance_Process(aInstance,bbox_info);
        if(isObsDet)
	{
	    printf("distance:%f,objwidth:%f\n",aInstance[0].estimate_distance,aInstance[0].estimate_objwidth);
		free_image(im);
		return (int)1;
	}
	else
	{
	    printf("No ObsDet\n");
		free_image(im);
		return (int)0;
	}        
}
*/


int main(int argc, char **argv)
{

	char* cfgfile = argv[1];
	char* weifile = argv[2];
        char* filename =argv[3];
        //char* datacfg= argv[4];        

        char buff[256];
        char *input = buff;       
        if(filename){
            strncpy(input, filename, 256);
        } else {
            printf("Enter Image Path: ");
            fflush(stdout);
            input = fgets(input, 256, stdin);
            if(!input) return 0;
            strtok(input, "\n");
        }       

	float cameraArray[9] = { 1.1234784457098569e+003f, 0.f, 6.4473815457954368e+002f, 0.f,
		1.1200599349358272e+003f, 4.7556164629827435e+002f, 0.f, 0.f, 1.f };//camera 内参
	float camera_h = 5.0;//camera height (mm)
	float theta0 = (float)0.0;
        float thresh=0.15;

        detector_t detector=YOLO_Init(cfgfile,weifile);

        image im = load_image_color(input,0,0);

        
        bbox_v bbox_info=YOLO_Process(detector,im,thresh);
        /*int i;
        for(i=0;i<bbox_info.bbox_count;i++)
        {
               printf("x,y,w,h:(%d,%d,%d,%d),objid:%d,prob:%f\n",bbox_info.bboxptr[i].x,bbox_info.bboxptr[i].y,bbox_info.bboxptr[i].w,bbox_info.bboxptr[i].h,bbox_info.bboxptr[i].obj_id,bbox_info.bboxptr[i].prob);
        }*/

        float ROIh=(1.0 / 3);
        float ROIw=(3.0 / 4);
        float Img_width=im.w;
        float Img_height=im.h;
	ObsDetInstance aInstance=ObsDetInstance_Init(camera_h,theta0,cameraArray,Img_width,Img_height,ROIh,ROIw);
	int isObsDet=ObsDetInstance_Process(&aInstance,bbox_info);
        if(isObsDet)
	    printf("distance:%f,objwidth:%f\n",aInstance.estimate_distance,aInstance.estimate_objwidth);
	else
	    printf("No ObsDet\n");

	save_image(im, "predictions");	

        free_image(im);
        YOLO_Clear(&detector);
	return (int)1;
}
