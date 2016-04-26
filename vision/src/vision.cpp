#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision/x264Packet.h>

extern "C"
{
	#include <libavcodec/avcodec.h>
  #include <libavformat/avio.h>
  #include <libavformat/avformat.h>
  #include <libswscale/swscale.h>
  #include <libavutil/opt.h>
  #include <libavutil/imgutils.h>
  #include <libavutil/samplefmt.h>
}

int flush_encoder(AVFormatContext *fmt_ctx,unsigned int stream_index){
    int ret;
    int got_frame;
    AVPacket enc_pkt;
    if (!(fmt_ctx->streams[stream_index]->codec->codec->capabilities &
        CODEC_CAP_DELAY))
        return 0;
    while (1) {
        enc_pkt.data = NULL;
        enc_pkt.size = 0;
        av_init_packet(&enc_pkt);
        ret = avcodec_encode_video2 (fmt_ctx->streams[stream_index]->codec, &enc_pkt,
            NULL, &got_frame);
        av_frame_free(NULL);
        if (ret < 0)
            break;
        if (!got_frame){
            ret=0;
            break;
        }
        printf("Flush Encoder: Succeed to encode 1 frame!\tsize:%5d\n",enc_pkt.size);
        /* mux encoded frame */
        ret = av_write_frame(fmt_ctx, &enc_pkt);
        if (ret < 0)
            break;
    }
    return ret;
}

unsigned char clip_value(unsigned char x,unsigned char min_val,unsigned char  max_val){
    if(x>max_val){
        return max_val;
    }else if(x<min_val){
        return min_val;
    }else{
        return x;
    }
}

//RGB to YUV420
bool RGB24_TO_YUV420(unsigned char *RgbBuf,int w,int h,unsigned char *yuvBuf)
{
    unsigned char*ptrY, *ptrU, *ptrV, *ptrRGB;
    memset(yuvBuf,0,w*h*3/2);
    ptrY = yuvBuf;
    ptrU = yuvBuf + w*h;
    ptrV = ptrU + (w*h*1/4);
    unsigned char y, u, v, r, g, b;
    for (int j = 0; j<h;j++){
        ptrRGB = RgbBuf + w*j*3 ;
        for (int i = 0;i<w;i++){

            r = *(ptrRGB++);
            g = *(ptrRGB++);
            b = *(ptrRGB++);
            y = (unsigned char)( ( 66 * r + 129 * g +  25 * b + 128) >> 8) + 16  ;
            u = (unsigned char)( ( -38 * r -  74 * g + 112 * b + 128) >> 8) + 128 ;
            v = (unsigned char)( ( 112 * r -  94 * g -  18 * b + 128) >> 8) + 128 ;
            *(ptrY++) = clip_value(y,0,255);
            if (j%2==0&&i%2 ==0){
                *(ptrU++) =clip_value(u,0,255);
            }
            else{
                if (i%2==0){
                *(ptrV++) =clip_value(v,0,255);
                }
            }
        }
    }
    return true;
}

AVFormatContext* pFormatCtx;  //格式串
AVOutputFormat* fmt;          //輸出格式
AVStream* video_st;           //Stream
AVCodecContext* pCodecCtx;    //編碼解碼器
AVCodec* pCodec;
AVPacket pkt;
uint8_t* picture_buf;         //buffer 緩衝
uint8_t* YUV_buf;
AVFrame* pFrame;
int picture_size;
int y_size;
int framecnt;
int framenum;

int in_w,in_h;                              //Input data's width and height
sensor_msgs::Image current_rgb_msg;
unsigned char *pic_yuv420;
unsigned char *pic_rgb24;
int all;
bool flag;
int framecount;
int a;

void encoderYUV2H264(){
  const char* out_file = "ds.h264";

  av_register_all();

  pFormatCtx = avformat_alloc_context();	//AVFormatContext 初始化

  fmt = av_guess_format(NULL, out_file, NULL);
  pFormatCtx->oformat = fmt;

  if (avio_open(&pFormatCtx->pb,out_file, AVIO_FLAG_READ_WRITE) < 0){
      printf("Failed to open output file! \n");
  }

  video_st = avformat_new_stream(pFormatCtx, 0);		//AVStream 初始化

  if (video_st==NULL){
  }
  //Param that must set
  pCodecCtx = video_st->codec;
  //pCodecCtx->codec_id =AV_CODEC_ID_HEVC;
  pCodecCtx->codec_id = fmt->video_codec;
  pCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
  pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  pCodecCtx->width = in_w;
  pCodecCtx->height = in_h;
  pCodecCtx->bit_rate = 512000;
  pCodecCtx->gop_size=250;

	//time_base 設定幀數 fp/s
  pCodecCtx->time_base.num = 1;					//numerator 分母
	//pCodecCtx->time_base.den = 25;
  pCodecCtx->time_base.den = 30;				//denominator 分子

  //pCodecCtx->me_range = 16;
  //pCodecCtx->max_qdiff = 4;
  //pCodecCtx->qcompress = 0.6;
  pCodecCtx->qmin = 1;
  pCodecCtx->qmax = 51;

  //Optional Param				兩張非B幀中間最大B幀的數量
  pCodecCtx->max_b_frames=2;

  // Set Option
  AVDictionary *param = 0;
  //H.264
  if(pCodecCtx->codec_id == AV_CODEC_ID_H264) {
      av_dict_set(&param, "preset", "slow", 0);
      av_dict_set(&param, "tune", "zerolatency", 0);
      //av_dict_set(&param, "profile", "main", 0);
  }
  //H.265
  if(pCodecCtx->codec_id == AV_CODEC_ID_H265){
      av_dict_set(&param, "preset", "ultrafast", 0);
      av_dict_set(&param, "tune", "zero-latency", 0);
  }

  //Show some Information
  av_dump_format(pFormatCtx, 0, out_file, 1);

  pCodec = avcodec_find_encoder(pCodecCtx->codec_id);
  if (!pCodec){
      printf("Can not find encoder! \n");
  }
  if (avcodec_open2(pCodecCtx, pCodec,&param) < 0){
      printf("Failed to open encoder! \n");
  }


  pFrame = av_frame_alloc();
  picture_size = avpicture_get_size(pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height);
  //picture_buf = (uint8_t *)av_malloc(picture_size);
  avpicture_fill((AVPicture *)pFrame, picture_buf, pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height);

  //Write File Header
  avformat_write_header(pFormatCtx,NULL);

  av_new_packet(&pkt,picture_size);
  y_size = pCodecCtx->width * pCodecCtx->height;

	vision::x264Packet packet;

  for (int i=0; i<framenum; i++){
      //Read raw YUV data
			picture_buf = YUV_buf + in_w*in_h*3/2*i;

      pFrame->data[0] = picture_buf;              // Y
      pFrame->data[1] = picture_buf+ y_size;      // U
      pFrame->data[2] = picture_buf+ y_size*5/4;  // V

      ROS_INFO("----buf_Y---- : %d",*(picture_buf+0));
      //PTS
      //pFrame->pts=i;
      pFrame->pts=i*(video_st->time_base.den)/((video_st->time_base.num)*25);
      int got_picture=0;
      //Encode

      int ret = avcodec_encode_video2(pCodecCtx, &pkt , pFrame, &got_picture);

			//Set data
		/*	packet.data.resize(ret);

			//Set width & height
			packet.img_width = in_w;
			packet.img_height = in_h;

			//copy NAL data
			//memcpy(&packet.data[0],buffer_,ret);

			memcpy(&packet.data,pkt.data,pkt.size);
						//Affect header
			packet.header = current_rgb_msg.header;
			packet.codec = 0;*/

			ROS_INFO("----AVPacket_size----:%d",pkt.size);
			/*if(pkt.data!=NULL){
						for(int i=0;i < pkt.size;i++){
								printf("%x",pkt.data[i]);
						}
			}*/

			//free(&packet.data);

      if(ret < 0){
          printf("Failed to encode! \n");
      }

      if (got_picture==1){
          printf("Succeed to encode frame: %5d\tsize:%5d\n",framecnt,pkt.size);
          framecnt++;
          pkt.stream_index = video_st->index;
          ret = av_write_frame(pFormatCtx, &pkt);
          av_free_packet(&pkt);
      }
  }


  //Flush Encoder
	ROS_INFO("--------------------GO Flush_encoder---------------");
	ROS_INFO("%c",pFormatCtx->filename);
  /*int ret = flush_encoder(pFormatCtx,0);

  if (ret < 0) {
      printf("Flushing encoder failed\n");
  }*/


  //Write file trailer
  av_write_trailer(pFormatCtx);
  //Clean
  if (video_st){
      avcodec_close(video_st->codec);
      av_free(pFrame);
      free(YUV_buf);
  }
  avio_close(pFormatCtx->pb);
  avformat_free_context(pFormatCtx);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  current_rgb_msg = *msg;
  //ROS_INFO("I heard: r:[%d]G:[%d]B:[%d]", current_rgb_msg.data[0],current_rgb_msg.data[1],current_rgb_msg.data[2]);

  /*for(int i=0 ; i<(in_w*in_h) ; i++){
    ROS_INFO("in for frame = %d",i);
  }*/

	in_w=current_rgb_msg.width;
	in_h=current_rgb_msg.height;

	//pic_rgb24 = (unsigned char*)malloc(in_w*in_h*3);

	pic_rgb24 = &current_rgb_msg.data[0];

	unsigned char*ptrY, *ptrU, *ptrV, *ptrRGB;

	pic_yuv420 = (unsigned char*)malloc(in_w*in_h*3/2);

	RGB24_TO_YUV420(pic_rgb24,in_w,in_h,pic_yuv420);

/*	for(int i = 0; i < in_h; i++) {
		for(int j = 0; j < in_w; j++) {
	  	*(pic_rgb24 + i*in_w + j) = current_rgb_msg.data[i*in_w + j];    //R
			*(pic_rgb24 + i*in_w + j + 1) = current_rgb_msg.data[i*in_w + j + 1];		//G
			*(pic_rgb24 + i*in_w + j + 2) = current_rgb_msg.data[i*in_w + j + 2];		//B
		}
	}*/

	/*ROS_INFO("R : %d",*(pic_rgb24 + 0));
	ROS_INFO("G : %d",*(pic_rgb24 + 1));
	ROS_INFO("B : %d",*(pic_rgb24 + 2));
	ROS_INFO("R : %d",*(pic_rgb24 + 3));
	ROS_INFO("G : %d",*(pic_rgb24 + 4));
	ROS_INFO("B : %d",*(pic_rgb24 + 5));

	ROS_INFO("Y1 : %d",*(pic_yuv420 + 0));
	ROS_INFO("Y2 : %d",*(pic_yuv420 + 1));*/

  if(flag==0){
    if(framecount < framenum){

			for(int i = 0; i < in_w*in_h*3/2; i++) {
		  	*(YUV_buf + framecount*in_w*in_h*3/2 + i) = *(pic_yuv420 + i);    //Y
			}
/*
			ROS_INFO("YUV_buf1 : %d",*(YUV_buf + 0));
			ROS_INFO("YUV_buf2 : %d",*(YUV_buf + 1));
			ROS_INFO("YUV_buf2_1 : %d",*(YUV_buf + 1*in_w*in_h*3/2 + 0));
			ROS_INFO("YUV_buf2_2 : %d",*(YUV_buf + 1*in_w*in_h*3/2 + 1));
			ROS_INFO("YUV_buf3_1 : %d",*(YUV_buf + 2*in_w*in_h*3/2 + 0));
			ROS_INFO("YUV_buf3_2 : %d",*(YUV_buf + 2*in_w*in_h*3/2 + 1));
*/
			framecount++;
			ROS_INFO("RGB to YUV frc:%d", framecount);
		}else{
			flag=1;
		}
		if(flag == 1){
			ROS_INFO("-------------in encoder-------------");
	/*
			picture_buf = (uint8_t *)malloc(framecount*in_w*in_h*3/2);

			for (int i=0; i<framenum; i++){
				//Read raw YUV data
				ROS_INFO("in there");
				picture_buf = YUV_buf + in_w*in_h*3/2*i;
				ROS_INFO("picture_buf1 : %d",*(picture_buf + 0));
				ROS_INFO("picture_buf2 : %d",*(picture_buf + 1));
			}*/

			encoderYUV2H264();
			ROS_INFO("in flag = 1 frame = %d",framecount);
		}
	}
  //Read raw YUV data
}

int main(int argc, char** argv){
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

	flag = 0;
  framecount = 0;

  framecnt=0;
  in_w=640,in_h=480;                              //Input data's width and height
  framenum=30;                                   //Frames to encode

	YUV_buf = (uint8_t *)malloc(in_w*in_h*3/2*framenum);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);

  ros::spin();
  return 0;
} // main
