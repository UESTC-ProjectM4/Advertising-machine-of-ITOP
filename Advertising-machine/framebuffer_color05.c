#include <stdio.h>
#include <stdlib.h>    
#include <string.h>    
#include <assert.h>    

#include <getopt.h>             /* getopt_long() */    

#include <fcntl.h>              /* low-level i/o */    
#include <unistd.h>             /*getpid()*/
#include <error.h>    
#include <errno.h>
#include <malloc.h>    
#include <sys/stat.h>    
#include <sys/types.h>    
#include <sys/time.h>    
#include <sys/mman.h>    
#include <sys/ioctl.h>
#include <linux/fb.h> 
#include "ASC16.h" 
#include "HZK16.h"
#include "ASC32.h" 
#include "HZK32.h"

#include <termios.h>
#include <errno.h>




//#include "x5.h"
#include "img2_5.h"
#include "img1.h"
#include "img6_9.h"
unsigned int * lcd_buffer = NULL;
__u8 *fb_buf;
__u32 screensize;
int cursor_x,cursor_y;
void  draw8x16 (int x,int y,unsigned char ch[], int color);  
void  draw16x32 (int x,int y,unsigned char ch[], int color);

void  draw16x16 (int x,int y,unsigned char ch[], int color);
void  draw32x32 (int x,int y,unsigned char ch[], int color);

void  show(int x, int y,unsigned char str[], int len, unsigned char flag,int color);

void  multi_screen();

void  showImage(unsigned char image[],int posx, int posy,int width,int height);


//uart-sound
typedef unsigned char uint8_t;
/**************芯片设置命令*********************/
uint8_t SYN_StopCom[]={0xFD,0X00,0X02,0X02,0XFD};//停止合成
uint8_t SYN_SuspendCom[]={0XFD,0X00,0X02,0X03,0XFC};//暂停合成
uint8_t SYN_RecoverCom[]={0XFD,0X00,0X02,0X04,0XFB};//恢复合成
uint8_t SYN_ChackCom[]={0XFD,0X00,0X02,0X21,0XDE};//状态查�?
uint8_t SYN_PowerDownCom[]={0XFD,0X00,0X02,0X88,0X77};//进入POWER DOWN 状态命�?
int Yyfd;
int set_opt(int,int,int,char,int);
void SYN_FrameInfo(uint8_t Music,uint8_t *HZdata);



int main(int argc,char *argv[])
{
   int i,fd,fbfd,txfd;
   struct fb_var_screeninfo vinfo;
   struct fb_fix_screeninfo finfo;
   
   int fb_xres,fb_yres,fb_bpp;//bits per pixel
   
   unsigned char data[60];
   unsigned char str[60];
   int  len;
   int  n;
   
   


   fbfd=open("/dev/fb0",O_RDWR);
   if(fbfd<0)
   {
     printf("Error:cannot open framebuffer device!\n");
	 close(txfd);
     return -1;
   }
   //get fb_fix_screeninfo
   if(ioctl(fbfd,FBIOGET_FSCREENINFO,&finfo))
   {
     printf("Error reading fixed information!\n");
     return -1;
   }
  //get fb_var_screeninfo
   if(ioctl(fbfd,FBIOGET_VSCREENINFO,&vinfo))
   {
     printf("Error reading variable information!\n");
     return -1;
   }
   printf("%dx,%dy,%dbpp\n",vinfo.xres,vinfo.yres,vinfo.bits_per_pixel);
   fb_xres=vinfo.xres;
   fb_yres=vinfo.yres;
   fb_bpp=vinfo.bits_per_pixel;
  
   screensize=vinfo.xres*vinfo.yres*vinfo.bits_per_pixel/8;
   fb_buf=(char *)mmap(0,screensize,PROT_READ|PROT_WRITE,MAP_SHARED,fbfd,0);
   if((int)(fb_buf)==-1)
   {
      printf("Error:failed to map framebuffer device to memory!\n");
     // close(fbfd);
      return -1;
   }  
  memset(fb_buf,0,screensize); 
    
  lcd_buffer = (unsigned int *)fb_buf;


   //uart_sound

   txfd = open("1.txt",O_RDONLY);
   if(txfd<0)
   {
     printf("Error:cannot open framebuffer device!\n");
     return -1;
   }
   unsigned char Yydata[100];
   len = read(txfd,Yydata,50);

   int nByte;
   char *uart3 = "/dev/ttySAC3";
   char buffer[512];
   char *uart_out = "please input\r\n";
   printf("Ready to Init Uart3!");
   if((Yyfd = open(uart3, O_RDWR|O_NOCTTY))<0)
		printf("open %s is failed!",uart3);
	else{
                  printf("open %s is successfully!",uart3);
		  set_opt(Yyfd, 9600, 8, 'N', 1);
            }
  //printf("%s\n",Yydata);
  while(1)
  {
      
      sleep(1);
      unsigned char data[60]="��е����S2";
      show(160, 104,data, 60,1,0xC0C0C0);  
      sleep(1);
      unsigned char str[60]="MECHREV-�廪ͬ��";
      show(112, 136,str, 60,1,0xC0C0C0); 
      sleep(5);
      memset(fb_buf,0,screensize); 

      SYN_FrameInfo(2,Yydata);


      //big picture show
      showImage(image01,0,0,480,272);
      sleep(8);
      memset(fb_buf,0,screensize); 

      showImage(image8,0,0,480,272);
      sleep(8);
      memset(fb_buf,0,screensize); 

      //four picture show
      showImage(image0,0,0,240,136);
      sleep(1);  
      showImage(image1,240,0,240,136);
      sleep(1);
      showImage(image2,0,136,240,136);
      sleep(1);
      showImage(image3,240,136,240,136);
      sleep(2);
      memset(fb_buf,0,screensize); 
   
      //show the small first picture
      showImage(image0,120,0,240,136);
      unsigned char data0[60]="����Խ��  ȫƾʮ��";
	show(96, 150,data0, 60,1,0xC0C0C0);  
      unsigned char data01[50]="10������ͷ����ǿ��";
	  show(80, 200,data01,40,1,0xC0C0C0);  
      sleep(5);
      memset(fb_buf,0,screensize);

      //big picture show
      showImage(image6,0,0,480,272);
      sleep(5);
      memset(fb_buf,0,screensize); 

      //show the small second picture 
      unsigned char data1[60]="14Ӣ�������170�ȵ��ֿ���";
      show(40, 150,data1, 60,1,0xC0C0C0);    
      unsigned char data10[50]="��������  ʵ��ɿ�";
      show(96, 200,data10, 40,1,0xC0C0C0);  
      showImage(image1,120,0,240,136);  
      sleep(5);
      memset(fb_buf,0,screensize); 


     // big picture show
      showImage(image7,0,0,480,272);
      sleep(5);
      memset(fb_buf,0,screensize); 

      //show the small third picture
      showImage(image3,120,0,240,136);	  
      unsigned char data30[60]="��������";
      show(176, 150,data30, 60,1,0xC0C0C0);
      unsigned char data3[60]="512G��̬   81.4%����ռ��";
      show(48, 200,data3, 50,1,0xC0C0C0); 
      sleep(5);
      memset(fb_buf,0,screensize);
 
      //big picture show
      //showImage(image9,0,0,480,272);
      //sleep(5);
      //memset(fb_buf,0,screensize); 


      

   
      //show the smalll four picture
      showImage(image2,120,0,240,136);
      unsigned char data2[60]="�Ա����� ��е����S2 ��ӭѡ��";
      show(24, 150,data2, 60,1,0xC0C0C0); 

      unsigned char data20[50]="�ٷ��콢  ��һ��ʮ";
      show(96, 200,data20, 40,1,0xC0C0C0); 

      sleep(5);
      memset(fb_buf,0,screensize); 
  }


  printf("ummap framebuffer device to memory!\n");
  //sleep(5);
  munmap(fb_buf,screensize);
  close(fbfd);
  close(txfd);
  return 0;
 }  
   
void  draw8x16 (int x,int y,unsigned char ch[], int color)
{
	int i,j,index =0;
	for (i=0;i <16;i++,index++)
	{
		for (j=00;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+j)] = 0x00;
			}
			
		}
	}	
}
void  draw16x32 (int x,int y,unsigned char ch[], int color){
	int i,j,index =0;
	for (i=0;i <32;i++)
	{
		for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+j)] = 0x00;
			}
			
		}
	   index++;
	   for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+8+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+8+j)] = 0x00;
			}
			
		}
	  index++;	
	}	
}
void  draw16x16 (int x,int y,unsigned char ch[], int color)
{
	int i,j,index =0;
	for (i=0;i <16;i++)
	{
		for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+j)] = 0x00;
			}
			
		}
	   index++;
	   for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+8+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+8+j)] = 0x00;
			}
			
		}
	  index++;	
	}	
}

void  draw32x32 (int x,int y,unsigned char ch[], int color)
{
	int i,j,index =0;
	for (i=0;i <32;i++)
	{
		for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+j)] = 0x00;
			}
			
		}
	   index++;
	   for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+8+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+8+j)] = 0x00;
			}
			
		}
	  index++;	  
	  for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+16+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+16+j)] = 0x00;
			}
			
		}
	  index++;
      for (j=0;j<8;j++)
		{
			if (ch[index] & (1<<(7-j)))
			{
				lcd_buffer[(y+i)*480+(x+24+j)] =  color;
			}else{
				lcd_buffer[(y+i)*480+(x+24+j)] = 0x00;
			}
			
		}
	  index++;		  
	}	
	
}

void  show(int x, int y,unsigned char str[], int len, unsigned char flag,int color)
{
	long index1;
	cursor_x = x;
	cursor_y = y;
	int n;
	for (n=0;n<len;)
  { 
   if(str[n]>160)
    {
	   index1 = (str[n]-161)*94+(str[n+1]-161);
	   if(flag)
	   {
		   if(!(cursor_x<448))
		   {
			  cursor_x = 0;
 			  cursor_y += 32;
		   }
		   index1 *=128;
		   draw32x32(cursor_x,cursor_y,&szHZK32[index1],color);
		   cursor_x +=32;
	   }else{
		   if(!(cursor_x<464))
		   {
			  cursor_x = 0;
 			  cursor_y += 16;
		   }
		   index1 *=32;
		   draw16x16(cursor_x,cursor_y,&szHZK16[index1],color);
		   cursor_x +=16;
	   }
	   n+=2;
    }else{
		index1 = str[n];
		if (flag)
		{
			 if(!(cursor_x<464))
		   {
			  cursor_x = 0;
 			  cursor_y += 32;
		   }
			index1 *= 64;
			draw16x32(cursor_x,cursor_y,&szASC32[index1],color);
            cursor_x +=16;			
		}else{
			 if(!(cursor_x<472))
		   {
			  cursor_x = 0;
 			  cursor_y += 16;
		   }
			index1 *= 16;
			draw8x16(cursor_x,cursor_y,&szASC16[index1],color);
			cursor_x +=8;
		}
		n++;
	}
  	
  }  
	
}
void  multi_screen()
{
  int i,j;
  for (i=0;i< 136;i++)
  {
	 for (j=0;j<240;j++) 
	 {
		lcd_buffer[i*480+j] = 0xff0000;
	 }
	 for(j=240;j<480;j++)
	 {
		lcd_buffer[i*480+j] = 0x00ff00; 
	 }
	  
  }
  
  for (i=136;i< 272;i++)
  {
	 for (j=0;j<240;j++) 
	 {
	   lcd_buffer[i*480+j] = 0x0000ff; 
	 }
	 for(j=240;j<480;j++)
	 {
		lcd_buffer[i*480+j] = 0xff00ff; 
	 }
	  
  }
	
}
void  showImage(unsigned char image[],int x, int y,int width,int height)
{
  int i,j,index =0;
  for (i=0;i <height;i++)
	{
		for (j=0;j<width;j++)
		{		
	    lcd_buffer[(y+i)*480+(x+j)] =  image[index *3]|(image[index *3 +1] << 8)|(image[index *3 +2] << 16);
		index++;			
		}
	}
	
}



int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) { 
		perror("SetupSerial 1");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	
	newtio.c_cflag  |=  CLOCAL | CREAD;
	
	newtio.c_cflag &= ~CSIZE;

	switch( nBits )	
	{
		case 5:
		      newtio.c_cflag |=CS5;
			  break; 
		case 6:
		      newtio.c_cflag |=CS6;
			  break; 
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}

	switch( nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': 
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  
		newtio.c_cflag &= ~PARENB;
		newtio.c_iflag &= ~INPCK;
		newtio.c_iflag &= ~ISTRIP;
		break;
	}

	switch( nSpeed )
	{
		case 2400:
			cfsetispeed(&newtio, B2400);
			cfsetospeed(&newtio, B2400);
			break;
		case 4800:
			cfsetispeed(&newtio, B4800);
			cfsetospeed(&newtio, B4800);
			break;
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
		case 460800:
			cfsetispeed(&newtio, B460800);
			cfsetospeed(&newtio, B460800);
			break;
		default:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
	}
	if( nStop == 1 )
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )
		newtio.c_cflag |=  CSTOPB;
		
		newtio.c_cc[VTIME]  = 0;
		newtio.c_cc[VMIN] = 0;
		
		tcflush(fd,TCIOFLUSH);
		
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
	
	//	printf("set done!\n\r");
	return 0;
}



void SYN_FrameInfo(uint8_t Music,uint8_t *HZdata)
{
/****************需要发送的文本**********************************/ 
		 unsigned  char  Frame_Info[50];
         unsigned  char  HZ_Length;  
		 unsigned  char  ecc  = 0;  			//定义校验字节
	     unsigned  int i=0; 
		 HZ_Length =strlen(HZdata); 			//需要发送文本的长度
 
/*****************帧固定配置信�?*************************************/           
		 Frame_Info[0] = 0xFD ; 			//构造帧头FD
		 Frame_Info[1] = 0x00 ; 			//构造数据区长度的高字节
		 Frame_Info[2] = HZ_Length + 3; 		//构造数据区长度的低字节
		 Frame_Info[3] = 0x01 ; 			//构造命令字：合成播放命�?	 		 
		 Frame_Info[4] = 0x01 | Music<<4 ;  //构造命令参数：背景音乐设定

/*******************校验码计�?**************************************/		 
		 for(i = 0; i<5; i++)   				//依次发送构造好�?个帧头字�?
	     {  
	         ecc=ecc^(Frame_Info[i]);		//对发送的字节进行异或校验	
	     }

	   	 for(i= 0; i<HZ_Length; i++)   		//依次发送待合成的文本数�?
	     {  
	         ecc=ecc^(HZdata[i]); 				//对发送的字节进行异或校验		
	     }		 
/*******************发送帧信息***************************************/		  
		  memcpy(&Frame_Info[5], HZdata, HZ_Length);
		  Frame_Info[5+HZ_Length]=ecc;
		  write(Yyfd,Frame_Info,5+HZ_Length+1);
}





