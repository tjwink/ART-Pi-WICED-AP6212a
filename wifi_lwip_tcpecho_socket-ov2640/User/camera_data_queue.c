#include "camera_data_queue.h"
#include "./sdram/bsp_sdram.h" 
#include "wifi_base_config.h"
#include "debug.h"

#define camera_data_queue_log(M, ...) custom_log("camera_data_queue", M, ##__VA_ARGS__)

camera_data  cam_data[CAMERA_QUEUE_NUM];
CircularBuffer cam_circular_buff;

//队列缓冲区的内存池0xD0000000
__attribute__((at(0xD0000000)))__align(4) uint8_t queue_buff[CAMERA_QUEUE_NUM][CAMERA_QUEUE_DATA_LEN];


int32_t find_jpeg_tail(uint8_t *data,uint8_t *jpeg_start,int32_t search_point) ;
void camera_queue_free(void);
int32_t camera_queue_init(void);
int  find_usable_index(void);
int32_t push_data_to_queue(void);
int32_t pull_data_from_queue(uint8_t **data_p, int32_t *len_p);
void clear_array_flag(uint8_t index);




/*环形缓冲队列*/

/**
  * @brief  初始化缓冲队列
  * @param  cb:缓冲队列结构体
  * @param  size: 缓冲队列的元素个数
  * @note 	初始化时还需要给cb->elems指针赋值
  */
void cbInit(CircularBuffer *cb, int size) 
{
    cb->size  = size;	/* maximum number of elements           */
    cb->start = 0; 		/* index of oldest element              */
    cb->end   = 0; 	 	/* index at which to write new element  */
//    cb->elems = (uint8_t *)calloc(cb->size, sizeof(uint8_t));  //elems 要额外初始化
}
 
/**
  * @brief  输出缓冲队列当前的状态信息
  * @param  cb:缓冲队列结构体
  */
void cbPrint(CircularBuffer *cb) 
{
    printf("size=0x%x, start=%d, end=%d\n", cb->size, cb->start, cb->end);
	  printf("size=0x%x, start_using=%d, end_using=%d\n", cb->size, cb->start_using, cb->end_using);
}
 
/**
  * @brief  判断缓冲队列是(1)否(0)已满
  * @param  cb:缓冲队列结构体
  */
int cbIsFull(CircularBuffer *cb) 
{
    return cb->end == (cb->start ^ cb->size); /* This inverts the most significant bit of start before comparison */ 
}
 
/**
  * @brief  判断缓冲队列是(1)否(0)全空
  * @param  cb:缓冲队列结构体
  */		
int cbIsEmpty(CircularBuffer *cb) 
{
    return cb->end == cb->start; 
}

/**
  * @brief  对缓冲队列的指针加1
  * @param  cb:缓冲队列结构体
  * @param  p：要加1的指针
  * @return  返回加1的结果
  */	
int cbIncr(CircularBuffer *cb, int p) 
{
    return (p + 1)&(2*cb->size-1); /* start and end pointers incrementation is done modulo 2*size */
}
 
/**
  * @brief  获取可写入的缓冲区指针
  * @param  cb:缓冲队列结构体
  * @return  可进行写入的缓冲区指针
  * @note  得到指针后可进入写入操作，但写指针不会立即加1，写完数据时，应调用cbWriteFinish
  */
camera_data* cbWrite(CircularBuffer *cb) 
{
    if (cbIsFull(cb)) /* full, overwrite moves start pointer */
    {
			return NULL;
		}		
		else
			cb->end_using = cbIncr(cb, cb->end); //未满，则增加1
		
	return  cb->elems[cb->end_using&(cb->size-1)];
}

/**
  * @brief  获取可当前正在使用的写缓冲区指针
  * @param  cb:缓冲队列结构体
  * @return  当前正在使用的写缓冲区指针
  */
camera_data* cbWriteUsing(CircularBuffer *cb) 
{
	return  cb->elems[cb->end_using&(cb->size-1)];
}

/**
  * @brief 数据写入完毕，更新写指针到缓冲结构体
  * @param  cb:缓冲队列结构体
  */
void cbWriteFinish(CircularBuffer *cb)
{
    cb->end = cb->end_using;
}
 
/**
  * @brief  获取可读取的缓冲区指针
  * @param  cb:缓冲队列结构体
  * @return  可进行读取的缓冲区指针
  * @note  得到指针后可进入读取操作，但读指针不会立即加1，读取完数据时，应调用cbReadFinish
  */
camera_data* cbRead(CircularBuffer *cb) 
{
		if(cbIsEmpty(cb))
			return NULL;
		
	cb->start_using = cbIncr(cb, cb->start);
	return cb->elems[cb->start_using&(cb->size-1)];
}


/**
  * @brief 数据读取完毕，更新读指针到缓冲结构体
  * @param  cb:缓冲队列结构体
  */
void cbReadFinish(CircularBuffer *cb) 
{
    cb->start = cb->start_using;
}



#include "stdlib.h"


//摄像头队列的指针指向的缓冲区全部销毁
void camera_queue_free(void)
{
    uint32_t i = 0;

    for(i = 0; i < CAMERA_QUEUE_NUM; i ++)
    {
        if(cam_data[i].head != NULL)
        {
            free(cam_data[i].head);
            cam_data[i].head = NULL;
        }
    }

    return;
}

//缓冲队列初始化，分配内存
int32_t camera_queue_init(void)
{
  uint32_t i = 0;

  memset(cam_data, 0, sizeof(cam_data));
		
	/*初始化缓冲队列*/
	cbInit(&cam_circular_buff,CAMERA_QUEUE_NUM);

    for(i = 0; i < CAMERA_QUEUE_NUM; i ++)
    {
        cam_data[i].head = queue_buff[i];//cam_global_buf+CAMERA_QUEUE_DATA_LEN*i;
        
        /*初始化队列缓冲指针，指向实际的内存*/
        cam_circular_buff.elems[i] = &cam_data[i];
        
        printf("cam_data[i].head=0x%x,cam_circular_buff.elems[i] =0x%x\r\n", (uint32_t)cam_data[i].head,(uint32_t)cam_circular_buff.elems[i]->head);

        memset(cam_data[i].head, 0, CAMERA_QUEUE_DATA_LEN);
    }

    return kNoErr;
}




extern int send_fream;
//从队列中取数据, data_p:起始地址, len_p:长度, array_index:下标
int32_t pull_data_from_queue(uint8_t **data_p, int32_t *len_p)
{
//		int32_t len_p_get = 0 ;		
		uint8_t jpeg_start_offset = 0;	
		camera_data *cam_data_pull;	
		//uint32_t time_old,time_new;
		
		if(!cbIsEmpty(&cam_circular_buff))//缓冲队列非空
		{
				/*从缓冲区读取数据，进行处理，*/
				cam_data_pull = cbRead(&cam_circular_buff);
			
				if (cam_data_pull == NULL)
						return kGeneralErr;
			
			/*查找文件*/	
				
				*len_p = find_jpeg_tail(cam_data_pull->head,&jpeg_start_offset,cam_data_pull->img_dma_len*2/3);


				if(*len_p != -1)
				{
						send_fream++;
						*data_p = cam_data_pull->head + jpeg_start_offset;
						return kNoErr;
				}else
				{
				}		
		}

    return kGeneralErr;
}

 


/**
  * @brief  查找jpeg文件头和文件尾
  * @param  data:数据区指针
	* @param  jpeg_start:返回jpeg文件头在数据区的偏移
	* @param  search_point:查找jpeg文件尾时，在数据区的搜索起始位置
  */

#define JPEG_HEAD_SEARCH_MAX 	10000

int32_t find_jpeg_tail(uint8_t *data,uint8_t *jpeg_start,int32_t search_point) 
{
    uint32_t i = 0,j=0;//,z=0;
    uint8_t *p = data;
	
	/*默认文件头位置*/
	*jpeg_start = 0;

	for(j = 0;j<JPEG_HEAD_SEARCH_MAX;j++)
	{
		if(! ((*p == 0xFF) && (*(p + 1) == 0xD8)) ) //若不是帧头
		{
			p++;
		}
    else if((*p == 0xFF) && (*(p + 1) == 0xD8)) //帧头判断
    {
//			printf("jpeg帧头->0xFF--0xD8\r\n");
			if(j!=0)//如果内存起始不是照片头，就将指针偏移
			{
				printf("JPEG_HEAD_SEARCH_MAX *p=0x%x,j=%d\r\n", *p,j );
				/*文件头实际偏移*/
				*jpeg_start=j;
			}

				/*搜索起点*/
				p += search_point;
			
        for(i = search_point ; i < CAMERA_QUEUE_DATA_LEN - 2; i ++)
        {
            if((*p == 0xFF) && (*(p + 1) == 0xD8))//再次判断
            {
                if(i == 0)
                {
                    p++;
                    continue;
                }else
                {
                    return -1;
                }
            }else if((*p == 0xFF) && (*(p + 1) == 0xD9))
            {//找到图片尾
//								camera_data_queue_log("pic len = %d", i+2 );
								//printf("jpeg帧尾->0xFF--0xD9\r\n");
                return  i + 2;
            }

            p++;
        }  	
				
				printf("picture tail error!\r\n");
				return -1;				
    }
	}

	
  printf("picture head error!\r\n");	
	
  return -1;
}



/**
  * @brief  查找jpeg文件头和文件尾
  * @param  data:数据区指针
	*	@return 正常找到文件：kNoErr，异常:kGeneralErr
  */
int32_t find_jpeg(camera_data *cambuf) 
{
	uint32_t i = 0,j=0;
	
	uint8_t *p = cambuf->head;	
	
	/*搜索文件尾起点*/
	uint32_t end_search_point = cambuf->img_dma_len*99/100;
	
	/*重置*/
	cambuf->img_real_start = NULL;	
	cambuf->img_real_len = -1;

	for(j = 0;j<JPEG_HEAD_SEARCH_MAX;j++)
	{
		if(! ((*p == 0xFF) && (*(p + 1) == 0xD8)) ) //若不是帧头
		{
			p++;
		}
    else if((*p == 0xFF) && (*(p + 1) == 0xD8)) //帧头判断
    {
			if(j!=0)			
			{	
				/*文件头实际偏移*/
				printf("JPEG_HEAD_SEARCH *p=0x%x,j=%d\r\n", *p,j );
			}
			
				/*文件头实际指针*/
				cambuf->img_real_start = p;

				/*文件尾搜索起点*/
				p += end_search_point;
			
        for(i = end_search_point ; i < CAMERA_QUEUE_DATA_LEN - 2; i ++)
        {
            if((*p == 0xFF) && (*(p + 1) == 0xD8))
            {
                if(i == 0)
                {
                    p++;
                    continue;
                }else
                {
                    return kGeneralErr;
                }
            }else if((*p == 0xFF) && (*(p + 1) == 0xD9))
            {
								//camera_data_queue_log("pic len = %d", i+2 );

								cambuf->img_real_len = i+2;
                return  kNoErr;
            }

            p++;
        }  	
				
				printf("picture tail error!\r\n");
				return kGeneralErr;				
    }
	}

	
  printf("picture head error!\r\n");	
	
  return kGeneralErr;
}





