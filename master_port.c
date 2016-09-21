/* $Id: master_port.c 3664 2011-07-19 03:42:28Z nanang $ */
/* 
 * Copyright (C) 2008-2011 Teluu Inc. (http://www.teluu.com)
 * Copyright (C) 2003-2008 Benny Prijono <benny@prijono.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */
#include <pjmedia/master_port.h>
#include <pjmedia/clock.h>
#include <pjmedia/errno.h>
#include <pj/assert.h>
#include <pj/lock.h>
#include <pj/pool.h>
#include <pj/string.h>
#include <hi_inc/sample_comm.h>
#include <hi_inc/hi35xx_audio.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pj/log.h>

#define AUDIO_ONEFRAME_SIZE_C 320
#define THIS_FILE	"master_port.c"

static HI_S32 s32Ret;
#define SAMPLE_AUDIO_PTNUMPERFRM   320
static AIO_ATTR_S stAioAttr;				//定义音频输入输出设备属性结构体

static int sendframebuf[AUDIO_ONEFRAME_SIZE_C];
static int sendframetempbuf[AUDIO_ONEFRAME_SIZE_C];
static HI_S32 call_start=PJ_FALSE;
extern char lance_call_start;
static char count=0;

typedef struct lance_AI_S
{
    HI_BOOL bStart;
    HI_S32  AiDev;
    HI_S32  AiChn;
    HI_S32  AencChn;
    HI_S32  AoDev;
    HI_S32  AoChn;
} pSAMPLE_AI_S;
static pSAMPLE_AI_S gs_stLanceAi[AI_DEV_MAX_NUM*AIO_MAX_CHN_NUM];


/******************************************************************************
* function : vb init & MPI system init
******************************************************************************/
HI_S32 SAMPLE_COMM_SYS_Init(VB_CONF_S *pstVbConf)
{
    MPP_SYS_CONF_S stSysConf = {0};
    HI_S32 s32Ret = HI_FAILURE;

    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    if (NULL == pstVbConf)						//空指针�
    {
        SAMPLE_PRT("input parameter is null, it is invaild!\n");
        return HI_FAILURE;
    }
    s32Ret = HI_MPI_VB_SetConf(pstVbConf);		//设置MPP视频缓冲池属性
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_VB_SetConf failed!\n");
        return HI_FAILURE;
    }
    s32Ret = HI_MPI_VB_Init();						//初始化MPP视频缓冲池  先配置后初始化
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_VB_Init failed!\n");
        return HI_FAILURE;
    }
    stSysConf.u32AlignWidth = SAMPLE_SYS_ALIGN_WIDTH;
    s32Ret = HI_MPI_SYS_SetConf(&stSysConf);		//配置系统控制参数
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_SYS_SetConf failed\n");
        return HI_FAILURE;
    }
    s32Ret = HI_MPI_SYS_Init();						//初始化MPP系统(除音频的编解码通道外)
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_SYS_Init failed!\n");
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

/******************************************************************************
* function : vb exit & MPI system exit
******************************************************************************/
HI_VOID SAMPLE_COMM_SYS_Exit(void)
{
    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();
    return;
}

/******************************************************************************
* function : to process abnormal case
******************************************************************************/
void SAMPLE_AUDIO_HandleSig(HI_S32 signo)
{
    if (SIGINT == signo || SIGTSTP == signo)
    {
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

    exit(0);
}

HI_S32 lance_audio_ai(HI_S32 call_flag)
{
	HI_S32 i;
	HI_S32 chn;
	HI_S32 AiFd;
	fd_set read_fds;
	struct timeval TimeoutVal;
	AI_CHN_PARAM_S stAiChnPara;					//定义通道参数结构体
	AUDIO_STREAM_S stStream;

	memset(sendframebuf,0,320);
	//for(chn=0;chn<CHN_NUM;chn++)
	for(chn=0;chn<1;chn++)
	{
		AiChn = chn;
		if(!lance_call_start)		//呼叫前初始化一次  呼叫后不进入
		{
			//获取AI通道参数
			s32Ret = HI_MPI_AI_GetChnParam(AiDev, AiChn, &stAiChnPara);
			if (HI_SUCCESS != s32Ret)
			{
				printf("%s: Get ai chn param failed,%#x\n", __FUNCTION__,s32Ret);
				return HI_FAILURE;
			}
			
			//设置AI通道参数
			stAiChnPara.u32UsrFrmDepth = 30;				//音频帧缓存深度
			s32Ret = HI_MPI_AI_SetChnParam(AiDev,AiChn, &stAiChnPara); 		
			if (HI_SUCCESS != s32Ret)
			{
				printf("%s: set ai chn param failed,%#x\n", __FUNCTION__,s32Ret);
				return HI_FAILURE;
			}			
		}
		FD_ZERO(&read_fds);
		AiFd = HI_MPI_AI_GetFd(AiDev, AiChn);
		FD_SET(AiFd,&read_fds);
		//while(1)
		while(call_flag)			//传入true
		{
			TimeoutVal.tv_sec = 1;
	        TimeoutVal.tv_usec = 0;
			
	        FD_ZERO(&read_fds);
	        FD_SET(AiFd,&read_fds);
	        //PJ_LOG(4,(THIS_FILE, "....select...."));		//select耗时40ms获取1帧
	        s32Ret = select(AiFd+1, &read_fds, NULL, NULL, &TimeoutVal);			//进行阻塞获取
	        if (s32Ret < 0) 													//负值：select错误
	        {
	            break;
	        }
	        else if (s32Ret == 0) 											//0：等待超时，没有可读写或错误的文件
	        {
	            printf("%s: get ai frame select time out\n", __FUNCTION__);
	            break;
	        }        
	        if (FD_ISSET(AiFd, &read_fds))
			{				
				//PJ_LOG(4,(THIS_FILE, "HI_MPI_AI_GetFrame"));
				//从Ai通道获取音频帧-----需要40ms的时间--8K的情况下
				s32Ret = HI_MPI_AI_GetFrame(AiDev, AiChn,&stFrame, NULL, HI_FALSE);		
				if (HI_SUCCESS != s32Ret )
				{
					//			输入缓存为0--->failed with 0xa015800e!
					printf("%s: HI_MPI_AI_GetFrame(%d, %d), failed with %#x!\n",__FUNCTION__, AiDev, AiChn, s32Ret);
					return HI_FAILURE;
				}
				//stFrame.enSoundmode=0---momo    enBitwidth=1----16bit/sample    stFrame.u32Len=640byte
				{					
					// 8K时  8000点p秒/320点p帧=25帧p秒	--->40ms/帧		--->之前噪声是 30ms下降沿、上升沿  10ms的水平信号
					// 16K时 1600点p秒/320点p帧=50帧p秒	--->20ms/帧
					memcpy(sendframebuf,stFrame.pVirAddr[0],320);
					//printf("strlen=%d \n",strlen(stFrame.pVirAddr[0]));
					call_flag = HI_FALSE;	
				}
				//HI_MPI_AI_ReleaseFrame(AiDev, AiChn,&stFrame, NULL);			//未加上frame释放接口时 下次select返回0出错
			}
		}					 		
	}
	return HI_SUCCESS;
}

/******************************************************************************
* function :	Ai -> Aenc -> file
*				Ai -> Adec -> Ao
* 				Ai -> pjmedia_port_get_frame【从下游端口获取音频帧】
******************************************************************************/
HI_S32 lance_AUDIO_AiAenc(AIO_ATTR_S *pstAioAttr)
{								
	pSAMPLE_AI_S *pstAi = NULL;
	HI_S32 i;
	
    /* config ai aenc dev attr */		//配置AI音频编码属性
    if (pstAioAttr == NULL)					//是否配置?
    {
        printf("[Func]:%s [Line]:%d [Info]:%s\n", __FUNCTION__, __LINE__, "NULL pointer");
        return HI_FAILURE;
    }

    /********************************************
      step 1: config audio codec			配置音频编码--->根据编码类型配置
    ********************************************/
    s32Ret = SAMPLE_COMM_AUDIO_CfgAcodec(pstAioAttr, gs_bMicIn);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_DBG(s32Ret);
        return HI_FAILURE;
    }
    /********************************************
      step 2: start Ai						开始AI输入
    ********************************************/
    s32AiChnCnt = pstAioAttr->u32ChnCnt; 			//输入的音频结构体
   	s32AencChnCnt = s32AiChnCnt;						//8路Aenc通道
    s32Ret = SAMPLE_COMM_AUDIO_StartAi(AiDev, s32AiChnCnt, pstAioAttr,  gs_bAiAnr, gs_pstAiReSmpAttr);
	if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_DBG(s32Ret);
        return HI_FAILURE;
    }
	/********************************************
      step 3: enable AO channle						使能AO通道
    ********************************************/
    pstAioAttr->u32ChnCnt = s32AiChnCnt;			//更新通道数		启用AI设备属性、设备、通道、噪声抑制、重采样
    s32Ret = SAMPLE_COMM_AUDIO_StartAo(AoDev, AoChn, pstAioAttr, gs_pstAoReSmpAttr);
    if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_DBG(s32Ret);
        return HI_FAILURE;
    }

    /********************************************
      step 4: start Aenc						开始编码	已更新通道数为1 ---->创建音频编码通道
    ********************************************/
    s32Ret = SAMPLE_COMM_AUDIO_StartAenc(s32AencChnCnt, gs_enPayloadType);
    if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_DBG(s32Ret);
        return HI_FAILURE;
    }
	for (i=0; i<s32AencChnCnt; i++)		// s32AencChnCnt = 1
    {
        AeChn = i;
        AiChn = i;

    
        s32Ret = SAMPLE_COMM_AUDIO_AencBindAi(AiDev, AiChn, AeChn);		//音频编码绑定AI通道
        if (s32Ret != HI_SUCCESS)
        {
            SAMPLE_DBG(s32Ret);
            return s32Ret;
        }
        printf("Ai(%d,%d) bind to AencChn:%d ok!\n",AiDev , AiChn, AeChn);
    }
    /* bind AI to AO channle */			//绑定AI到AO通道
    if (0)			//当resample and anr时,此判断为真
    {
        s32Ret = SAMPLE_COMM_AUDIO_CreatTrdAiAo(AiDev, AiChn, AoDev, AoChn);
        if (s32Ret != HI_SUCCESS)
        {
            SAMPLE_DBG(s32Ret);
            return HI_FAILURE;
        }
    }
    else
    {   
        s32Ret = SAMPLE_COMM_AUDIO_AoBindAi(AiDev, AiChn, AoDev, AoChn);		//绑定设备号、通道号
        if (s32Ret != HI_SUCCESS)
        {
            SAMPLE_DBG(s32Ret);
            return HI_FAILURE;
        }
    }

	return HI_SUCCESS;
}

/**************************************
*				音频初始化配置
***************************************/
static HI_S32 hi35xx_audio_init(void)
{
	//printf("\nhi35xx_audio_init!!\n\n");
    VB_CONF_S stVbConf;

	//初始化stAio
	stAioAttr.enSamplerate = AUDIO_SAMPLE_RATE_8000;				//采样率
	stAioAttr.enBitwidth = AUDIO_BIT_WIDTH_16;					//采样精度
	stAioAttr.enWorkmode = AIO_MODE_I2S_SLAVE;					//音频输入输出工作模式----->I2S从模式
	stAioAttr.enSoundmode = AUDIO_SOUND_MODE_MONO;				//音频声道模式----->单声道momo
	stAioAttr.u32EXFlag = 1;										// 8bit到16bit扩展标志（8bit精度时有效）
	stAioAttr.u32FrmNum = 30;										//缓存帧数目		
	stAioAttr.u32PtNumPerFrm = 160;//SAMPLE_AUDIO_PTNUMPERFRM;	// 320		 //每帧的采样点个数  8000点/秒 / 320点/帧 = 25帧/秒
	stAioAttr.u32ChnCnt = CHN_NUM;									//支持的最大通道数目
	stAioAttr.u32ClkSel = 0;

	signal(SIGINT, SAMPLE_AUDIO_HandleSig);
	signal(SIGTERM, SAMPLE_AUDIO_HandleSig);

	memset(&stVbConf, 0, sizeof(VB_CONF_S));				// 从stVbConf开始，清零一段长度为VB_CONF_S的空间
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);			//配置初始化MPP视频缓冲池、MPP系统
    if (HI_SUCCESS != s32Ret)
    {
        printf("%s: system init failed with %d!\n", __FUNCTION__, s32Ret);
        return HI_FAILURE;
    }
	
	lance_AUDIO_AiAenc(&stAioAttr);
	
	return HI_SUCCESS;

}

struct pjmedia_master_port
{
    unsigned	     options;
    pjmedia_clock   *clock;
    pjmedia_port    *u_port;
    pjmedia_port    *d_port;
    unsigned	     buff_size;
    void	    *buff;
    pj_lock_t	    *lock;
};


static void clock_callback(const pj_timestamp *ts, void *user_data);


/*
 * Create a master port.		创建主机端口
 *
 */
PJ_DEF(pj_status_t) pjmedia_master_port_create( pj_pool_t *pool,		/* 缓冲池   pjsua_var.snd_pool		*/
						pjmedia_port *u_port,										/* 上流端口 pjsua_var.null_port		*/
						pjmedia_port *d_port,										/* 下流端口 conf_port					*/
						unsigned options,											/* 0										*/
						pjmedia_master_port **p_m)							/* 主机端口 &pjsua_var.null_snd		*/
{
    pjmedia_master_port *m;
    unsigned clock_rate;
    unsigned channel_count;
    unsigned samples_per_frame;
    unsigned bytes_per_frame;
    pjmedia_audio_format_detail *u_afd, *d_afd;
    pj_status_t status;	
	

    /* Sanity check */
    PJ_ASSERT_RETURN(pool && u_port && d_port && p_m, PJ_EINVAL);

    u_afd = pjmedia_format_get_audio_format_detail(&u_port->info.fmt, PJ_TRUE);		//查看端口信息中是否包含音频信息  真时取回
    d_afd = pjmedia_format_get_audio_format_detail(&d_port->info.fmt, PJ_TRUE);

    /* Both ports MUST have equal clock rate */					//必须有相同的时钟频率
    PJ_ASSERT_RETURN(u_afd->clock_rate == d_afd->clock_rate,
		     PJMEDIA_ENCCLOCKRATE);
printf("null_port->clock_rate=%d conf_port->clock_rate=%d\n",u_afd->clock_rate,d_afd->clock_rate);
//			打印出来都是8K----2016.8.24
    /* Both ports MUST have equal samples per frame */		//必须有相同的帧采样数
    PJ_ASSERT_RETURN(PJMEDIA_PIA_SPF(&u_port->info)==
			PJMEDIA_PIA_SPF(&d_port->info),
		     PJMEDIA_ENCSAMPLESPFRAME);

    /* Both ports MUST have equal channel count */				//必须有相同的通道数
    PJ_ASSERT_RETURN(u_afd->channel_count == d_afd->channel_count,
		     PJMEDIA_ENCCHANNEL);


    /* Get clock_rate and samples_per_frame from one of the port. */	//从端口获取时钟频率及帧采样数
    clock_rate = u_afd->clock_rate;							//
    samples_per_frame = PJMEDIA_PIA_SPF(&u_port->info);		//
    channel_count = u_afd->channel_count;					//


    /* Get the bytes_per_frame value, to determine the size of the
     * buffer. We take the larger size of the two ports.获取帧字节数的值  以决定缓冲区大小  我们取两个端口的最大值
     */
    bytes_per_frame = PJMEDIA_AFD_AVG_FSZ(u_afd);
    if (PJMEDIA_AFD_AVG_FSZ(d_afd) > bytes_per_frame)
		bytes_per_frame = PJMEDIA_AFD_AVG_FSZ(d_afd);


    /* Create the master port instance */							//创建主机端口实例
    m = PJ_POOL_ZALLOC_T(pool, pjmedia_master_port);
    m->options = options;
    m->u_port = u_port;			//null_port
    m->d_port = d_port;			//conference_bridge

    
    /* Create buffer */												//创建缓冲区
    m->buff_size = bytes_per_frame;
    m->buff = pj_pool_alloc(pool, m->buff_size);
    if (!m->buff)
		return PJ_ENOMEM;

    /* Create lock object */											
    status = pj_lock_create_simple_mutex(pool, "mport", &m->lock);
    if (status != PJ_SUCCESS)
		return status;

	/********************音频初始化配置********************/
	s32Ret = hi35xx_audio_init();
	if (s32Ret != HI_SUCCESS )	
	{
		printf("Hi35xx audio init fail!!\n");
		return PJ_FALSE;
	}
	
	/*****************************************************/

    /* Create media clock */											//创建媒体时钟回调函数
    status = pjmedia_clock_create(pool, clock_rate, channel_count, samples_per_frame, options, &clock_callback,m, &m->clock);
    if (status != PJ_SUCCESS) 
	{
		pj_lock_destroy(m->lock);
		return status;
    }

    /* Done */
    *p_m = m;

    return PJ_SUCCESS;
}


/*
 * Start the media flow.
 */
PJ_DEF(pj_status_t) pjmedia_master_port_start(pjmedia_master_port *m)
{
    PJ_ASSERT_RETURN(m && m->clock, PJ_EINVAL);
    PJ_ASSERT_RETURN(m->u_port && m->d_port, PJ_EINVALIDOP);

    return pjmedia_clock_start(m->clock);
}


/*
 * Stop the media flow.
 */
PJ_DEF(pj_status_t) pjmedia_master_port_stop(pjmedia_master_port *m)
{
    PJ_ASSERT_RETURN(m && m->clock, PJ_EINVAL);
    
    return pjmedia_clock_stop(m->clock);
}


/* Poll the master port clock */
PJ_DEF(pj_bool_t) pjmedia_master_port_wait( pjmedia_master_port *m,
					    pj_bool_t wait,
					    pj_timestamp *ts)
{
    PJ_ASSERT_RETURN(m && m->clock, PJ_FALSE);

    return pjmedia_clock_wait(m->clock, wait, ts);
}

/*
 * Callback to be called for each clock ticks.		时钟滴答时采集 播放帧  持续装载帧...
 */
static void clock_callback(const pj_timestamp *ts, void *user_data)
{
    pjmedia_master_port *m = (pjmedia_master_port*) user_data;
    pjmedia_frame frame;
	//pjmedia_frame dframe;
    pj_status_t status;


    /* Lock access to ports. */
    pj_lock_acquire(m->lock);

	
	//添加海思的音频输入
	//							m->u_port = null_port
	//-------------------------------------------------//
	//----------------		mic录音输入		-------------//			null_port===>master_port===>conference_bridge
    /* Get frame from upstream port and pass it to downstream port */		//从上流端口获取帧  传递给下流端口

    HI_MPI_AI_ReleaseFrame(AiDev, AiChn,&stFrame, NULL);
	pj_bzero(&stFrame, sizeof(AUDIO_FRAME_S));
    if(lance_call_start)		//----呼叫标志位 confirm时该位置一
    		call_start = PJ_TRUE;
	lance_audio_ai(call_start);		//将音频输入ai 转为 帧的形式输出(ai-->frame)
	
    pj_bzero(&frame, sizeof(frame));				//清零空间      
    frame.buf = &sendframebuf;	
    frame.size = m->buff_size;					//---只能320
    frame.timestamp.u64 = ts->u64;
	frame.type =PJMEDIA_FRAME_TYPE_AUDIO;
	if(sizeof(frame.buf)==0)
		frame.type = PJMEDIA_FRAME_TYPE_NONE;
    status = pjmedia_port_put_frame(m->d_port, &frame);				//往下游conference_bridge灌入帧


	//添加海思的音频播放
	//							m->d_port = conference_bridge
	//-------------------------------------------------//		
	//----------------		spk语音输出		-------------//					null_port===>master_port===>conference_bridge
    /* Get frame from downstream port and pass it to upstream port */		//从下游端口获取帧  传递给上游端口

    pj_bzero(&frame, sizeof(frame));
    frame.buf = m->buff;				
    frame.size = m->buff_size;
    frame.timestamp.u64 = ts->u64;

    status = pjmedia_port_get_frame(m->d_port, &frame);				//从下游conference_bridge获取帧
    if (status != PJ_SUCCESS)
		frame.type = PJMEDIA_FRAME_TYPE_NONE;
	
	//if(0)
	if(call_start)		//----呼叫标志位 confirm时该位置一
	{
		stFrame.pVirAddr[0]=frame.buf;
		HI_MPI_AO_SendFrame(AoDev,AoChn,&stFrame, HI_TRUE);
    	//status = pjmedia_port_put_frame(m->u_port, &frame);				//向上游null_port灌入帧
	}
	HI_MPI_AI_ReleaseFrame(AiDev, AiChn,&stFrame, NULL);
    /* Release lock */
    pj_lock_release(m->lock);
}


/*
 * Change the upstream port.
 */
PJ_DEF(pj_status_t) pjmedia_master_port_set_uport(pjmedia_master_port *m,
						     pjmedia_port *port)
{
    PJ_ASSERT_RETURN(m && port, PJ_EINVAL);

    /* Only supports audio for now */
    PJ_ASSERT_RETURN(port->info.fmt.type==PJMEDIA_TYPE_AUDIO, PJ_ENOTSUP);

    /* If we have downstream port, make sure they have matching samples per
     * frame.
     */
    if (m->d_port) {
	PJ_ASSERT_RETURN(
	    PJMEDIA_PIA_PTIME(&port->info) ==
		PJMEDIA_PIA_PTIME(&m->d_port->info),
	    PJMEDIA_ENCSAMPLESPFRAME
	);
    }

    pj_lock_acquire(m->lock);

    m->u_port = port;

    pj_lock_release(m->lock);

    return PJ_SUCCESS;
}


/*
 * Get the upstream port.
 */
PJ_DEF(pjmedia_port*) pjmedia_master_port_get_uport(pjmedia_master_port*m)
{
    PJ_ASSERT_RETURN(m, NULL);
    return m->u_port;
}


/*
 * Change the downstream port.
 */
PJ_DEF(pj_status_t) pjmedia_master_port_set_dport(pjmedia_master_port *m,
						  pjmedia_port *port)
{
    PJ_ASSERT_RETURN(m && port, PJ_EINVAL);

    /* Only supports audio for now */
    PJ_ASSERT_RETURN(port->info.fmt.type==PJMEDIA_TYPE_AUDIO, PJ_ENOTSUP);

    /* If we have upstream port, make sure they have matching samples per
     * frame.
     */
    if (m->u_port) {
	PJ_ASSERT_RETURN(
	    PJMEDIA_PIA_PTIME(&port->info) ==
		    PJMEDIA_PIA_PTIME(&m->u_port->info),
	    PJMEDIA_ENCSAMPLESPFRAME
	);
    }

    pj_lock_acquire(m->lock);

    m->d_port = port;

    pj_lock_release(m->lock);

    return PJ_SUCCESS;
}


/*
 * Get the downstream port.
 */
PJ_DEF(pjmedia_port*) pjmedia_master_port_get_dport(pjmedia_master_port*m)
{
    PJ_ASSERT_RETURN(m, NULL);
    return m->d_port;
}


/*
 * Destroy the master port, and optionally destroy the u_port and 
 * d_port ports.
 */
PJ_DEF(pj_status_t) pjmedia_master_port_destroy(pjmedia_master_port *m,
						pj_bool_t destroy_ports)
{
    PJ_ASSERT_RETURN(m, PJ_EINVAL);

    if (m->clock) {
	pjmedia_clock_destroy(m->clock);
	m->clock = NULL;
    }

    if (m->u_port && destroy_ports) {
	pjmedia_port_destroy(m->u_port);
	m->u_port = NULL;
    }

    if (m->d_port && destroy_ports) {
	pjmedia_port_destroy(m->d_port);
	m->d_port = NULL;
    }

    if (m->lock) {
	pj_lock_destroy(m->lock);
	m->lock = NULL;
    }


    return PJ_SUCCESS;
}


