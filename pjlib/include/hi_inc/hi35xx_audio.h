#ifndef     __HI35XX_AUDIO_H__
#define     __HI35XX_AUDIO_H__

#include "hi_type.h"



#ifdef __cplusplus
#if __cplusplus

extern "C"{
#endif
#endif /* End of #ifdef __cplusplus */


#define SAMPLE_AUDIO_AI_DEV 0
#define SAMPLE_AUDIO_AO_DEV 0
#define CHN_NUM	2


#define SAMPLE_DBG(s32Ret)\
do{\
    printf("s32Ret=%#x,fuc:%s,line:%d\n", s32Ret, __FUNCTION__, __LINE__);\
}while(0)


///全局变量音频指针




static PAYLOAD_TYPE_E gs_enPayloadType = PT_G711A;
static AUDIO_FRAME_S stFrame; 				//音频帧结构体

static HI_BOOL gs_bMicIn = HI_FALSE;

static HI_BOOL gs_bAiAnr = HI_FALSE;
static HI_BOOL gs_bAioReSample = HI_FALSE;
static HI_BOOL gs_bUserGetMode = HI_FALSE;
static AUDIO_RESAMPLE_ATTR_S *gs_pstAiReSmpAttr = NULL;
static AUDIO_RESAMPLE_ATTR_S *gs_pstAoReSmpAttr = NULL;
static AUDIO_DEV	AiDev = SAMPLE_AUDIO_AI_DEV;
static AI_CHN		AiChn;
static AUDIO_DEV	AoDev = SAMPLE_AUDIO_AO_DEV;
static AO_CHN		AoChn = 0;
static ADEC_CHN	AdChn = 0;
static HI_S32		s32AiChnCnt;
static HI_S32		s32AencChnCnt;
static AENC_CHN	AeChn;
char lance_call_start;



#ifdef __cplusplus
#if __cplusplus
}

#endif

#endif /* End of #ifdef __cplusplus */
#endif /* End of #ifndef __HI35XX_AUDIO_H__*/

