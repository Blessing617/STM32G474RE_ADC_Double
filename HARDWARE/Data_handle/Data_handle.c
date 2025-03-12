#include "Data_handle.h"

#define FFT_LENGTH		 1024
#define FRE				 1024000

uint32_t ADC_Value 	[FFT_LENGTH] = {0};
uint16_t ADC_1		[FFT_LENGTH] = {0};
uint16_t ADC_2		[FFT_LENGTH] = {0};
int      ADC_3		[FFT_LENGTH] = {0};

arm_cfft_radix4_instance_f32 scfft;

float fft_inputbuf [FFT_LENGTH*2] = {0};	//FFT��������
float fft_outputbuf[FFT_LENGTH]   = {0};	//FFT�������

void Fre_Control(uint32_t Fre)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

	uint32_t MaxData;
	uint16_t div=1;	
	while( (SystemCoreClock/Fre/div)>65535 )
	{
		div++;
	}
	MaxData =  SystemCoreClock/Fre/div - 1;	
	
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = div-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = MaxData;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Init(void)
{
	Fre_Control(FRE);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);  // ��ADC����У׼�����û�н���У׼�ɼ��������ݻ���ƫ��
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	
	HAL_TIM_Base_Start(&htim3);		
	HAL_ADC_Start(&hadc2);	
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&ADC_Value, 1024);  //��ʼͬ���ɼ�ADC
}

void ADC_Get(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);   // ��ADC����У׼
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&ADC_Value, FFT_LENGTH);  //��ʼͬ���ɼ�ADC	
}	

int fft_getpeak(float *inputx,float *input,float *output,uint16_t inlen,uint8_t x,uint8_t N,float y) //  intlen �������鳤�ȣ�xѰ�ҳ���
{                                                                           
	int i,i2;
	uint32_t idex;  //��ͬ����һ�������еģ���Ϊ�����ڲ�ͬ�ĺ����б�����
	float datas;
	float sum;
	int outlen=0;
	for(i=0;i<inlen-x;i+=x)
	{
		arm_max_f32(input+i,x,&datas,&idex);   
		if( (input[i+idex]>=input[i+idex+1])&&(input[i+idex]>=input[i+idex-1])&&( (2*datas)/FFT_LENGTH )>y)   
		   {
			   sum=0;   
			   for(i2=i+idex-N;i2<i+idex+N;i2++)   
			   {
				   sum+=input[i2];          
			   }        
			   if(1.5f*sum/(2*N)<datas)       
			   {                                                                                             
				     output[3*outlen+2] = atan2(inputx[2*(i+idex+1)+1],inputx[2*(i+idex+1)])*180/3.1415926f;				   
				     output[3*outlen+1] = 1.0f*(2.0f*datas)/FFT_LENGTH;   	//�������
					 output[3*outlen] = 1.0*FRE*(i+idex+1)/FFT_LENGTH;		//����Ƶ��		   
			   }                                                                                               
               else continue;			   
		   }			
		else continue;	
	}
	return outlen;	
}

float freamp[50];//��ȡ����г��Ƶ�ʺͷ�

void Data_handle(void)  
{	
	uint16_t i=0,ad1,ad2,ad3;
	uint16_t freamplen; 
	float angel=0;
	float32_t  v1_max=0,v2_max=0,v3_max=0,v1_min=4096,v2_min=4096,v3_min=4096,v1,v2,v3;
	float32_t  HZ1,HZ2,HZ3;
	float32_t  phase1,phase2;
	
	arm_cfft_radix4_instance_f32 scfft;
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
	
	for(i = 0, ad1 = 0, ad2 = 0 ,ad3 = 0; i<FFT_LENGTH; i++)   //�ֽ����ݣ� ADC_Value�ĸ�16λ��ADC2�����ݣ�ADC_Value�ĵ�16λ��ADC1������
	{
		ADC_1[ad1++] =  (uint16_t)ADC_Value[i];
		ADC_2[ad2++] =  (uint16_t)(ADC_Value[i]>>16);
		ADC_3[ad3++] =  (int)ADC_1[i] - (int)ADC_2[i];
//	printf("%d\r\n",ADC_3[i]);
	}	
	
		for(i=0;i<FFT_LENGTH;i++) 
		{	
			if(v1_max<=ADC_1[i])
			v1_max=ADC_1[i];
			if(v1_min>=ADC_1[i])
			v1_min=ADC_1[i];
		}		
		v1=(v1_max-v1_min)*3.3f/4095.0f;
	
		for(i=0;i<FFT_LENGTH;i++) 
		{	
			if(v2_max<=ADC_2[i])
			v2_max=ADC_2[i];
			if(v2_min>=ADC_2[i])
			v2_min=ADC_2[i];
		}
		
		v2=(v2_max-v2_min)*3.3f/4095.0f;
		
		for(i=0;i<FFT_LENGTH;i++) 
		{	
			if(v3_max<=ADC_3[i])
			v3_max=ADC_3[i];
			if(v3_min>=ADC_3[i])
			v3_min=ADC_3[i];
		}	
		v3=(v3_max-v3_min)*3.3f/4095.0f;
		
	  OLED_ShowFloat(0,0,v1,2);
	  OLED_ShowFloat(0,2,v2,2);
	  OLED_ShowFloat(0,4,v3,2);		
		
	  for(i=0;i<FFT_LENGTH;i++) 
	  {			
		fft_inputbuf[2*i]=ADC_1[i]*(3.3/4096.0);    //���������ź�ʵ��
		fft_inputbuf[2*i+1]=0;						//�鲿ȫ��Ϊ0
	  }
	  
	  arm_cfft_radix4_f32(&scfft,fft_inputbuf);  											//fft����
	  arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);								//��������������ģ�÷�ֵ
	  freamplen=fft_getpeak(fft_inputbuf,fft_outputbuf+1,freamp,FFT_LENGTH/2,10,5,0.2);		//Ѱ�һ�����г��	
	  
	  HZ1=freamp[0];	
	  phase1=freamp[2];	
	  freamp[0]=0,freamp[2]=0;	  
	  OLED_ShowFloat(50,0,HZ1,2);
	  
	  for(i=0;i<FFT_LENGTH;i++) 
	  {
		fft_inputbuf[2*i]=ADC_2[i]*(3.3/4096);   								 			//���������ź�ʵ��
		fft_inputbuf[2*i+1]=0;																//�鲿ȫ��Ϊ0				
	  }	
	  
	  arm_cfft_radix4_f32(&scfft,fft_inputbuf);  											//fft����
	  arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);								//��������������ģ�÷�ֵ
	  freamplen=fft_getpeak(fft_inputbuf,fft_outputbuf+1,freamp,FFT_LENGTH/2,10,5,0.2); 	//Ѱ�һ�����г��
	  
	  HZ2=freamp[0];
	  phase2=freamp[2];																		//��λ
	  freamp[0]=0,freamp[2]=0;	  
	  OLED_ShowFloat(50,2,HZ2,2);
	  
	  angel=phase2-phase1;			  
	  if(angel>180) angel=angel-180;  
	  if(angel<-180) angel=angel+180; 
	  OLED_ShowFloat(0,6,angel,2);	

	  for(i=0;i<FFT_LENGTH;i++) 
	  {
		fft_inputbuf[2*i]=ADC_3[i]*(3.3/4096);   								 			//���������ź�ʵ��
		fft_inputbuf[2*i+1]=0;																//�鲿ȫ��Ϊ0				
	  }	
	  
	  arm_cfft_radix4_f32(&scfft,fft_inputbuf);  											//fft����
	  arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);								//��������������ģ�÷�ֵ
	  freamplen=fft_getpeak(fft_inputbuf,fft_outputbuf+1,freamp,FFT_LENGTH/2,10,5,0.2); 	//Ѱ�һ�����г��

	  HZ3=freamp[0]; 
	  OLED_ShowFloat(50,4,HZ3,2);
}

