#ifndef DLOG_4CH_IQ_H_
#define DLOG_4CH_IQ_H_

//*********** Structure Definition ********//
typedef struct{
	int *input_ptr1;
	int *input_ptr2;
	int *input_ptr3;
	int *input_ptr4;
	int *output_ptr1;
	int *output_ptr2;
	int *output_ptr3;
	int *output_ptr4;
	int prev_value;
	int trig_value;
	int status;
	int pre_scalar;
	int skip_count;
	int size;
	int count;
}DLOG_4CH_IQ;

//*********** Function Declarations *******//
void DLOG_4CH_IQ_init(DLOG_4CH_IQ *v);
void DLOG_4CH_IQ_FUNC(DLOG_4CH_IQ *v);

//*********** Macro Definition ***********//
#define DLOG_4CH_IQ_MACRO(v)												\
	switch(v.status)														\
	{																		\
	case 0: /* wait for trigger*/											\
		if((int)*v.input_ptr1>v.trig_value && v.prev_value<v.trig_value)	\
		{																	\
			/* rising edge detected start logging data*/					\
			v.status=1;														\
		}																	\
		break;																\
		                                                                    \
	case 1:																	\
		v.skip_count++;														\
		                                                                    \
		if (v.skip_count == v.pre_scalar)									\
		{																	\
			v.skip_count = 0;												\
			v.output_ptr1[v.count] = *v.input_ptr1;							\
			v.output_ptr2[v.count] = *v.input_ptr2;							\
			v.output_ptr3[v.count] = *v.input_ptr3;							\
			v.output_ptr4[v.count] = *v.input_ptr4;							\
			v.count++;														\
			                                                                \
			if(v.count==v.size)												\
			{																\
				v.count     = 0;											\
				v.status    = 0;											\
			}																\
		}																	\
		break;																\
	}																		\
	v.prev_value = *v.input_ptr1;

	
#endif /* DLOG_4CH_IQ_H_ */
