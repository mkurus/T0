/*
 * bsp.h
 *
 *  Created on: 27 Oca 2016
 *      Author: admin
 */

#ifndef BSP_H_
#define BSP_H_

#define T0_TELIT_GL865
//#define T0_QUECTEL_M66

#ifdef 	T0_TELIT_GL865
#undef 	T0_QUECTEL_M66
#undef  T0_SIMCOM_SIM800C
#endif

#ifdef 	T0_SIMCOM_SIM800C
#undef 	T0_TELIT_GL865
#undef  T0_QUECTEL_M66
#endif

#ifdef 	T0_QUECTEL_M66
#undef 	T0_TELIT_GL865
#undef  T0_SIMCOM_SIM800C
#endif

#define DEVICE_MODEL  "T0"

#endif /* BSP_H_ */

