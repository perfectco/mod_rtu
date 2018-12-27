/** @file custom_baord.h
 * 
 * @brief A description of the module's purpose.
 * 
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Perfect Co.
 * All rights reserved.
 */


#ifndef __CUSTOM_BOARD_H__
#define __CUSTOM_BOARD_H__

#if defined(BOARD_TCB_PERFCO_REV_001)
 #include "board_tcb_rev_001.h"
#elif defined(BOARD_SCALE_PERFCO_REV_001)
 #include "board_scale_rev_001.h"
#elif defined (BOARD_SCALE_PERFCO_REV_001_SOFTSERVE)
 #include "board_scale_softserve_rev_001.h"
#elif defined(BOARD_PCA10040_CUSTOM)
 #include "board_pca10040.h"
#elif defined(BOARD_PCA10056_CUSTOM)
  #include "board_pca10056_custom.h"
#elif defined(BOARD_NRF52840_MDK)
  #include "nrf52840_mdk.h"
#elif defined(BOARD_NRF52840_MDK_BAKEIT)
  #include "nrf52840_mdk_bakeit.h"
#else
 #error "Board is not defined"
#endif

#endif /* __CUSTOM_BOARD_H__ */
