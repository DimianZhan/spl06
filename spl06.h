/***************************************************************************
 *
 *  Copyright (c) 2019 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file spl06.h
 *
 * @author zwh <zwh@raaworks.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

extern void spl06_task(void);

extern void spl06_dma_request(void);
extern void spl06_dma_process(void);

#ifdef __cplusplus
}
#endif
