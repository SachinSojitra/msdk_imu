/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Fitness sample application interface.
 *
 *  Copyright (c) 2011-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *
 *  Portions Copyright (c) 2022-2023 Analog Devices, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/

#ifndef EXAMPLES_MAX32655_BLUETOOTH_BLE_FIT_FIT_API_H_
#define EXAMPLES_MAX32655_BLUETOOTH_BLE_FIT_FIT_API_H_

#include "wsf_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifndef FIT_CONN_MAX
#define FIT_CONN_MAX 1
#endif

/*! \brief Increase MTU size to accomodate all IMUs and their ID
- by default MTU size is  is 23 by default - including 3 bytes ATT protocol header
- 8*6 (for all 6 IMUs) + 8 bytes for IMU ids + 8 extra bytes + 3 bytes att header */
#define FIT_APP_ATT_MAX_MTU 67

/*! \brief 1 if creating fw for upper body controller othewise 0, should be passed as cmd arg to make */
#ifndef IMU_UPPER_BODY_SENSOR
#define IMU_UPPER_BODY_SENSOR 1
#endif

/*! \brief Used to determine starting id of lower body IMUs e.g. it will start from 7 */
#define UPPER_BODY_IMU_COUNT 6

/*! \brief Demo purpose. Decides how many quaternion data needs to be send. Possible values: 8, 16, 24, 32, 40, 48 */
#define QUATERNION_DATA_LEN 48

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void FitStart(void);

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void FitHandlerInit(wsfHandlerId_t handlerId);

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for the application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void FitHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

#ifdef __cplusplus
};
#endif

#endif // EXAMPLES_MAX32655_BLUETOOTH_BLE_FIT_FIT_API_H_
