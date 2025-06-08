/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example IMU service implementation.
 *
 *  Copyright (c) 2011-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
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

#ifndef SVC_IMU_H
#define SVC_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup IMU_SERVICE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/
// Demo purpose. Decides how many quaternion data needs to be send. Possible values: 8, 16, 24, 32, 40, 48
#define QUATERNION_DATA_LEN 16

/** \name IMU Error Codes
 *
 */
/**@{*/
#define IMU_ERR_CP_NOT_SUP          0x80    /*!< \brief Control Point value not supported */
/**@}*/

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/** \name IMU Service Handles
 *
 */
/**@{*/
#define IMU_START_HDL               0x70              /*!< \brief Start handle. */
#define IMU_END_HDL                 (IMU_MAX_HDL - 1) // IMU_GYRO_CH_CCC_HDL /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief IMU Service Handles */
enum
{
  IMU_SVC_HDL = IMU_START_HDL,      /*!< \brief IMU service declaration */
  IMU_QUATERNION_CH_HDL,            /*!< \brief IMU quaternion measurement characteristic */
  IMU_QUATERNION_VAL_HDL,           /*!< \brief IMU quaternion measurement */
  IMU_QUATERNION_DESC_HDL,          /*!< \brief IMU quaternion measurement user description */
  IMU_QUATERNION_CH_CCC_HDL,        /*!< \brief IMU quaternion measurement client characteristic configuration */
  IMU_MAX_HDL                       /*!< \brief Maximum handle. */
};
/**@}*/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcImuAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcImuRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
// void SvcImuCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);
// TODO: Do we need to implement above function?
/*! \} */    /* HEART_RATE_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_IMU_H */
