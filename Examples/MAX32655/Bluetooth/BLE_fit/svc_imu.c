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

#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_ch.h"
#include "svc_imu.h"
#include "svc_cfg.h"
#include "fit_api.h" // to get QUATERNION_DATA_LEN macro
#include "imu_api.h" // to get singleImuData_t struct

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef IMU_SEC_PERMIT_READ
#define IMU_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef IMU_SEC_PERMIT_WRITE
#define IMU_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/* Custom UUIDs for IMU Service and Quaternion Characteristic (LSB) */
// IMU Service: 12345678-1234-1234-1234-1234567890AB
static const uint8_t imuServiceUuid[] = {0xAB, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
// Quaternion Characteristic: 12345678-1234-1234-1234-1234567890AC
#define QUATERNION_CHAR_UUID 0xAC, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
static const uint8_t quaterCharUuid[] = {QUATERNION_CHAR_UUID};

// IMU Service Declaration Variables
static const uint16_t imuServiceLen = sizeof(imuServiceUuid);

// Quaternion Characteristic Declaration Variables
static const uint8_t quaterChar[] = {ATT_PROP_NOTIFY, 
                                    UINT16_TO_BYTES(IMU_QUATERNION_VAL_HDL),
                                    QUATERNION_CHAR_UUID};
static const uint16_t quaterCharLen = sizeof(quaterChar);

// Quaternion Characteristic Value Variables
static uint8_t quaterVal[QUATERNION_DATA_LEN + (QUATERNION_DATA_LEN/sizeof(singleImuData_t))] = {0x00};
static const uint16_t quaterValLen = sizeof(singleImuData_t); // By default sizeof(singleImuData_t) but can be up to QUATERNION_DATA_LEN at runtime.

// Quaternion client characteristic configuration Variables
static uint8_t quaterCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t quaterCccLen = sizeof(quaterCcc);

// Quaternion Characteristic User Description (optional)
static const uint8_t quaterDesc[] = "Quaternion Sensor Value";
static const uint16_t quaterDescLen = sizeof(quaterDesc);

// Attribute List for IMU Service
static const attsAttr_t imuAttrList[] = {
    // IMU Service Declaration
    {
        attPrimSvcUuid,
        (uint8_t *)imuServiceUuid,
        (uint16_t *)&imuServiceLen,
        sizeof(imuServiceUuid),
        0, // No special settings
        ATTS_PERMIT_READ
    },
    // Quaternion Characteristic Declaration
    {
        attChUuid,
        (uint8_t *)quaterChar,
        (uint16_t *)&quaterCharLen,
        sizeof(quaterChar),
        0, // No special settings
        ATTS_PERMIT_READ
    },
    // Quaternion Characteristic Value
    {
        quaterCharUuid,           // Full 128-bit UUID
        (uint8_t *)quaterVal,     // Value will be updated dynamically
        (uint16_t *)&quaterValLen,
        sizeof(quaterVal),
        ATTS_SET_UUID_128,      // 128-bit UUID
        ATTS_PERMIT_READ
    },
    // Quaternion Characteristic User Description
    {
        attChUserDescUuid,
        (uint8_t *)quaterDesc,
        (uint16_t *)&quaterDescLen,
        sizeof(quaterDesc),
        0, // No special settings
        ATTS_PERMIT_READ
    },
    // Quaternion CCCD
    {
        attCliChCfgUuid,
        (uint8_t *)quaterCcc,
        (uint16_t *)&quaterCccLen,
        sizeof(quaterCcc),
        ATTS_SET_CCC,
        (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
    }
};

// IMU Service Group
static attsGroup_t imuSvcGroup = {
    NULL,
    (attsAttr_t *)imuAttrList,
    NULL,
    NULL,
    IMU_START_HDL, // Start handle
    IMU_END_HDL // End handle
};

// Add IMU Service to GATT Server
void SvcImuAddGroup(void)
{
    AttsAddGroup(&imuSvcGroup);
}

// Remove IMU Service from GATT Server
void SvcImuRemoveGroup(void)
{
    AttsRemoveGroup(IMU_START_HDL);
}

// Register Callbacks for IMU Service
void SvcImuCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
    imuSvcGroup.readCback = readCback;
    imuSvcGroup.writeCback = writeCback;
}
