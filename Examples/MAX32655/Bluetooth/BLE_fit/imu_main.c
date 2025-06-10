/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  IMU sensor.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "att_api.h"
#include "svc_ch.h"
#include "svc_imu.h"
#include "app_api.h"
#include "app_hw.h"
#include "imu_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief Connection control block */
typedef struct
{
  dmConnId_t    connId;               /*! \brief Connection ID */
  bool_t        imuToSend;            /*! \brief IMU data ready to be sent on this channel */
} imuConn_t;

/*! \brief Control block */
static struct
{
  imuConn_t     conn[DM_CONN_MAX];    /* \brief connection control block */
  wsfTimer_t    measTimer;            /* \brief periodic measurement timer */
  imuData_t     data;                  /* \brief IMU measurement */
  imuCfg_t      cfg;                  /* \brief configurable parameters */
  bool_t        txReady;              /* \brief TRUE if ready to send notifications */
  uint8_t       flags;                /* \brief heart rate measurement flags */
} imuCb;

/**************************************************************************************************
  Connections
**************************************************************************************************/
static bool_t ImuNoConnActive(void)
{
  imuConn_t    *pConn = imuCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE)
    {
      return FALSE;
    }
  }
  return TRUE;
}

static void imuSetupToSend(void)
{
  imuConn_t    *pConn = imuCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE)
    {
      pConn->imuToSend = TRUE;
    }
  }
}

static imuConn_t *imuFindNextToSend(uint8_t cccIdx)
{
  imuConn_t     *pConn = imuCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE && pConn->imuToSend)
    {
      if (AttsCccEnabled(pConn->connId, cccIdx))
      {
        return pConn;
      }
    }
  }
  return NULL;
}

static uint8_t imuBuild(dmConnId_t connId, uint8_t **pBuf, imuData_t *pLatestImuData)
{
  uint8_t   *pImuData;
  uint8_t   len = QUATERNION_DATA_LEN + (QUATERNION_DATA_LEN / sizeof(singleImuData_t)); // TODO: Make this dynamic based on quaternion count
  uint8_t   sensorId = (IMU_UPPER_BODY_SENSOR ? 1 : (1 + UPPER_BODY_IMU_COUNT));
  uint8_t   maxLen = AttGetMtu(connId) - ATT_VALUE_NTF_LEN;
  APP_TRACE_INFO2("Maxlen %d Sending %d\n", maxLen, len);
  /* Adjust length if necessary */
  if (len > maxLen)
  {
    len = maxLen;
  }

  /* Allocate buffer */
  if ((*pBuf = (uint8_t *)WsfBufAlloc(len)) != NULL)
  {
    /* Add data to buffer */
    pImuData = *pBuf;
    /* qX, qY, qZ, qW */
    UINT8_TO_BSTREAM(pImuData, sensorId++);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qX);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qY);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qZ);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qW);
#if QUATERNION_DATA_LEN >= 16
    UINT8_TO_BSTREAM(pImuData, sensorId++);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu2.qX);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu2.qY);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu2.qZ);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu2.qW);
#if QUATERNION_DATA_LEN >= 24
    UINT8_TO_BSTREAM(pImuData, sensorId++);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu3.qX);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu3.qY);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu3.qZ);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu3.qW);
#if QUATERNION_DATA_LEN >= 32
    UINT8_TO_BSTREAM(pImuData, sensorId++);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu4.qX);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu4.qY);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu4.qZ);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu4.qW);
#if QUATERNION_DATA_LEN >= 40
    UINT8_TO_BSTREAM(pImuData, sensorId++);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu5.qX);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu5.qY);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu5.qZ);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu5.qW);
#if QUATERNION_DATA_LEN >= 48
    UINT8_TO_BSTREAM(pImuData, sensorId++);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu6.qX);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu6.qY);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu6.qZ);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu6.qW);
#endif // QUATERNION_DATA_LEN 16
#endif // QUATERNION_DATA_LEN 24
#endif // QUATERNION_DATA_LEN 32
#endif // QUATERNION_DATA_LEN 40
#endif // QUATERNION_DATA_LEN 48
    /* return length */
    return (uint8_t)(pImuData - *pBuf);
  }
  return 0;
}

static void imuSendNtf(dmConnId_t connId)
{
  uint8_t *pBuf;
  uint8_t len;

  /* Build heart rate measurement characteristic */
  if ((len = imuBuild(connId, &pBuf, &imuCb.data)) > 0)
  {
    /* Send notification */
    AttsHandleValueNtf(connId, IMU_QUATERNION_VAL_HDL, len, pBuf);

    /* Free allocated buffer */
    WsfBufFree(pBuf);
  }
}

static void imuConnOpen(dmEvt_t *pMsg)
{
  imuCb.txReady = TRUE;
}

static void imuHandleValueCnf(attEvt_t *pMsg)
{
  imuConn_t  *pConn;

  if (pMsg->hdr.status == ATT_SUCCESS && pMsg->handle == IMU_QUATERNION_VAL_HDL)
  {
    imuCb.txReady = TRUE;

    /* find next connection to send (note ccc idx is stored in timer status) */
    if ((pConn = imuFindNextToSend(imuCb.measTimer.msg.status)) != NULL)
    {
      imuSendNtf(pConn->connId);
      imuCb.txReady = FALSE;
      pConn->imuToSend = FALSE;
    }
  }
}

// TODO: Meet's point. Currently dummy values
void AppHwImuRead(imuData_t *pImu)
{
  ++(pImu->imu1.qX); // IMU1 Simulated quaternion X value
  ++(pImu->imu1.qY); // IMU1 Simulated quaternion Y value
  ++(pImu->imu1.qZ); // IMU1 Simulated quaternion Z value
  ++(pImu->imu1.qW); // IMU1 Simulated quaternion W value
  APP_TRACE_INFO4("IMU1 qX = %x qY = %x, qZ = %x, qW = %x", pImu->imu1.qX, pImu->imu1.qY, pImu->imu1.qZ, pImu->imu1.qW);
#if QUATERNION_DATA_LEN >= 16
  ++(pImu->imu2.qX); // IMU2 Simulated quaternion X value
  ++(pImu->imu2.qY); // IMU2 Simulated quaternion Y value
  ++(pImu->imu2.qZ); // IMU2 Simulated quaternion Z value
  ++(pImu->imu2.qW); // IMU2 Simulated quaternion W value
  APP_TRACE_INFO4("IMU2 qX = %x qY = %x, qZ = %x, qW = %x", pImu->imu2.qX, pImu->imu2.qY, pImu->imu2.qZ, pImu->imu2.qW);
#if QUATERNION_DATA_LEN >= 24
  ++(pImu->imu3.qX); // IMU3 Simulated quaternion X value
  ++(pImu->imu3.qY); // IMU3 Simulated quaternion Y value
  ++(pImu->imu3.qZ); // IMU3 Simulated quaternion Z value
  ++(pImu->imu3.qW); // IMU3 Simulated quaternion W value
  APP_TRACE_INFO4("IMU3 qX = %x qY = %x, qZ = %x, qW = %x", pImu->imu3.qX, pImu->imu3.qY, pImu->imu3.qZ, pImu->imu3.qW);  
#if QUATERNION_DATA_LEN >= 32
  ++(pImu->imu4.qX); // IMU4 Simulated quaternion X value
  ++(pImu->imu4.qY); // IMU4 Simulated quaternion Y value
  ++(pImu->imu4.qZ); // IMU4 Simulated quaternion Z value
  ++(pImu->imu4.qW); // IMU4 Simulated quaternion W value
  APP_TRACE_INFO4("IMU4 qX = %x qY = %x, qZ = %x, qW = %x", pImu->imu4.qX, pImu->imu4.qY, pImu->imu4.qZ, pImu->imu4.qW);
#if QUATERNION_DATA_LEN >= 40
  ++(pImu->imu5.qX); // IMU5 Simulated quaternion X value
  ++(pImu->imu5.qY); // IMU5 Simulated quaternion Y value
  ++(pImu->imu5.qZ); // IMU5 Simulated quaternion Z value
  ++(pImu->imu5.qW); // IMU5 Simulated quaternion W value
  APP_TRACE_INFO4("IMU5 qX = %x qY = %x, qZ = %x, qW = %x", pImu->imu5.qX, pImu->imu5.qY, pImu->imu5.qZ, pImu->imu5.qW);
#if QUATERNION_DATA_LEN >= 48
  ++(pImu->imu6.qX); // IMU6 Simulated quaternion X value
  ++(pImu->imu6.qY); // IMU6 Simulated quaternion Y value
  ++(pImu->imu6.qZ); // IMU6 Simulated quaternion Z value
  ++(pImu->imu6.qW); // IMU6 Simulated quaternion W value
  APP_TRACE_INFO4("IMU6 qX = %x qY = %x, qZ = %x, qW = %x", pImu->imu6.qX, pImu->imu6.qY, pImu->imu6.qZ, pImu->imu6.qW);
#endif // QUATERNION_DATA_LEN 16
#endif // QUATERNION_DATA_LEN 24
#endif // QUATERNION_DATA_LEN 32
#endif // QUATERNION_DATA_LEN 40
#endif // QUATERNION_DATA_LEN 48
}

void ImuMeasTimerExp(wsfMsgHdr_t *pMsg)
{
  imuConn_t *pConn;
  /* if there are active connections */
  if (ImuNoConnActive() == FALSE)
  {
    /* set up imu measurement to be sent on all connections */
    imuSetupToSend();

    /* read IMU measurement sensor data */
    AppHwImuRead(&imuCb.data);

    /* if ready to send measurements */
    if (imuCb.txReady)
    {
      /* find next connection to send (note ccc idx is stored in timer status) */
      if ((pConn = imuFindNextToSend(pMsg->status)) != NULL)
      {
        imuSendNtf(pConn->connId);
        imuCb.txReady = FALSE;
        pConn->imuToSend = FALSE;
      }
    }
    else
    {
      APP_TRACE_ERR0("txReady is false\n");
    }
    /* restart timer */
    WsfTimerStartMs(&imuCb.measTimer, imuCb.cfg.period);
  }
  else
  {
    APP_TRACE_INFO0("No connection active\n");
  }
}

void ImuInit(wsfHandlerId_t handlerId, imuCfg_t *pCfg)
{
    imuCb.measTimer.handlerId = handlerId;
    imuCb.cfg = *pCfg;
}

void ImuMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t hrmCccIdx)
{
  /* if this is first connection */
  if (ImuNoConnActive())
  {
    APP_TRACE_INFO3("ImuMeasStart: conn %u timer for %u ms with event %d\n", connId, imuCb.cfg.period, timerEvt);
    /* initialize control block */
    imuCb.measTimer.msg.event = timerEvt;
    imuCb.measTimer.msg.status = hrmCccIdx;

    /* start timer */
    WsfTimerStartMs(&imuCb.measTimer, imuCb.cfg.period);
  }

  /* set conn id */
  imuCb.conn[connId - 1].connId = connId;
}

void ImuMeasStop(dmConnId_t connId)
{
  /* clear connection */
  imuCb.conn[connId - 1].connId = DM_CONN_ID_NONE;
  imuCb.conn[connId - 1].imuToSend = FALSE;

  /* if no remaining connections */
  if (ImuNoConnActive())
  {
    /* stop timer */
    WsfTimerStop(&imuCb.measTimer);
  }
}

void ImuProcMsg(wsfMsgHdr_t *pMsg)
{
  // APP_TRACE_INFO1("ImuProcMsg event %u", pMsg->event);
  if (pMsg->event == DM_CONN_OPEN_IND)
  {
    // APP_TRACE_INFO0("imuConnOpen");
    imuConnOpen((dmEvt_t *) pMsg);
  }
  else if (pMsg->event == ATTS_HANDLE_VALUE_CNF)
  {
    // APP_TRACE_INFO0("imuHandleValueCnf");
    imuHandleValueCnf((attEvt_t *) pMsg);
  }
  else if (pMsg->event == imuCb.measTimer.msg.event)
  {
    // APP_TRACE_INFO0("ImuMeasTimerExp");
    ImuMeasTimerExp(pMsg);
  }
}

// TODO: Do we need to implement this?
// uint8_t ImuWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
//     uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);
