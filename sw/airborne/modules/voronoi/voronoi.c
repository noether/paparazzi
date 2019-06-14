/*
 * Copyright (C) 2019 Hector Garcia de Marina
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include <math.h>
#include <std.h>

#include "modules/voronoi/voronoi.h"
#include "subsystems/datalink/datalink.h" // dl_buffer
#include "subsystems/datalink/telemetry.h"
#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"
#include "std.h"

#if PERIODIC_TELEMETRY
static void send_voronoi(struct transport_tx *trans, struct link_device *dev)
{
 if (vor_control.centralized)
    pprz_msg_send_VORONOI(trans, dev, AC_ID, &vor_control.xc, &vor_control.yc);
}
#endif // PERIODIC TELEMETRY

// Control
/*! Default centralized/decentralized */
#ifndef VOR_CENTRALIZED
#define VOR_CENTRALIZED 1
#endif
/*! Default radius for the circumference */
#ifndef VOR_RADIUS
#define VOR_RADIUS 80
#endif
/*! Default timeout for the neighbors' information */
#ifndef VOR_TIMEOUT
#define VOR_TIMEOUT 1500
#endif
/*! Default broadcasting time */
#ifndef VOR_BROADTIME
#define VOR_BROADTIME 200
#endif

struct vor_con vor_control = {VOR_CENTRALIZED, VOR_RADIUS, VOR_TIMEOUT, VOR_BROADTIME, 0, 0};
struct vor_tab vor_tables;

uint32_t last_transmision = 0;

void voronoi_init(void)
{
// We need to init the table of neighbors
//  for (int i = 0; i < DCF_MAX_NEIGHBORS; i++) {
//    dcf_tables.tableNei[i][0] = -1;
//    dcf_tables.error_sigma[i] = 0;
//  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VORONOI, send_voronoi);
#endif
}

bool voronoi_main_loop(void)
{
  uint32_t now = get_sys_time_msec();

  // Table / Execute algorithm
  //for (int i = 0; i < DCF_MAX_NEIGHBORS; i++) {
  //  if (dcf_tables.tableNei[i][0] != -1) {
  //    uint32_t timeout = now - dcf_tables.last_theta[i];
  //    if (timeout > dcf_control.timeout) {
  //      dcf_tables.tableNei[i][3] = dcf_control.timeout;
  //    } else {
  //      dcf_tables.tableNei[i][3] = (uint16_t)timeout;

  //      float t1 = dcf_control.theta;
  //      float t2 = dcf_tables.tableNei[i][1] * M_PI / 1800.0;
  //      float td = dcf_tables.tableNei[i][2] * M_PI / 1800.0;

  //      float c1 = cosf(t1);
  //      float s1 = sinf(t1);
  //      float c2 = cosf(t2);
  //      float s2 = sinf(t2);

  //      float e = atan2f(c2 * s1 - s2 * c1, c1 * c2 + s1 * s2) - gvf_control.s * td;

  //      u += e;
  //      dcf_tables.error_sigma[i] = (uint16_t)(e * 1800.0 / M_PI);
  //    }
  //  }
  // }

  
  // Update the new XY
  // u *= -gvf_control.s * dcf_control.k;

  // Go and orbit around XY
  // gvf_ellipse_XY(xc, yc, dcf_control.radius + u, dcf_control.radius + u, 0);

  if ((now - last_transmision > vor_control.broadtime) && (autopilot_get_mode() == AP_MODE_AUTO2)) {
    send_vorinfo_to_nei();
    last_transmision = now;
  }

  return true;
}

void send_vorinfo_to_nei(void)
{
  //struct pprzlink_msg msg;

  //for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
    //if (dcf_tables.tableNei[i][0] != -1) {
    //  msg.trans = &(DefaultChannel).trans_tx;
    //  msg.dev = &(DefaultDevice).device;
    //  msg.sender_id = AC_ID;
    //  msg.receiver_id = dcf_tables.tableNei[i][0];
    //  msg.component_id = 0;
    //  pprzlink_msg_send_DCF_THETA(&msg, &(dcf_control.theta));
  //  }
}

void parseVorfromGCS(void)
{
  if(vor_control.centralized)
  {
    vor_control.xc = DL_VOR_GCS_AIR_xc(dl_buffer);
    vor_control.yc = DL_VOR_GCS_AIR_yc(dl_buffer);
  }
}

// I leave this here in case we need to look how to parse a message into a table

/*
void parseRegTable(void)
{
  uint8_t ac_id = DL_DCF_REG_TABLE_ac_id(dl_buffer);
  if (ac_id == AC_ID) {
    uint8_t nei_id = DL_DCF_REG_TABLE_nei_id(dl_buffer);
    int16_t desired_sigma = DL_DCF_REG_TABLE_desired_sigma(dl_buffer);

    if (nei_id == 0) {
      for (int i = 0; i < DCF_MAX_NEIGHBORS; i++) {
        dcf_tables.tableNei[i][0] = -1;
      }
    } else {
      for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
        if (dcf_tables.tableNei[i][0] == (int16_t)nei_id) {
          dcf_tables.tableNei[i][0] = (int16_t)nei_id;
          dcf_tables.tableNei[i][2] = desired_sigma;
          return;
        }

      for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
        if (dcf_tables.tableNei[i][0] == -1) {
          dcf_tables.tableNei[i][0] = (int16_t)nei_id;
          dcf_tables.tableNei[i][2] = desired_sigma;
          return;
        }
    }
  }
}

void parseThetaTable(void)
{
  int16_t sender_id = (int16_t)(SenderIdOfPprzMsg(dl_buffer));
  for (int i = 0; i < DCF_MAX_NEIGHBORS; i++)
    if (dcf_tables.tableNei[i][0] == sender_id) {
      dcf_tables.last_theta[i] = get_sys_time_msec();
      dcf_tables.tableNei[i][1] = (int16_t)((DL_DCF_THETA_theta(dl_buffer)) * 1800 / M_PI);
      break;
    }
}

*/
