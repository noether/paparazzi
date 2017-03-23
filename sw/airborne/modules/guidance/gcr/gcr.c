/*
 * Copyright (C) 2016 Hector Garcia de Marina
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include <math.h>
#include "std.h"
#include "../include/std.h"

#include "modules/guidance/gcr/gcr.h"

#include "subsystems/navigation/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/autopilot.h"

// Control
gcr_con gcr_control;

// Formation
gcr_fc gcr_formation;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_gcr(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_GCR(trans, dev, AC_ID, &gcr_control.error, &gcr_formation.eh, &gcr_formation.en, 
          &gcr_formation.ep);
}
#endif

void gcr_init(void)
{
  gcr_control.gamma = 0.003;
  gcr_control.wo = 10;

  // Neighbor's position, speed and course (we assume same wo)
  gcr_formation.nx = 0;
  gcr_formation.ny = 0;
  gcr_formation.ns = 1;
  gcr_formation.nc = 0;
  gcr_formation.nwo = gcr_control.wo;

  // Desired distance w.r.t. HOME and neighbor
  gcr_formation.dh = 30;
  gcr_formation.dn = 30;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GCR, send_gcr);
#endif
}

bool gcr_wo(uint8_t wp, float gamma, float wo)
{
  wo = wo*M_PI/180;

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float ground_speed = stateGetHorizontalSpeedNorm_f();
  float course = stateGetHorizontalSpeedDir_f();
  float cx = waypoints[wp].x;
  float cy = waypoints[wp].y;
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;

  // Calculation of the desired angular velocity
  float omega = wo +
      gamma*wo*ground_speed*((px-cx)*sinf(course)+(py-cy)*cosf(course));

  // Error signal
  gcr_control.error = (px-cx+(ground_speed/wo)*cosf(course))*
      (px-cx+(ground_speed/wo)*cosf(course)) +
      (py-cy-(ground_speed/wo)*sinf(course))*
      (py-cy-(ground_speed/wo)*sinf(course));


  // Coordinated turn
  h_ctl_roll_setpoint =
      atanf(omega * ground_speed / GVF_GRAVITY / cosf(att->theta));
  BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

  lateral_mode = LATERAL_MODE_ROLL;

  return true;
}

// Formation control for a triangle, where agent1 is HOME
bool gcr_fc_wo(uint8_t wp, float gamma, float wo)
{
  wo = wo*M_PI/180;

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float ground_speed = stateGetHorizontalSpeedNorm_f();
  float course = stateGetHorizontalSpeedDir_f();
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;
  float hx = waypoints[wp].x;
  float hy = waypoints[wp].y;

  float cpx = px + (ground_speed/wo)*cosf(course);
  float cpy = py - (ground_speed/wo)*sinf(course);
  float cnx = gcr_formation.nx + (gcr_formation.ns/wo)*cosf(gcr_formation.nc);
  float cny = gcr_formation.ny - (gcr_formation.ns/wo)*sinf(gcr_formation.nc);

  float zhx = cpx - hx;
  float zhy = cpy - hy;
  float znx = cpx - cnx;
  float zny = cpy - cny;

  float norm_zh = sqrtf(zhx*zhx + zhy*zhy);
  float norm_zn = sqrtf(znx*znx + zny*zny);

  float eh = norm_zh - gcr_formation.dh;
  float en = norm_zn - gcr_formation.dn;

  float ux = eh*zhx/norm_zh + en*znx/norm_zn;
  float uy = eh*zhy/norm_zh + en*zny/norm_zn;

  gcr_formation.eh = eh;
  gcr_formation.en = en;
  gcr_formation.ep = gcr_formation.nc - course;
  NormRadAngle(gcr_formation.ep);

  float omega = wo + gamma*wo*ground_speed*(ux*sinf(course)+uy*cosf(course))
      + 10*gamma*sinf(gcr_formation.ep);

  // Coordinated turn
  h_ctl_roll_setpoint =
      atanf(omega * ground_speed / GVF_GRAVITY / cosf(att->theta));
  BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);

  lateral_mode = LATERAL_MODE_ROLL;

  return true;
}

void gcr_set_gain(float gamma)
{
  gcr_control.gamma = gamma;
}

void gvf_set_wo(float wo)
{
  gcr_control.wo = wo;
}
