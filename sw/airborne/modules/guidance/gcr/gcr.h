/*
 * Copyright (C) 2016  Hector Garcia de Marina
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

/** \file gcr.h
 *
 *  Guidance algorithm to control angular velocities / centers of rotation
 */

#ifndef GCR_H
#define GCR_H

#define GCR_GRAVITY 9.806

#include "std.h"

typedef struct {
  float error;
  float gamma;
  float wo;
} gcr_con;

typedef struct {
  // Neighbor's position, speed and course (we assume same wo)
  float nx;
  float ny;
  float ns;
  float nc;
  float nwo;

  // Desired distance w.r.t. HOME and neighbor (we assume 'dn' are the
  // same for both planes)
  float dh;
  float dn;
  float en;
  float eh;
  float ep;
} gcr_fc;

extern gcr_con gcr_control;
extern gcr_fc gcr_formation;

extern void gcr_init(void);
extern bool gcr_wo(uint8_t wp, float gamma, float wo);
extern bool gcr_fc_wo(uint8_t wp, float gamma, float wo);
extern void gcr_set_gain(float gamma);
extern void gcr_set_wo(float wo);

#endif // GVF_H
