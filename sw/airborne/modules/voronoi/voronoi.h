/*
 * Copyright (C) 2019  Hector Garcia de Marina
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

/** @file voronoi.h
 *
 *  Distributed/Centralized area coverage algorithm
 */

#ifndef VOR_H
#define VOR_H

/*! Default number of max neighbors per aircraft */
#ifndef VOR_MAX_NEIGHBORS
#define VOR_MAX_NEIGHBORS 10
#endif


#include "std.h"

struct vor_con {
  bool centralized;
  float radius;
  uint16_t timeout;
  uint16_t broadtime;
  float xc;
  float yc;
};

extern struct vor_con vor_control;

struct vor_tab {
  int16_t tableNei[VOR_MAX_NEIGHBORS][4];
};

extern struct vor_tab vor_tables;

extern void voronoi_init(void);
extern bool voronoi_main_loop(void);
extern void send_vorinfo_to_nei(void);

extern void parseVorfromGCS(void);


#endif // VOR_H
