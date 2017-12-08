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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */
#include "math/pprz_algebra_float.h"
#include "subsystems/abi.h"
#include "subsystems/abi_sender_ids.h"
#include "subsystems/datalink/datalink.h" // dl_buffer
#include "autopilot.h"
#include "modules/fc_rotor/fc_rotor.h"

#ifndef ABI_C
#define ABI_C 1
#endif

int fc_rotor_on;

void fc_rotor_init(void)
{
}

void fc_read_msg(void)
{
    struct FloatVect3 u;
    uint8_t ac_id = DL_FC_ROTOR_ac_id(dl_buffer);

    if (ac_id == AC_ID) {
      uint8_t dim = DL_FC_ROTOR_dim(dl_buffer);

      u.x = DL_FC_ROTOR_ux(dl_buffer);
      u.y = DL_FC_ROTOR_uy(dl_buffer);
      u.z = DL_FC_ROTOR_uz(dl_buffer);

      if(dim == 1)
      {
        AbiSendMsgACCEL_SP(ACCEL_SP_3D_ID, &u);
      }
      else if(dim == 0)
      {
        AbiSendMsgACCEL_SP(ACCEL_SP_2D_ID, &u);
      }
    }
}

