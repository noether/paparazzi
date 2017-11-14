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
#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/datalink.h" // dl_buffer
#include "autopilot.h"
#include "modules/fc_rotor/fc_rotor.h"

#ifndef FC_ROTOR_ID
#define FC_ROTOR_ID 66
#endif

#ifndef ABI_C
#define ABI_C 1
#endif

int fc_rotor_on;

void fc_rotor_init(void)
{
    fc_rotor_on = 0;
}

void fc_read_msg(void)
{
    struct FloatVect3 u;
    uint8_t ac_id = DL_FC_ROTOR_ac_id(dl_buffer);

    if (ac_id == AC_ID) {
      printf("Msg received\n");
      uint8_t av = DL_FC_ROTOR_av(dl_buffer);

      u.x = DL_FC_ROTOR_ux(dl_buffer);
      u.y = DL_FC_ROTOR_uy(dl_buffer);
      u.z = DL_FC_ROTOR_uz(dl_buffer);


      if(av == 0)
        return;
      else if(av == 1)
      {
        printf("U %i %i %f \n", AC_ID, av, u.x);
        AbiSendMsgACCEL_SP(SENDER_ID, &u);
      }
    }
}

