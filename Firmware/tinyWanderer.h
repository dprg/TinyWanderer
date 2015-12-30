/****************************************************************************
*     Copyright (C) 2011 Paul Bouchier                                      *
*                                                                           *
*     This program is free software: you can redistribute it and/or modify  *
*     it under the terms of the GNU General Public License as published by  *
*     the Free Software Foundation, either version 3 of the License, or     *
*     (at your option) any later version.                                   *
*                                                                           *
*     This program is distributed in the hope that it will be useful,       *
*     but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*     GNU General Public License for more details.                          *
*                                                                           *
*     You should have received a copy of the GNU General Public License     *
*     along with this program.  If not, see <http://www.gnu.org/licenses/>, *
*     or the LICENSE-gpl-3 file in the root directory of this repository.   *
****************************************************************************/

#define LSERVO_STOP 108	// left servo stopped
#define LSERVO_FWD 114	// left servo stopped 
#define LSERVO_FWD_SLO 110	// left servo stopped
#define LSERVO_BACK 96	// left servo stopped
#define LSERVO_BACK_SLO 100	// left servo stopped

#define RSERVO_STOP 108	// right servo stopped
#define RSERVO_FWD 104	// left servo stopped 
#define RSERVO_FWD_SLO 106	// left servo stopped
#define RSERVO_BACK 112	// left servo stopped
#define RSERVO_BACK_SLO 109	// left servo stopped

enum direction_e {FWD_STRAIGHT, FWD_LEFT, FWD_RIGHT, BACK_LEFT, BACK_RIGHT, BACK_STRAIGHT};

#define MAX_20MS 3000			// the highest we let the 20ms counter go before clearing it
#define normalize20ms(x) ((x) > MAX_20MS ? (x) - MAX_20MS : (x))

#define ADC_VREF_TYPE 0x20
#define LIGHT_DIFF_TRIGGER 10
#define DARK 235
#define DARK_HYSTERESIS 10

static void servoPulseSm();

