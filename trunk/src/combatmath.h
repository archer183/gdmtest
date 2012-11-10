/*
 * Authors (alphabetical order)
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny <shadow@privy.de>
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * - with  minor math work for rc warships by
 * - G. McFadden
 *
 * open9x is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */



/* already tried to have only one return at the end */
	uint8_t scp[65] = {255,255,255,254,254,253,252,251,250,249,
		247,246,244,242,240,238,236,233,231,	228,
		225,222,219,215,212,208,205,201,197,193,
		189,	185,180,176,171,167,162,157,152,147,
		142,136,131,	126,120,115,109,103,98,92,
		86,80,74,68,62,56,50,44,37,31,
		25,19,13,6,0};/*100,100,100,100,100,99,99,99,
		98,98,97,96,96,95,94,93,92,91,90,89,88,87,86,
		84,83,82,80,79,77,76,74,72,71,69,67,65,63,62,
		60,58,56,53,51,49,47,45,43,41,38,36,34,31,29,
		27,24,22,20,17,15,12,10,7,5,2,0};*/
	/* Preceeding defines one quarter of a cosine centered on zero, phase shift to create sine.  
	OUTPUT IS +/- 255 range for an input range of +/-128.  -1024 ->1024 is assumed in real life 
	and is divided by 8 to get the correct range for this array.  Larger not used to save memory.
	OUTPUT IS SCALED BACK UP tot +/-1020 by code. errors induced by this are small enough to ignore
	in most cases.  well worth it for not having to use 16 bit.  
	
	**Obsolete note follows**
	Might consider going to unsigned 8 bit to allow array values from 0 to 200
	for more resolution with no memory hit.  65 parts makes the symmetric math easier.  with int16 variables on the output,
	this should not be a problem*/
	uint8_t acosn[58]={239,237,235,232,230,228,
		225,223,221,218,216,213,211,208,206,203,
		201,198,195,193,190,187,184,182,179,176,
		173,170,167,164,161,158,155,152,148,145,
		141,138,134,131,127,123,119,115,111,106,
		102,97,92,87,81,75,68,61,53,43,30,0};/*{106,104,102,101,99,97,
		95,94,92,90,88,86,85,83,81,79,77,74,
		72,70,68,65,63,60,58,55,52,49,46,43,
		39,35,30,24,17,0};*/
	/* for better resolution, acos which should run from 0 to 128 has been scaled for the lookup 
	only.   so we scale up for better resolution for free
	by a factor of 3.   look for this to be divided out in the code.
	*/

int16_t INTCOS(int16_t x){
		x = x/8;   
		while (x > 128) {
			x = x - 256;
		}
		while (x < -128) {
			x = x + 256;
		}
		if ( x < -64) {
			x=-4*scp[128-abs(x)];
		}
		else if (x < 0) {
			x=4*scp[abs(x)];
		}
		else if (x < 65) {
			x = 4*scp[x];
		}
		else if (x < 129) {
			x = -4*scp[128-x];
		}
      return x; // will add actual after verification of function of this change
}
int16_t INTSIN(int16_t x){
		x = x/8;   //convert to 8-ish bit range for table lookup
		while (x > 128) {
			x = x - 256;
		}
		while (x < -128) {
			x = x + 256;
		}
		if /*( x < -64) {
			x=-4*scp[abs(x+64)];
		}
		else if*/ (x < 0) {
			x=-4*scp[abs(x+64)];
		}
		/*else if (x < 65) {
			x = 4*scp[128-(64+x)];
		}*/
		else if (x < 129) {
			x = 4*scp[abs(x-64)];
		}
		return x;
}
int16_t INTACOS(int16_t x){ // NOTE:  CURVE MUST BE SCALED SUCH THAT INPUT IS +/- 1020 It is obvious if you don't do that.
		x=x/8;
		//while (x > 128) {
		//	x=x-256;
		//}
		//while (x<100 ) {
		//	x=x+100;
		if (x < -128) {
			x = 128;
		}
		else if (x<-70) {
			x=1020-(4*acosn[abs(x)-71])/3;
		}
		else if (x < 71) { //curve fit for middle section of arccos}
			x=(((-10*x)/15)+128)*4;  //the 4x multiplier takes range from +/-255 to +/- 1024 effectively
		}
		else if (x < 129) {
			x=(4*acosn[x-71])/3;
		}
		else {
			x = 0;
		}
		//x=x*8;
		return x;
}

int16_t INTASIN(int16_t x){ // NOTE:  CURVE MUST BE SCALED SUCH THAT INPUT IS +/- 1020 It is obvious if you don't do that.
		x=x/8;
		//while (x > 128) {
		//	x=x-256;
		//}
		//while (x<100 ) {
		//	x=x+100;
		if (x < -128) {
			x = 128;
		}
		else if (x<-70) {
			x=1020-(4*acosn[abs(x)-71])/3;
		}
		else if (x < 71) { //curve fit for middle section of arccos}
			x=(((-10*x)/15)+128)*4;  //the 4x multiplier takes range from +/-255 to +/- 1024 effectively
		}
		else if (x < 129) {
			x=(4*acosn[x-71])/3;
		}
		else {
			x = 0;
		}
		//x=x*8;
		x=510-x;   //convert from ACOS TO ASIN ,  remember our angular range is +/-1020  = +/- Pi radians   so Pi/2 =510
		return x;
}

int16_t BETAfcn(int16_t x) { /*this function converts from alpha(angle off centerline, zero at straight forward, 
							 positive WRT right hand rule), the azimuth angle for bow turrets to beta, the azimmuth
							 angle for stern turrets with 0 pointed straight aft and positive WRT the LEFT hand rule, 
							 e.g clockwise looking down on the ship vs alpha's angle positive counterclockwise looking down
							 this means positive angles are to port, negative to starboard for a ship*/
	if (x >1024 ){
		x=1024;
	}
	else if (x < -1024){
		x = -1024;
	}
	if (x >= 0) {
		x = 1024-x;
	}
	else if (x < 0){
	 x = -x-1024;
	}
	return x;
}

int8_t ChannelChoice(int16_t x){
/* this function takes an input and selects channels 0 - 6 (0-3 for sticks, 4-7 for pots) and returns that array index*/
x = x / 100;  //this takes the +/- 1024 input and converts it to +/- 10
if (x < -8){
	x = 0;
}
else if (x < -6){
	x = 1;
}
else if (x < -4){ 
	x = 2;
}
else if (x < -2){
	x = 3;
}
else if (x < 0){ 
	x = 4;
}
else if (x < 2){
	x = 5;
}
else if (x < 4){ 
	x = 6;
}
else { 
	x = 7;
}
return x;
}


