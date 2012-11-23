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

/* INSTRUCTIONS FOR COMBAT WARSHIP USAGE:

ALL GLOBAL VARIABLES ARE PULLED FROM FLIGHT PHASE ZERO. I SUGGEST NOT HAVING THE COMPILE OPTION FOR PHASES ENABLED

Global variable #1: Determines Azimuth angle input stick or pot.  Stick count is 0-3, pot index is 4-6.  Choose as follows.  
		Set GVAR 1 in TX at -90 for 0, -70 for 1, -50 for 2, -30 for 3, -10 for 4, 10 for 5, 30 for 6
Global variable #2: Determines Range input stick or pot.  Stick count is 0-3, pot index is 4-6.  Choose as follows.  
		Set GVAR 2 in TX at -90 for 0, -70 for 1, -50 for 2, -30 for 3, -10 for 4, 10 for 5, 30 for 6
Global variable #3: Determines MAXIMUM dimensionless range for the primary turret cluster (range/distance between turret clusters) set to 
		Set GVAR 2 in TX at -90 for 1, -70 for 2, -50 for 4, -30 for 6, -10 for 8, 10 for 10, 30 for 12  
			***************************** SUGGEST NO SETTING HIGHER THAN 4,***************************************

			Range in stern turret = 1*(Rmax+1)/Rmax....  e.g. Rmax = 1, Rmax stern = 2
		

Functions available in mixes:   RNG (as shown in TX menu) gives the aft turret actual range to target.  Bow turret set by input stick (-100%-100%) -100% = 0 range, 100% = Maximum Range
								AZM (as shown in TX menu) gives the Azimuth angle BETA for stern turret.  ALPHA (bow turret angle) defined by input stick (+/-100%) 
																BETA: -Pi to Pi  (-100% to 100%)
																ALPHA:-Pi to Pi  (-100% to 100%)



						AFT CLUSTER											BOW CLUSTER
		--------------------+-----------------------------------------------------+-----------   Alpha Positive  Counterclockwise about bow cluster, zero to bow
																								Beta positive CLOCKWISE with zero at the stern
																								BetaV is an internal angle used in math. BetaV+Alpha = +/-Pi depending on port/stbd.  
																										If alpha is the angle from bow turret to target, 
																										betav is the same angle referenced clockwise with zero to the stern
To use, select appropriate function in pulldown menu in TX.  for bow cluster, range is the input chosen (e.g. Pot 1 for range, Pot 2 for azimuth or vise versa)
																for stern cluster, use one of the virtual channels such as 15 or 16.  
																		For range, set the weight to 100%.  Set the curve to be RNG
																		-Now on the actual output channel, select the previous channel as the input.  create a custom curve to map
																			the range value to the appropriate servo value to achieve that range based on linkage geometry
																			-Remember that RNG function is LINEAR.  e.g. -100% = 0, +100% = Maximum range
																			-Adjust the curve until the range achieved by the turret is the range you are commanding.
																			-This curve should be proven out before using the RNG function since the RNG function is a 
																				function of both the range from bow cluster AND the angle off of bow cluster.
																				-Prove it by applying any fixed input channel, e.g. P1, and make sure that P1=-100% range = 0 (not minimum range but zero, you can not do this mechanically)
																																							P1 = 100% range = maximum range = N times teh distance between your turret clusters as set by GVAR3
																																							P1 = 0% range = 50% of maximum range.  continue to refine as necessary
																																							Once that is set, apply the angle corrected curve.



 already tried to have only one return at the end */
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

int16_t BETAVfcn(int16_t x) { /*this function converts from alpha(angle off centerline, zero at straight forward, 
							 positive WRT right hand rule), the azimuth angle for bow turrets to beta, the azimmuth
							 angle for stern turrets with 0 pointed straight aft and positive WRT the LEFT hand rule, 
							 e.g clockwise looking down on the ship vs alpha's angle positive counterclockwise looking down
							 this means positive angles are to port, negative to starboard for a ship*/
	//if (x >1023 ){
  //	x=1024;
	//}
  // else if (x < -1023){
  // 	x = -1024;
  // }
  // if (x > -1) {
  // 	x = 1024-x;
  // }
  // else {//if (x < 0){
  //  x = -x-1024;
  // }
  int16_t BetaV = 0;
  BetaV = 1024-x;
  if (x < 0) {
    BetaV = BetaV-2048;
    }
    
    if (BetaV < -1023) {
    BetaV = -1024;
    }
    else if (BetaV > 1023) {
    BetaV = 1024;
    }
    
    
  //new test code
  return BetaV;
	//return x;
}

uint8_t ChannelChoice(int16_t x){
/* this function takes an input and selects channels 0 - 6 (0-3 for sticks, 4-6 for pots) and returns that array index*/
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
else { 
	x = 6;
}

return x;
}


uint8_t RangeMax(int16_t x){
	// this function sets the maximum range
	x=x/100;

if (x < -8){
	x = 1;
}
else if (x < -6){
	x = 2;
}
else if (x < -4){ 
	x = 4;
}
else if (x < -2){
	x = 6;
}
else if (x < 0){ 
	x = 8;
}
else if (x < 2){
	x = 10;
}
else { 
	x = 12;
}

return x;
}
uint16_t isqrt32b(uint32_t n)
{
    uint16_t c = 0x8000;
    uint16_t g = 0x8000;

    for(;;) {
        if((uint32_t)g*g > n)
            g ^= c;
        c >>= 1;
        if(c == 0)
            return g;
        g |= c;
    }
}

int16_t INTSQRT(int32_t x){
	//this estimates the square root of x within the range of sqrt(x) = 0 to 4096
	//x = x*x;
	if (x < 1) {
		x = 0;
	}


	int16_t nmax,n2,n3;

	int32_t n = 0;

	nmax = 4096;
 
	n2 = -1;
	n = 2048;
	n3 = 0;
	if (x == 0) {
		n2 = 0;
	} 
	else {
		
	while ( n2 < 0 ) {
		if (n == x/n) {
			
			n2 = n;
			}
		else {
			n = (n + x/n)/2;
		}

		n3=n3+1;
		if (n3 > 2000) {
			n2 = 0;
		}
	}
	}
	return n2;
}
int16_t TargetRange(int16_t Range16) {
	/*  this function calculates range to target.  it takes no direct inputs, however it uses values set in global variables to select sources
	I would prefer to remove these but until I figure out the menu structure and modify it, this is how it is.

	Range is dimensionless range.  Dimensionelss range is defined as range divided by the distance between bow and stern turret clusters on the ship.   E.G. math is the same
			for dimensionless range whether the turret spacing is 1m or 0.25m.  the differences is the ship with the closer turret spacing has a closer real range
			for an equivalent dimensionless range.  GRAVITY AND WIND not accounted for.  

	**** Definitions ****
	Global variable #1:  input number (0 to 6) for azimuth.  Azimuth angle is taken directly from calibratedstick[n], so full angle range = +/- 1024 = +/- Pi radians
					referenced to bow of ship. Positive to Port, Negative to starboard.
	Global Variabel #2: input number (0 to 6) for range.  Range is taken directly from calibratedstick[n] Range = +/-1024 = +/-100% = 0 to Max Range in menu structure
	*/

	int32_t Range32;
	int16_t Az16,RmaxStern,x;
	uint8_t m,n,Rmax;//,Rmult;   // m = Az stick n = range stick, 

	//m = ChannelChoice(GVAR_VALUE(0,0));
	//n = ChannelChoice(GVAR_VALUE(1,0));
	m = 4; //for debug set P1 for Azimuth, P2 for range
		n = 5;
	//Rmax = RangeMax(GVAR_VALUE(2,0));
	Rmax = 2;  //for debug 
	//prevents divide by zero error...
	if (Rmax < 1) {
		Rmax = 1;
	}

	
	RmaxStern = (2048*(Rmax + 1))/Rmax;

	////Az16=calibratedStick[m];  //-1 reverses pot to match physical turret
	//Range16=calibratedStick[n];
	//Following conditional prevents values too large from being used
		//if (Range16 > 1024) {
		//	Range16 = 1024;
		//}
		//else if (Range16 < -1024) {
		//	Range16 = -1024;
		//}

	////Az16 = BETAVfcn(Az16); //convert to virtual Beta for math purposes
//	Az16 += 1024;  // shift range to 0 to 2048 for math purposes    ***** DO NOT SHIFT SINCE COSINE FUNCTION EXPECTS +/-1024 *****
	Range16 = Range16 + 1024; // shift range to 0 to 2048 for math purposes
	// now we implement range_stern = sqrt( 1^2 + Rbow^2 -2*1*Rbow*Cos(BetaVirtual))    remember that 2 is 2, but 1 is not 1.  1 is turret cluster spacing/turret cluster spacing 
	//and must be scaled based on the range input we will see for Rbow, hence the math below

	//Take heed, do not exceed the limitation of a 32bit signed variable!  the parentheses are structured to minimize rounding errors in divisors!

	//intcos returns +/-1020 so we must divide by 1020 so that effectively its range is +/-1.  but we must do so after it is multiplied out large enough so the errors don't
	//kill the accuracy

	//adding conditionals to deal with possible integer overflow
	//Range32 = INTCOS(Az16)*4;  // the  4 is really 2048*2/1020
	
	//Range32 = -1*(Range32*Range16)/Rmax;  //this output now exists in range of +/-8388608, still smaller than the limitation of -2147483648 to 2147483647, which we would exceed if not careful with previous step
	
	Range32 = (int32_t)Range16*(int32_t)Range16;
	//***Range32a=Range16;

	//Range32 = Range32 + Range32a + 2048*2048/Rmax/Rmax;  // next step is the square root.  still need to implement
	
	//***Range32 = Range32/Range32a;
	
	
	//Range32 = INTSQRT(Range32);

	////now we scale that range back to the +/-1024 we are expecting



	//if (Range32 > RmaxStern){
	//	Range32 = RmaxStern;  //this should be the largest possible value given that the inputs are shifted to 0-2048, so max of sqrt(2048^2+2048^2-2*2048*2048*cos(beta)) = 4096
	//}
	//else if (Range32 < 0) {
	//	Range32 = 0;
	//}

	////remember, everything is referenced currently to 0 to 2048 being full range on the primary turret.   
	//Range32 = (Range32*Rmax)/(Rmax+1); //this should result in a proper scaling....
	//Range32 = Range32 - 1024; //now in +/-1024 land
	//if (Range32 > 1023){
	//	Range32 = 1024;
	//}
	//else if(Range32 <-1023){
	//	Range32 = -1024;
	//}

	//Range32 = Range16;
	//Range32 = Range32*Range32;
	Range32 = Range32/((int32_t)Range16);
	x = Range32-1024;

	return x;


}

int16_t TargetRange2(){

	int8_t R1max,RangeIndex,AzIndex;
	int16_t R1,Alpha,BetaV,Returnvar;
	int32_t R2,R1l;
	//the following three should be set via global variables
	RangeIndex = 5;
	AzIndex = 4;
	R1max = 2;

	R1 = calibratedStick[RangeIndex];
	Alpha = calibratedStick[AzIndex];

	BetaV = BETAVfcn(Alpha);
	//shift from +/-1024 to 0->2048
	R1 = R1 + 1024;
	R1l=R1;
	R2 = INTCOS(BetaV);
	//  -2*R1*L*cos(betav)  properly scaled.  
	//cos returns +/-1020. -4 = -2*2048/1020
	R2 = ((-4)*R2*((int32_t)R1))/((int32_t)R1max);
	// R1^2+L^2 -2*L*R1*cos(betav)  properly scaled
	R2 = (int32_t)R1*(int32_t)R1;
		
	R2 = R2	+ ((int32_t)2048)*((int32_t)2048)/((int32_t)R1max*(int32_t)R1max);//+int32_t(R2);//+(2048*2048)/((int32_t)R1max*(int32_t)R1max);
	// R2 = sqrt of previous
	//R2 = R2/int32_t(2);

	////R2 = INTSQRT(R2);
	R2=isqrt32b((uint32_t)R2);
	// this should output 0 to 2048*(R1max+1)/R1max
	//now for the proper scaling

	////R2 = (R2*(int32_t)R1max)/((int32_t)(R1max+1));

	//scale back to +/-1024
	
	R2 = R2 - 1024;

	//now for tail end error checking

	//Returnvar = (int16_t)R2;
	Returnvar = R2;

	if (Returnvar < -1023) {
		Returnvar = -1024;
	}
	else if (Returnvar > 1023) {
		Returnvar = 1024;
	}

	

	return Returnvar;





}
int16_t TargetRange3(){

														//int8_t R1max,RangeIndex,AzIndex;
														//int16_t R1,Alpha,BetaV,Returnvar;
														//int32_t R2,R2t;
														////the following three should be set via global variables
														//RangeIndex = 5;
														//AzIndex = 4;
														//R1max = 2;

														//R1 = calibratedStick[RangeIndex];
														//Alpha = calibratedStick[AzIndex];

														//BetaV = BETAVfcn(Alpha);
														////shift from +/-1024 to 0->2048
														//R1 = R1 + 1024;

														//R2t = INTCOS(BetaV);
														////  -2*R1*L*cos(betav)  properly scaled.  
														////cos returns +/-1020. -4 = -2*2048/1020
														//R2 = ((-4)*R2t*((int32_t)R1))/((int32_t)R1max);
														//// R1^2+L^2 -2*L*R1*cos(betav)  properly scaled
														//R2 = (int32_t)R1*(int32_t)R1+ 1048576;//+int32_t(R2)+(2048*2048)/((int32_t)R1max*(int32_t)R1max);
														//// R2 = sqrt of previous
														//R2 = INTSQRT(R2);
														//// this should output 0 to 2048*(R1max+1)/R1max
														////now for the proper scaling

														//////R2 = (R2*(int32_t)R1max)/((int32_t)(R1max+1));



														////scale back to +/-1024

														//R2 = R2 - 1024;

														////now for tail end error checking

														////Returnvar = (int16_t)R2;
														//Returnvar = R2;

														//if (Returnvar < -1023) {
														//	Returnvar = -1024;
														//}
														//else if (Returnvar > 1023) {
														//	Returnvar = 1024;
														//}

														//

														//return Returnvar;
	int8_t R1max,RangeIndex,AzIndex;
	int16_t R1,Alpha,BetaV,Returnvar;
	int32_t R2,R1l;
	//the following three should be set via global variables
	RangeIndex = 5;
	AzIndex = 4;
	R1max = 2;

	R1 = calibratedStick[RangeIndex];
	Alpha = calibratedStick[AzIndex];

	BetaV = BETAVfcn(Alpha);
	//shift from +/-1024 to 0->2048
	R1 = R1 + 1024;
	R1l=R1;
	R2 = INTCOS(BetaV);
	//  -2*R1*L*cos(betav)  properly scaled.  
	//cos returns +/-1020. -4 = -2*2048/1020
	R2 = ((-4)*R2*(R1l))/((int32_t)R1max);
	// R1^2+L^2 -2*L*R1*cos(betav)  properly scaled
	R2 = (int32_t)R1*(int32_t)R1;
		
	R2 = R2/((int32_t)4)	+ (int32_t)562500;//+int32_t(R2);//+(2048*2048)/((int32_t)R1max*(int32_t)R1max);
	// R2 = sqrt of previous
	//R2 = R2/int32_t(2);

	////R2 = INTSQRT(R2);
	R2=isqrt32b((uint32_t)R2);
	// this should output 0 to 2048*(R1max+1)/R1max
	//now for the proper scaling

	////R2 = (R2*(int32_t)R1max)/((int32_t)(R1max+1));

	//scale back to +/-1024
	
	R2 = R2 - 1024;

	//now for tail end error checking

	//Returnvar = (int16_t)R2;
	Returnvar = R2;

	if (Returnvar < -1023) {
		Returnvar = -1024;
	}
	else if (Returnvar > 1023) {
		Returnvar = 1024;
	}

	

	return Returnvar;





}
int16_t TargetRange4(){

																//int8_t R1max,RangeIndex,AzIndex;
																//int16_t R1,Alpha,BetaV,Returnvar;
																//int32_t R2,R2t;
																////the following three should be set via global variables
																//RangeIndex = 5;
																//AzIndex = 4;
																//R1max = 2;

																//R1 = calibratedStick[RangeIndex];
																//Alpha = calibratedStick[AzIndex];

																//BetaV = BETAVfcn(Alpha);
																////shift from +/-1024 to 0->2048
																//R1 = R1 + 1024;

																//R2t = INTCOS(BetaV);
																////  -2*R1*L*cos(betav)  properly scaled.  
																////cos returns +/-1020. -4 = -2*2048/1020
																//R2 = ((-4)*R2t*((int32_t)R1))/((int32_t)R1max);
																//// R1^2+L^2 -2*L*R1*cos(betav)  properly scaled
																//R2 = (int32_t)R1*(int32_t)R1;//+int32_t(1024*1024);//+R2+(2048*2048)/((int32_t)R1max*(int32_t)R1max);
																//// R2 = sqrt of previous
																//R2 = INTSQRT(R2);
																//// this should output 0 to 2048*(R1max+1)/R1max
																////now for the proper scaling

																//////R2 = (R2*(int32_t)R1max)/((int32_t)(R1max+1));

																////scale back to +/-1024

																//////R2 = R2 - 1024;

																////now for tail end error checking

																////Returnvar = (int16_t)R2;
																//Returnvar = R2-1024;

																//if (Returnvar < -1023) {
																//	Returnvar = -1024;
																//}
																//else if (Returnvar > 1023) {
																//	Returnvar = 1024;
																//}

																//

																//return Returnvar;


	int8_t R1max,RangeIndex,AzIndex;
	int16_t R1,Alpha,BetaV,Returnvar;
	int32_t R2;
	//the following three should be set via global variables
	RangeIndex = 5;
	AzIndex = 4;
	R1max = 2;

	R1 = calibratedStick[RangeIndex];
	//Alpha = calibratedStick[AzIndex];
	Alpha = 0;
	BetaV = BETAVfcn(Alpha);
	//shift from +/-1024 to 0->2048
	R1 = R1 + 1024;
	
	R2 = 1020;//INTCOS(BetaV); 
	//  -2*R1*L*cos(betav)  properly scaled.  
	//cos returns +/-1020. -4 = -2*2048/1020
	R2 = ((-4)*(int32_t)R2*((int32_t)R1))/((int32_t)R1max);
	// R1^2+L^2 -2*L*R1*cos(betav)  properly scaled
	R2 = (int32_t)R1*(int32_t)R1 + (int32_t)R2;
	R2 = (int32_t)R2 + ((int32_t)2048)*((int32_t)2048)/((int32_t)R1max*(int32_t)R1max);
		
	//R2 = R2	+ (int32_t)2250000;//+int32_t(R2);//+(2048*2048)/((int32_t)R1max*(int32_t)R1max);
	// R2 = sqrt of previous
	//R2 = R2/int32_t(2);

	////R2 = INTSQRT(R2);
	R2=isqrt32b((uint32_t)R2);
	// this should output 0 to 2048*(R1max+1)/R1max
	//now for the proper scaling

	R2 = ((int32_t)R2*(int32_t)R1max)/((int32_t)(R1max+1));

	//scale back to +/-1024
	
	R2 = R2 - 1024;

	//now for tail end error checking

	//Returnvar = (int16_t)R2;
	Returnvar = R2;

	if (Returnvar < -1023) {
		Returnvar = -1024;
	}
	else if (Returnvar > 1023) {
		Returnvar = 1024;
	}

	

	return Returnvar;



}
int16_t TR2(){

	int8_t R1max,RangeIndex,AzIndex;
	int16_t R1,Alpha,BetaV,Returnvar;
	int32_t R2;
	RangeIndex = 5;
	AzIndex = 4;
	R1max = 2;

	R1 = calibratedStick[RangeIndex];
	Alpha = calibratedStick[AzIndex];

	BetaV = BETAVfcn(Alpha);
	R1 = R1 + 1024;
	R2 = INTCOS(BetaV);
	R2 = ((-4)*R2*((int32_t)R1))/((int32_t)R1max);

	R2 = (int32_t)R1*R1 + 2250000;


	R2 = INTSQRT(R2);

	R2 = R2 - 1024;

	Returnvar = R2;

	if (Returnvar < -1023) {
		Returnvar = -1024;
	}
	else if (Returnvar > 1023) {
		Returnvar = 1024;
	}

	

	return Returnvar;
}