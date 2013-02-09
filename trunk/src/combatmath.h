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

//the following lists the global variables that are used for inputs
enum CombatInputs {
	RangeIN = 1,  //corresponds to gvar 10  remember gvar 1,2,..n -> [0,1,2,..n-1]
	BearingIN = 0,
	MaxRangeIN = 2,
	SpacingG2 = 6,   //spacing for as yet not implemented additional turret groups, remember that this will be a fraction  of the dist between frontmost and aftmost group
	SpacingG3 = 5
	//DO NOT FORGET TO UPDATE MODEL MENU NAMES FOR GVARS!

};

enum combatarrayout {
	C9X_T4B,
	C9X_T4R,
	C9X_T3B,
	C9X_T3R,
	C9X_T2B,
	C9X_T2R,
	C9X_ARY
};

extern int16_t combatarray[C9X_ARY];

uint8_t scp[65] = {255,255,255,255,255,254,253,252,251,250,
		248,247,245,243,241,239,237,234,231,	229,
		226,223,220,216,213,209,206,202,198,194,
		190,	185,181,177,172,167,162,157,152,147,
		142,137,132,	126,121,115,109,104,98,92,
		86,80,74,68,62,56,50,44,38,31,
		25,19,13,6,0};
	/* Preceeding defines one quarter of a cosine centered on zero, phase shift to create sine.  
	OUTPUT IS +/- 255 range for an input range of +/-128.  -1024 ->1024 is assumed in real life 
	and is divided by 8 to get the correct range for this array.  Larger not used to save memory.
	OUTPUT IS SCALED BACK UP tot +/-1024 by code. errors induced by this are small enough to ignore
	in most cases.  well worth it for not having to use 16 bit.  
	
	**Obsolete note follows**
	Might consider going to unsigned 8 bit to allow array values from 0 to 200
	for more resolution with no memory hit.  65 parts makes the symmetric math easier.  with int16 variables on the output,
	this should not be a problem*/
uint8_t acosn[58]={240,238,236,233,231,229,
		226,224,221,219,216,214,212,209,206,204,
		201,199,196,193,191,188,185,182,180,177,
		174,171,168,165,162,159,155,152,149,145,
		142,139,135,131,127,124,120,115,111,107,
		102,97,92,87,81,75,69,61,53,43,31,0};
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
		if (x < -125) { //create 256 value from table above
			x=-4*(scp[128-abs(x)]+1);
		}
		else if ( x < -64) {
			x=-4*scp[128-abs(x)];
		}
		else if (x < -2) {
			x=4*scp[abs(x)];
		}
		else if (x < 3) {
			x=4*(scp[abs(x)]+1);
		}
		else if (x < 65) {
			x = 4*scp[x];
		}
		else if (x < 126) {  //new code adds the 256 value to the origninal 255 to correctly scale the cosine to +/-1024
			x = -4*scp[128-x];
		}
		else if (x < 129) {  //create the +256 end point
			x = -4*(scp[128-x]+1);
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
		else if*/ (x < 0) {  //takes care of the pesky 256 value
			if (x < -67 && x > -61){
				x=-4*(scp[abs(x+64)]+1);
			}
			else {
			x=-4*scp[abs(x+64)];
			}
		}
		/*else if (x < 65) {
			x = 4*scp[128-(64+x)];
		}*/
		else if (x < 129) {
			if (x < 67 && x > 61){
				x=4*(scp[abs(x-64)]+1);
			}
			else {
			x = 4*scp[abs(x-64)];
			}
		}
		return x;
}
int16_t INTACOS(int16_t x){ // NOTE:  CURVE MUST BE SCALED SUCH THAT INPUT IS +/- 1024 It is obvious if you don't do that.
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
			x=1024-(4*acosn[abs(x)-71])/3;
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

int16_t INTASIN(int16_t x){ // NOTE:  CURVE MUST BE SCALED SUCH THAT INPUT IS +/- 1024 It is obvious if you don't do that.
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
			x=1024-(4*acosn[abs(x)-71])/3;
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
		x=512-x;   //convert from ACOS TO ASIN ,  remember our angular range is +/-1024  = +/- Pi radians   so Pi/2 =512
		//this gives the middle band solution for arcsine =  (-Pi/2 to +Pi/2) anthing outside
		//that range is also a solution within that range.


		
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

//uint8_t ChannelChoice(int16_t x){
///* this function takes an input and selects channels 0 - 6 (0-3 for sticks, 4-6 for pots) and returns that array index*/
//x = x / 100;  //this takes the +/- 1024 input and converts it to +/- 10
//if (x < -8){
//	x = 0;
//}
//else if (x < -6){
//	x = 1;
//}
//else if (x < -4){ 
//	x = 2;
//}
//else if (x < -2){
//	x = 3;
//}
//else if (x < 0){ 
//	x = 4;
//}
//else if (x < 2){
//	x = 5;
//}
//else { 
//	x = 6;
//}
//
//return x;
//}


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
uint16_t isqrt32b(uint32_t n)  // integer square root courtesy of existing code.
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

int16_t Comb_input_fcn(int16_t x){
	x = abs(x)/10;
	if (x < 16){
		x = ex_chans[x];
	}
	else if (x > 19 && x < 27){
		x = x-20;
		x = calibratedStick[x];
	}
	else {
		x = 0;
	}

	return x;
}

int16_t LawOfCosAmp(int16_t Ra, int16_t Rb, int16_t Alpha){
// returns length of side Rc of triangle where alpha is included angle between legs Ra and Rb
	int32_t RI;  //RI is internal variable
	int16_t RO; //RO is return variable

	RI =INTCOS(Alpha); 
	//  -2*R1*L*cos(betav)  properly scaled.  
	//cos returns +/-1024. -4 = -2*2048/1024
	if( Ra > Rb) {
	RI = ((int32_t)RI*(int32_t)(-2)*(int32_t)Ra/((int32_t)1024))*(int32_t)Rb;
	}
	else {

	RI = ((int32_t)RI*(int32_t)(-2)*(int32_t)Rb/((int32_t)1024))*(int32_t)Ra;
	}
	//R2 = ((-4)*(int32_t)CosBV*((int32_t)R1))/((int32_t)R1max);
	// R1^2+L^2 -2*L*R1*cos(betav)  properly scaled
	RI = (int32_t)Ra*(int32_t)Ra+(int32_t)Rb*(int32_t)Rb+(int32_t)RI;
	//R2 = (int32_t)R1*(int32_t)R1 + (int32_t)R2;
	//R2 = (int32_t)R2 + ((int32_t)2048)*((int32_t)2048)/((int32_t)R1max*(int32_t)R1max);
		
	//R2 = R2	+ (int32_t)2250000;//+int32_t(R2);//+(2048*2048)/((int32_t)R1max*(int32_t)R1max);
	// R2 = sqrt of previous
	//R2 = R2/int32_t(2);

	////R2 = INTSQRT(R2);
	RI=isqrt32b((uint32_t)RI);
	RO = (int16_t)RI;
		return RO;



}


void TargetRange(){
	/*  this function calculates range to target.  it takes no direct inputs, however it uses values set in global variables (TBD) to select sources
	I would prefer to remove these but until I figure out the menu structure and modify it, this is how it is.

	Range is dimensionless range.  Dimensionelss range is defined as range divided by the distance between bow and stern turret clusters on the ship.   E.G. math is the same
			for dimensionless range whether the turret spacing is 1m or 0.25m.  the differences is the ship with the closer turret spacing has a closer real range
			for an equivalent dimensionless range.  GRAVITY AND WIND not accounted for.  

	**** Definitions ****
	Global variable #1:  input number (0 to 6) for azimuth.  Azimuth angle is taken directly from calibratedstick[n], so full angle range = +/- 1024 = +/- Pi radians
					referenced to bow of ship. Positive to Port, Negative to starboard.
	Global Variabel #2: input number (0 to 6) for range.  Range is taken directly from calibratedstick[n] Range = +/-1024 = +/-100% = 0 to Max Range in menu structure
	*/

	int8_t R1max,i;
	int16_t R1,Alpha,BetaV,Lst;
	int32_t R2,CosBV,Beta;
	//the following three should be set via global variables

	R1 = GVAR_VALUE(2,0);  // gvar 0 = corelates to 0 = 0, 10 = 1, 20 = 2 and so on
	R1 = R1/10;
	R1max = (int8_t)R1;
	
	if (R1max < 1){
		R1max = 1;
	}
	if (R1max >5){
		R1max = 5;
	}
	Lst = 2048/((int16_t)R1max);  //dist between turrets relative to R1max in 0-2048 domain
	//gvar 0 = corelates to 0 = 0, 10 = 1, 20 = 2 and so on thru 150 =15 for mixes 0-15 (1-16 in display)
	// 200 = 20 thru 260 = 26  correlates to input pots 0-6 (0,1,2,3 = sticks, 4,5,6 = pots)

	//RangeIN = 1,  //corresponds to gvar 10  remember gvar 1,2,..n -> [0,1,2,..n-1]
	//BearingIN = 0,
	//MaxRangeIN = 2,
	//SpacingG2 = 6,   //spacing for as yet not implemented additional turret groups, remember that this will be a fraction  of the dist between frontmost and aftmost group
	//SpacingG3 = 5

	R1 = Comb_input_fcn(GVAR_VALUE(RangeIN,0));
	Alpha = Comb_input_fcn(GVAR_VALUE(BearingIN,0));
	combatarray[3]=(GVAR_VALUE(1,0));
	combatarray[4]=(GVAR_VALUE(0,0));

	/*RangeIndex = 5;
	AzIndex = 4;
	R1max = 2;*/

	//R1 = calibratedStick[RangeIndex];
	//Alpha = calibratedStick[AzIndex];


	//Alpha = 0;
	BetaV = BETAVfcn(Alpha);  //converts from alpha to beta function.  alpha goes to beta virtual, alpha virtual goes to beta
	//shift R1 from +/-1024 to 0->2048
	R1 = R1 + 1024;
	
	if (R1 > 2048) {
		R1 = 2048;
	}
	else if (R1 < 0){
		R1 = 0;
	}

	CosBV =INTCOS(BetaV); 

	//recall inputs...  for function Ra = L = 2048/R1max, Rb = R1 shifted to 0-2048, Alpha = BetaV in range of +/-1024

	R2 = LawOfCosAmp(Lst,R1,BetaV);
			//			//  -2*R1*L*cos(betav)  properly scaled.  
			//			//cos returns +/-1020. -4 = -2*2048/1020
			//R2 = ((-4)*(int32_t)CosBV*((int32_t)R1))/((int32_t)R1max);
			//
			//			// R1^2+L^2 -2*L*R1*cos(betav)  properly scaled
			//			R2 = (int32_t)R1*(int32_t)R1 + (int32_t)R2;
			//			R2 = (int32_t)R2 + ((int32_t)2048)*((int32_t)2048)/((int32_t)R1max*(int32_t)R1max);
			//				
			//			//R2 = R2	+ (int32_t)2250000;//+int32_t(R2);//+(2048*2048)/((int32_t)R1max*(int32_t)R1max);
			//			// R2 = sqrt of previous
			//			//R2 = R2/int32_t(2);

			//			////R2 = INTSQRT(R2);
			//			R2=isqrt32b((uint32_t)R2);
	// this should output 0 to 2048*(R1max+1)/R1max
	//now for the proper scaling

	//first find gamma as gamma+betaV=beta

	//-L*cos(BetaV)  : Have to multiply by 1024 to change scale of fraction from +/-1 to +/-1024
	Beta = (int32_t)CosBV*(int32_t)Lst;  //*1024/1024
	//R1-Lcos(betaV)
	Beta=(int32_t)R1*(int32_t)1024-(int32_t)Beta;
	//(R1-Lcos(betaV)/R2

	if (R2 == 0) {
		R2 = 1;
	}

	Beta = (int32_t)Beta/((int32_t)R2);
	Beta = INTACOS((int16_t)Beta);
	//now we have the correction factor, so...

	Beta = abs(BetaV)+Beta;

	if (Alpha < 0) {  //flops to other side if alpha in negative half of range
		Beta = -Beta;
	}



	//code below must happen after the azimuth value is determined...




	R2 = ((int32_t)R2*(int32_t)R1max)/((int32_t)(R1max+1));

	//scale back to +/-1024
	
	R2 = R2 - 1024;

	//now for tail end error checking

	//Returnvar = (int16_t)R2;
	/*Returnvar = R2;

	if (Returnvar < -1023) {
		Returnvar = -1024;
	}
	else if (Returnvar > 1023) {
		Returnvar = 1024;
	}*/
	combatarray[C9X_T4R] = R2;
	combatarray[C9X_T4B] = Beta;
	combatarray[2] = R1max*1024/10;


	i=0;
	while (i<C9X_ARY){  //
	if (combatarray[i] < -1023) {
		combatarray[i] = -1024;
	}
	else if (combatarray[i] > 1023) {
		combatarray[i] = 1024;
	}
	i=i+1;
	}
	//return Returnvar;



}
