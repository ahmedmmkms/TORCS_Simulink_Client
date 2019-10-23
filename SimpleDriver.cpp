/***************************************************************************
 
    file                 : SimpleDriver.cpp
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
/***************************************************************************
 
    robot                : Presto 2013
    author(s)            : Uwe Kadritzke
 
 ***************************************************************************/

#include "SimpleDriver.h"

/* Gear Changing Constants*/
const int SimpleDriver::gearUp[6]=
	{
		9000,9000,9000,9000,9000,0
	};

const int SimpleDriver::gearDown[6]=
	{
		// 0,3600,5100,5700,6100,6600
		0,3600,4800,5500,5900,6500
	};

const float SimpleDriver::angle[19]=
	{
		-90,
		-63,
		-45,
		-35,
		-23,
		-15,
		-11,
		-6,
		-3,
		0,
		3,
		6,
		11,
		15,
		23,
		35,
		45,
		63,
		90
	};

/* Stuck constants*/
const int SimpleDriver::stuckTime = 25;
const float SimpleDriver::stuckAngle = 0.523598775; //PI/6

/* Accel and Brake Constants*/
const float SimpleDriver::maxSpeedDist=90.0;
const float SimpleDriver::maxSpeed=330.0;

/* Steering constants*/
const float SimpleDriver::steerLock=0.785398;

/* ABS Filter Constants */
const float SimpleDriver::wheelRadius[4]={0.3179,0.3179,0.3276,0.3276};
const float SimpleDriver::absSlip=2.0;
const float SimpleDriver::absRange=3.0;
const float SimpleDriver::absMinSpeed=3.0;

/* Clutch constants */
const float SimpleDriver::clutchMax=0.5;
const float SimpleDriver::clutchDelta=0.05;
const float SimpleDriver::clutchRange=0.82;
const float SimpleDriver::clutchDeltaTime=0.02;
const float SimpleDriver::clutchDeltaRaced=10;
const float SimpleDriver::clutchDec=0.04;
const float SimpleDriver::clutchMaxModifier=1.3;
const float SimpleDriver::clutchMaxTime=1.5;

float SimpleDriver::oppangle[36];


///////////////////////////////////////////////////////////////////////////////
void SimpleDriver::init(float *angles) {
///////////////////////////////////////////////////////////////////////////////

// set angles to cross track edge in equal distances:
	angles[0]  = -90.0;
	angles[1]  = -63.0;
	angles[2]  = -45.0;
	angles[3]  = -35.0;
	angles[4]  = -23.0;
	angles[5]  = -15.0;
	angles[6] =  -11.0;
	angles[7]  =  -6.0;
	angles[8]  =  -3.0;
	angles[9]  =   0.0;
	angles[10]  =  3.0;
	angles[11]  =  6.0;
	angles[12] =  11.0;
	angles[13]  = 15.0;
	angles[14]  = 23.0;
	angles[15]  = 35.0;
	angles[16] =  45.0;
	angles[17]  = 63.0;
	angles[18] =  90.0;

	count = 0;
	avgslip = 1.0;
	cslip = 1;
	laststeer = 0.0;
	lastAccel = 0.0;
	dev = 0.0;

//Opponent Sensors
	for (int i=0; i<36; i++) {
		oppangle[i] = (-175.0 + 10.0*i)/180.0*PI;
		// cout << oppangle[i] << " ";
	}
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
CarControl SimpleDriver::wDrive(CarState cs) {
///////////////////////////////////////////////////////////////////////////////

	// measuring the sensor beams
	sampleTrack(cs);

		// compute steering
		float steer = getSteer(cs);
		// normalize steering
		if (steer < -1)
			steer = -1;
		if (steer > 1)
			steer = 1;

	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > stuckAngle && cs.getSpeedX() < 5.0) {
		// update stuck counter
		stuck++;
	} // if not stuck decrease stuck counter
	else {
		stuck--;
	}

	// Limit stuck between 0 and 2*stuckTime
	stuck = (stuck < 0) ? 0 : stuck;
	stuck = (stuck > 2*stuckTime) ? 2*stuckTime : stuck;
	//cout << stuck << endl;

	// after car is stuck for a while apply recovering policy
	if (stuck > stuckTime) {
		// set gear and steering command assuming car is 
		// pointing in a direction outside of track 
		// dev = 0.0;
		// to bring car parallel to track axis
		// float steer = - cs.getAngle()/steerLock;
		steer = -steer;
		int gear = -1; // gear R
		float accel = 0.5;
		float brake = 0.0;
		
		// if car is pointing in the correct direction revert gear and steer  
		if (cs.getAngle()*cs.getTrackPos() > 1.0) {
			gear = 1;
			if (cs.getSpeedX() < -0.1) {
			accel = 0.0;
			brake = 0.8;
			} else {
			accel = 0.5;
			brake = 0.0;
			}
			steer = -steer;
		}

		// Calculate clutching
		clutching(cs,clutch);



		// build a CarControl variable and return it
		CarControl cc (accel,brake,gear,steer,clutch);
		return cc;
	}
	// car is not stuck: 
	else {
		// compute accel/brake command
		float accel_and_brake = getAccel(cs);
		// apply a bit of negative feedback
		accel_and_brake = 0.5*accel_and_brake + 0.5*lastAccel;
		lastAccel = accel_and_brake;

		// compute gear 
		int gear = getGear(cs);

		// Steering for opponents:
		if (oppsens != -1) {
			if (fabs(opp_x-4.0) < 10.0) { // opp_x between -6.0 & 14.0
				if (opp_y < 0.0) {
					steer = steer - 0.25/max(oppdist-2.0f , 1.0f);
				} else {
					steer = steer + 0.25/max(oppdist-2.0f , 1.0f);
				}
			} else if (opp_x < -10.0) {
				if (opp_y < 0.0) {
					steer = steer + 0.3/max(oppdist-8.0f , 1.0f);
				} else {
					steer = steer - 0.3/max(oppdist-8.0f , 1.0f);
				}
			}
		// Braking for opponents
		if (fabs(opp_y) < 3.0) {
			// if opponent is in front or back modify accel/brake
			// depending on position (cos) and distance. 
			// brake or accelerate: 
			accel_and_brake = accel_and_brake - cos(0.65*oppangle[oppsens])/sqrt(oppdist);
			}
		}

//if (cs.getSpeedX() > 80.0) 
//	accel_and_brake += 0.5;


		// set accel and brake from the joint accel/brake command 
		float accel,brake;
		if (accel_and_brake > 0) {
			accel = accel_and_brake;
			brake = 0;
		}
		else {
			accel = 0;
			brake = filterABS(cs,-accel_and_brake); // apply ABS to brake
		}

		// Calculate clutching
		clutching(cs,clutch);

		// build a CarControl variable and return it
		CarControl cc(accel,brake,gear,steer,clutch);
		return cc;
	}
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
float SimpleDriver::getAccel(CarState &cs) {
///////////////////////////////////////////////////////////////////////////////

	float radius = 0.0;
	float targetSpeed = maxSpeed;
	// float mu = 0.95;
	float mu = min(1.5*cslip/avgslip , 1.5); // adaptive mu 
	float G = 9.81;

	// out of track?
	if (fabs(cs.getTrackPos()) > 1.0) {
		return 0.4;
	} else {
		if (gettrack[10] > gettrack[9]) { // RIGHT TURN
		radius = getTurnRadius(
			gettrack[9],
			gettrack[8],
			gettrack[7],
			fabs(angle[8]/180*PI),
			fabs(angle[7]/180*PI)
			);
		} else if (gettrack[8] > gettrack[9])	{ //LEFT TURN
			radius = getTurnRadius(
			gettrack[9],
			gettrack[10],
			gettrack[11],
			fabs(angle[10]/180*PI),
			fabs(angle[11]/180*PI)
			);
		}
		if (radius > 0.0) {
			targetSpeed = 6.0*sqrt( mu*G*(radius+gettrack[9]) );
		}
	}
	
	targetSpeed = (targetSpeed > maxSpeed) ? maxSpeed : targetSpeed;
	
	//cout << cs.getSpeedY() << endl;
	
	
	//float carSpeedSq = cs.getSpeedX()*cs.getSpeedX();
	//float targetSpeedSq = targetSpeed*targetSpeed;
	//float brakedist = (carSpeedSq - targetSpeedSq) / (2.0*mu*G);
	//float freedist = sqrt(gettrack[9]*gettrack[9]*(1 + tan(fabs(cs.getAngle()))));
	//float brakedist = carSpeedSq / (2.0*mu*G);
	
	//cout << cs.getSpeedX() << endl;
	//cout << targetSpeed << endl;
	//cout << brakedist << endl;
	//cout << cs.getTrack(9) << endl;
	//cout << freedist << endl;
	//cout << "---" << endl;
	
	//return 2*log( targetSpeed/(fabs(cs.getSpeedX())+1) );
	
	
	
	
// if slip.wheel[3,4] > maxAccelSlip : increaseClutch
// >> accel: if clutch > 0.5 : reduce accel
		float slip = 0.0f;
		for (int i = 2; i < 4; i++) {
			slip += 3.6*cs.getWheelSpinVel(i)*wheelRadius[i];
		}
		// slip is the difference between actual speed of car and average speed of wheels
		// slip = cs.getSpeedX() - slip/2.0f;slip/0.00001f
	
	
	
	return 2/(1+exp(cs.getSpeedX()-targetSpeed)) - 1;
	
/*
	if (brakedist < freedist && cs.getSpeedX() < targetSpeed) {
//	cout << "mode 1" << endl;
		//return 1.0;
		return log( targetSpeed/( fabs(cs.getSpeedX())+1 ) - 1 );
	} else
//	cout << "mode 2" << endl;
		return 2/(1+exp(cs.getSpeedX()-targetSpeed)) - 1; // hard braking, save cornering
		//return 2/(1+fabs(cs.getSpeedX()/targetSpeed)) - 1; // fast cornering with occasional understeer
		//return 2/(1+fabs(carSpeedSq/targetSpeedSq)) - 1;
		//return 2/(1+exp(sqrt(cs.getSpeedX() - targetSpeed))) - 1;
		//return 2/(1+exp(sqrt(carSpeedSq - targetSpeedSq))) - 1;
*/
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
int SimpleDriver::getGear(CarState &cs) {
///////////////////////////////////////////////////////////////////////////////

	int gear = cs.getGear();
	int rpm  = cs.getRpm();
	timeSinceShift--;
	
//cout << rpm << endl;

	// if gear is 0 (N) or -1 (R) just return 1 
	if (gear < 1) {
		return 1;
	}
	
	// check if the RPM value of car is greater than the one suggested 
	// to shift up the gear from the current one
	// wait some time before shifting gears again	 
	if ( gear < 6 && rpm > gearUp[gear-1] && timeSinceShift < 1 ) {
		timeSinceShift = 20; // reset counter
		clutch = clutchMax; // press clutch
		return gear + 1;
		
	// check if the RPM value of car is lower than the one suggested 
	// to shift down the gear from the current one
	// wait some time before shifting gears again
	} else if ( gear > 1 && rpm < gearDown[gear-1] && timeSinceShift < 1 ) {
		timeSinceShift = 50; // reset counter
		clutch = clutchMax; // press clutch
		return gear - 1;

	// otherwhise keep current gear
	} else { 
		return gear;
	}
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
void SimpleDriver::clutching(CarState &cs, float &clutch) {
///////////////////////////////////////////////////////////////////////////////

	double maxClutch = clutchMax;

	// Check if the current situation is the race start
	if (cs.getDistRaced() < clutchDeltaRaced)
	clutch = maxClutch;

	// Adjust the current value of the clutch
	if(clutch > 0) {
		double delta = clutchDelta;
		if (cs.getGear() < 2) {
		// Apply a stronger clutch output when the gear is one and the race is just started
			delta /= 2;
			maxClutch *= clutchMaxModifier;
			// if (cs.getCurLapTime() < clutchMaxTime)
			// clutch = maxClutch;
		}
		// check clutch is not bigger than maximum values
		clutch = min(maxClutch,double(clutch));

		// if clutch is not at max value decrease it quite quickly
		if (clutch != maxClutch) {
			clutch -= delta;
			clutch = max(0.0,double(clutch));
		}
		// if clutch is at max value decrease it very slowly
		else
			clutch -= clutchDec;
	}
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
float SimpleDriver::getSteer(CarState &cs) {
///////////////////////////////////////////////////////////////////////////////
	/* Steering is divided into two methods: a steer value derived from the 
	sensor beams and a steer value based on the position of the car on the 
	track. Both steer values are mixed together depending on the track 
	position: The closer the car is to the border of the track, the more track 
	steering and the less sensor steering is used. When the car leaves the 
	track, only track steering is used as sensor steering becomes unreliable. */
	
	// sensorsteer
	/* sensorsteer uses the sensor beams. Therefore, the driver can look ahead 
	and also steer according to the radius of the turn(s) ahead. Sensor 
	steering looks on the track in the direction of the car regardless of the 
	angle. Thus, it can find straights through chicanes. 
	The beams are used as vectors and are added up for a resulting vector. The 
	car drives in the direction where there is the most space. 
	The angle of the car is mixed into the resulting vector according to the 
	angle, using a sine factor. */
	
	float dist = 0.0;
	float x = 0.0;
	float y = 0.0;
	float sensorsteer = 0.0;
	
	// vector addition of sensor beams
	// double angle to improve turn sensitivity
	for (int i=2; i<17; i++) {
		dist = pow(round(fabs(gettrack[i])),3.0); // -1 when car offtrack !!
		x += dist*cos(2.0*angle[i]/180*PI - 0.5*cs.getAngle()*sin(fabs(cs.getAngle())));
		y += dist*sin(2.0*angle[i]/180*PI - 0.5*cs.getAngle()*sin(fabs(cs.getAngle())));
	//cout << x << " ";
	//cout << y << " ";
	}
	//cout << "---" << endl;
	
	// measure center beam
	/*dist = pow(round(fabs(cs.getTrack(9))),2.5);
	x += dist*cos(-cs.getAngle());
	y += dist*sin(-cs.getAngle());*/

	// add track position
	/*dist = pow(round(fabs(cs.getTrack(9))),2.5);
	x += dist*cos(-cs.getAngle());
	y += dist*sin(-cs.getAngle());*/
	//y += 100.5*getTrackWidth(cs)*cs.getTrackPos();
	
	//cout << "---" << endl;
	
	
	// vector direction to steer angle
	if (x != 0.0) // prevent nan bug !!!
		sensorsteer = -atan( 2.0*y/x )/steerLock;
	
	// flatten steer response by negative feedback
	float steer = 0.5*sensorsteer - 0.5*laststeer;
	steer *= max(sqrt(avgslip/cslip) , 1.0f);
	laststeer = steer;
	sensorsteer = steer;
	
	/*
	if (cs.getTrack(11) > cs.getTrack(9)) { // RIGHT TURN
		dev += (dev > 0.95) ? 0.0 : 0.05;
	} else if (cs.getTrack(7) > cs.getTrack(9))	{ //LEFT TURN
		dev -= (dev < -0.95) ? 0.0 : 0.05;
	}*/
		
	float tracksteer = 0.0;
	//tracksteer = (-0.5*atan(cs.getTrackPos() + dev) + cs.getAngle())/steerLock; 
	tracksteer = (-0.5*atan(cs.getTrackPos()) + 0.5*cs.getAngle())/steerLock; 
	
	float steermix = fabs(cs.getTrackPos()); 
	steermix = (steermix > 1.0) ? 1.0 : steermix;
	steermix = pow(steermix,3.0);

	//cout << sensorsteer << endl;
	//cout << tracksteer << endl;
	//cout << "---" << endl;

	return (1-steermix)*sensorsteer + steermix*tracksteer;
	//return tracksteer;
	//return sensorsteer;
}
///////////////////////////////////////////////////////////////////////////////


/***************************************************************************
  Helper Functions
***************************************************************************/


///////////////////////////////////////////////////////////////////////////////
void SimpleDriver::sampleTrack(CarState &cs) {
///////////////////////////////////////////////////////////////////////////////
	// Oversampling does not work !!
	/*
	// First run: Overwrite
	for (int i=0; i<19; i++) {
		gettrack[i] = 0.1*cs.getTrack(i);
	}
	cout << cs.getTrack(9);
	// 2nd ~ 10th run: Add
	for (int n=1; n<10; n++) {
		for (int i=0; i<19; i++) {
			gettrack[i] += 0.1*cs.getTrack(i);
		}
	cout << " " << cs.getTrack(9);
	}
	cout << endl; */
	for (int i=0; i<19; i++) {
		gettrack[i] = min(cs.getTrack(i) , 200.0f);
	}

	// Check opponent sensors: 
	oppdist = 50.0f; // set to "alert" value
	oppsens = -1; // set to "invalid"
	/*
	// find nearest opponent ("radar version") :
	for (int i=0; i<36; i++) {
		if (oppdist > cs.getOpponents(i)) {
			oppdist = cs.getOpponents(i);
			oppsens = i;
		}
	}
	*/

	// Scanner version: 18,17,19,16,20,15,21,14,...
	int j = 1;
	for (int i=18; i<36; i++) {
		if (oppdist > cs.getOpponents(i)) {
			oppdist = cs.getOpponents(i);
			oppsens = i;
		}
		if (oppdist > cs.getOpponents(i-j)) {
			oppdist = cs.getOpponents(i-j);
			oppsens = i-j;
		}
		j += 2;
	}

	if (oppsens > -1) {
		// Calc vector:
		opp_x = oppdist * cos(oppangle[oppsens]);
		opp_y = oppdist * sin(oppangle[oppsens]);
	}
	// cout << oppdist << " " << oppsens << " " << opp_x << " " << opp_y << endl;
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
float SimpleDriver::getTrackWidth(CarState &cs) {
///////////////////////////////////////////////////////////////////////////////
	float trwidth = gettrack[0] + gettrack[18];
	trwidth *= sin(0.5*PI - cs.getAngle());
	//cout << trwidth << endl;
	return trwidth;
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
float SimpleDriver::filterABS(CarState &cs, float brake) {
///////////////////////////////////////////////////////////////////////////////
	// convert km/h to m/s: 
	float speed = cs.getSpeedX()/3.6;
	// when speed lower than min speed for abs do nothing
	if (speed < absMinSpeed)
		return brake;

	// compute the speed of wheels in m/s
	float slip = 0.0f;
	for (int i = 0; i < 4; i++) {
		slip += cs.getWheelSpinVel(i) * wheelRadius[i];
	}
	// slip is the difference between actual speed of car and average speed of wheels
	slip = speed - slip/4.0f;

	cslip++;
	avgslip += max(slip , 0.0f);
	// cout << 1.5*cslip/avgslip << " " << sqrt(avgslip/cslip) <<endl;

	// when slip too high apply ABS
	if (slip > absSlip)
		brake = brake - (slip - absSlip)/absRange;

	// check brake is not negative, otherwise set it to zero
	if (brake < 0)
		return 0;
	else
		return brake;
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
double SimpleDriver::getTurnRadius(double side_a, double side_b, double side_c, 
								   double angle_a, double angle_b) {
///////////////////////////////////////////////////////////////////////////////

	// The sensor beams form 3 triangles: between the center beam  and the next 
	// (c-a-b), between the next-to-center and the outer beam (b-d-e), and 
	// between the center beam and outer beam (c-f-e). 
	// The sides of these triangles that are also chords at the (outer) turn 
	// radius form another triangle: a-d-f. The circle enclosing the triangle 
	// approximately equals the outer turn radius. 
	
	// Parameter: 
	double c = side_a;   // longest sensor beam
	double b = side_b;   // middle beam
	double e = side_c;   // shortest beam
	double aa = angle_a; // sensor angle between c, b
	double ff = angle_b; // sensor angle between c, e

	// Helper vars: 
	double cSq = c*c;
	double bSq = b*b;
	double eSq = e*e;
	double dd = ff-aa;
	double tradius = 0.0;

	// 3rd side a from c, b, angle:
	double a = sqrt( bSq + cSq - 2*b*c*cos(aa) );

	// 3rd side d aus b, e, angle:
	double d = sqrt( bSq + eSq - 2*b*e*cos(dd) );

	// f aus c, e, angle:
	double f = sqrt( cSq + eSq - 2*c*e*cos(ff) );

	// Area solution to prevent rounding error sensitive overuse of 
	// sin/cos/asin/acos functions:
	if (a+d > f) {
		double s = 0.5*(a+d+f);
		double A = sqrt( s*(s-a)*(s-d)*(s-f) );
		tradius = min( (a*d*f)/(4*A), 10000.0);
	}

	//cout << "c" << c << " b" << b << " e" << e << " ff" << ff << " aa" << aa << " dd" << dd << endl;
	//cout << "a" << a << " d" << d << " f" << f << endl;
	//cout << "r" << tradius << endl;

	return tradius;

}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
void SimpleDriver::onRestart() {
///////////////////////////////////////////////////////////////////////////////
	cout << "Restarting the race!" << endl;
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
void SimpleDriver::onShutdown() {
///////////////////////////////////////////////////////////////////////////////
	cout << "Next time I'll get you!" << endl;
}
///////////////////////////////////////////////////////////////////////////////
