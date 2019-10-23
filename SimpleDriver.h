/***************************************************************************
 
    file                 : SimpleDriver.h
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

#ifndef SIMPLEDRIVER_H_
#define SIMPLEDRIVER_H_

#include <iostream>
#include <cmath>
#include <windows.h>
#include "BaseDriver.h"
#include "CarState.h"
#include "CarControl.h"
#include "SimpleParser.h"
#include "WrapperBaseDriver.h"

#define PI 3.141592653589793
// const double pi = 3.141592653589793;


using namespace std;

class SimpleDriver : public WrapperBaseDriver
{
public:
	
	// Constructor
	SimpleDriver(){stuck=0;clutch=0.0;};

	// SimpleDriver implements a simple and heuristic controller for driving
	virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	virtual void init(float *angles);

private:
	int count;
	
	/* Gear Changing Constants*/
	
	// RPM values to change gear 
	static const int gearUp[6];
	static const int gearDown[6];
		
	/* Stuck constants*/
	
	// How many time steps the controller wait before recovering from a stuck position
	static const int stuckTime;
	// When car angle w.r.t. track axis is grather tan stuckAngle, the car is probably stuck
	static const float stuckAngle;
	
	/* Steering constants*/
	
	// Angle associated to a full steer command
	static const float steerLock;	
	// Min speed to reduce steering command 
	static const float steerSensitivityOffset;
	// Coefficient to reduce steering command at high speed (to avoid loosing the control)
	static const float wheelSensitivityCoeff;
	// angles
	static const float angle[19];
	static float oppangle[36];
	
	/* Accel and Brake Constants*/
	
	// max speed allowed
	static const float maxSpeed;
	// Min distance from track border to drive at  max speed
	static const float maxSpeedDist;
	// pre-computed sin5
	static const float sin5;
	// pre-computed cos5
	static const float cos5;
	
	/* ABS Filter Constants */
	
	// Radius of the 4 wheels of the car
	static const float wheelRadius[4];
	// min slip to prevent ABS
	static const float absSlip;
	// range to normalize the ABS effect on the brake
	static const float absRange;
	// min speed to activate ABS
	static const float absMinSpeed;

	/* Clutch constants */
	static const float clutchMax;
	static const float clutchDelta;
	static const float clutchRange;
	static const float clutchDeltaTime;
	static const float clutchDeltaRaced;
	static const float clutchDec;
	static const float clutchMaxModifier;
	static const float clutchMaxTime;

	// counter of steps
	int stuck;
	int cslip;
	float avgslip;
	
	// current clutch
	float clutch;

	// Solves the gear changing subproblems
	int getGear(CarState &cs);

	// Solves the steering subproblems
	float getSteer(CarState &cs);
	float laststeer;
	float dev;
	float gettrack[19];
	
	// Solves the gear changing subproblems
	float getAccel(CarState &cs);
	float lastAccel;
	
	// Apply an ABS filter to brake command
	float filterABS(CarState &cs, float brake);

	// Solves the clutching subproblems
	void clutching(CarState &cs, float &clutch);
	int timeSinceShift;
	
	// Opponent handling
	float oppdist;
	int oppsens;
	float opp_x;
	float opp_y;
	
	// Gives width of track
	float getTrackWidth(CarState &cs);
	
	// Computes turn radius
	double getTurnRadius(double side_a, double side_b, double side_c, double angle_a, double angle_b);
	
	// Measures sensor beams
	void sampleTrack(CarState &cs);
	
};
#endif /*SIMPLEDRIVER_H_*/
