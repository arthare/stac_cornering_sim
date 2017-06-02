// CorneringSimulation.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <fstream>
#include <iostream>
using namespace std;


int _tmain(int argc, _TCHAR* argv[])
{
  // physics parameters
  const double cda = 0.261;
  const double crr = 0.0033;
  const double mass = 80;
  const double wattage = 259;
  const double rho = 1.184; // density of air at 25C
  const double roadWidth = 3.7;
  
  // where is the corner along our simulated road?
  const double cornerPosition = 2000;

  // how much road are we simulating?
  const double simLength = 4000;


  // rider-confidence parameters
  const double maxBrakingGs = 0.8;
  const double maxCorneringGs = 0.9;

  const double maxBrakingAccel = maxBrakingGs * 9.81;
  const double maxCorneringAccel = maxCorneringGs * 9.81;
  

  // first, let's figure out terminal velocity for this combo of settings
  double terminalVelocity = 0;
  {
    double startSpeed = 1;
    const double dt = 0.0001;
    for(int x = 0;x < 10000000; x++)
    {
      // power = speed * force
      // force = power / speed
      const double forwardForce = wattage / startSpeed;
      const double rollingForce = -crr * mass * 9.81;
      const double aeroForce = -0.5 * cda * rho * startSpeed * startSpeed;
      const double totalForce = forwardForce + rollingForce + aeroForce;
      const double accel = totalForce / mass;
      startSpeed += accel * dt;

      if(abs(accel) < 0.0001)
      {
        terminalVelocity = startSpeed;
        break;
      }
    }
  }

  // ok, now we know the terminal velocity for this particular rider

  // this took me waaaaay too long to calculate, but appears to be correct.  This is the radius of the circle from the centerline to the apex to the centerline (right-hand turn) of a road with width roadWidth
  const double corneringRadius = roadWidth * (2 + sqrt(2.0));
  const double corneringSpeed = sqrt(maxCorneringAccel * corneringRadius); // centripetal acceleration equation

  const double cornerTotalCircumference = 2 * corneringRadius * 3.14159;
  const double corneringLength = cornerTotalCircumference / 4;



  double position = 0;
  double speed = terminalVelocity;
  double t = 0;

  ofstream file;
  file.open("c:\\temp\\cornering.txt");

  double dt = 0.01;
  // corner starts at cornerPosition, continues until cornerPosition+corneringLength
  while(position < simLength)
  {
    if(position < cornerPosition)
    {
      // not cornering yet, but we need to watch for braking

      
      double distanceToCorner = cornerPosition - position;
      // maxSpeedAllowed is the maximum speed the rider is allowed in order to decelerate in time for the corner
      // calculated from kinematic equation: v2^2 = v1^2 + 2 * acceleration * distance
      // in our case, v2 is their maximum allowable speed, v1 is the cornering speed they're declerating to, acceleration is their maximum braking force, distance is how far they are from the corner
      double maxSpeedAllowed = sqrt(pow(corneringSpeed, 2) + 2 * maxBrakingAccel * distanceToCorner);
      
      speed = min(maxSpeedAllowed, terminalVelocity); // assume the rider brakes perfectly if they need to, but otherwise maintains their constant-wattage terminal velocity
    }
    else if(position < cornerPosition + corneringLength)
    {
      // we're in the corner
      speed = min(corneringSpeed, terminalVelocity);
    }
    else
    {
      // we're re-accelerating 
      const double forwardForce = wattage / speed;
      const double rollingForce = -crr * mass;
      const double aeroForce = -0.5 * cda * rho * speed * speed;
      const double totalForce = forwardForce + rollingForce + aeroForce;
      const double accel = totalForce / mass;
      speed += accel * dt;
    }

    position += speed * dt;
    t += dt;

    file<<t<<"\t"<<speed<<"\t"<<position<<endl;
  }
  file.close();

  int iSeconds = (int)t;
  int minutes = iSeconds / 60;
  int seconds = iSeconds % 60;

  double kph = 3.6*(simLength / t);
  cout<<"Time to do "<<simLength<<"m was "<<minutes<<"m "<<seconds<<"s"<<" = "<<kph<<"km/h.  Cornered at "<<corneringSpeed * 3.6<<"km/h terminal = "<<3.6*terminalVelocity<<"km/h"<<endl;
  system("pause");
	return 0;
}

