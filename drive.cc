/*
 * laserobstacleavoid.cc
 *
 * a simple obstacle avoidance demo
 *
 * @todo: this has been ported to libplayerc++, but not tested
 */

//#define debug

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "args.h"
#include "values.h"

#define RAYS 32


#include <stdlib.h> //for sleep(sec)

struct laserData
{
	double dist = 1000;
	double bearing = 1000;
	bool valid = false;
};

enum states { front, back, nothing};

int main(int argc, char **argv)
{
  parse_args(argc,argv);

  // we throw exceptions on creation if we fail
  try
  {
    using namespace PlayerCc;
    using namespace std;

    PlayerClient robot(gHostname, gPort);
    Position2dProxy pp(&robot, 0);
    LaserProxy lp(&robot, gIndex);

    std::cout << robot << std::endl;

    pp.SetMotorEnable(true);


    sleep(2.0);

	int count, half;

	laserData legPos, previousLegPos; //global that keeps track of where the legs that robot is following are
	double minLeft, minRight, minAll;

	previousLegPos.bearing = 0; // means robot will initially follow set of legs closest to straight infront of it.
    // go into read-think-act loop
    for(;;)
    {

    // this blocks until new data comes; 10Hz by default
    robot.Read();

	count = lp.GetCount();

	half = count/2;
    //std::cout<<pp.GetXPos()<<"  " <<pp.GetYPos()<<std::endl;
	if (count == 0)
	{
		std::cout << "No laser Data - rereading" << std::endl;
	}
	else{
		//std::cout << count << std::endl;
		//std::cout << half << std::endl;
		minLeft = lp.GetMinRight();
		minRight = lp.GetMinLeft();
		minAll = std::min(minLeft,minRight);
		cout <<minRight << " " << minLeft<< endl;

		if(minAll < MIN_RANGE_AVOID)
    	{
    		pp.SetSpeed(-0.3,0);
    		std::cout<<"<------AVOIDING-------->"<<std::endl;

    		double turnRate = 0.6;

    		if (minLeft < MIN_RANGE_AVOID)
    		{
    			turnRate = turnRate * -1;
    		}
    		pp.SetSpeed(0.1, -turnRate);

    		usleep(20000);
    		//pp.SetSpeed(0.1,0);
    		
    		
    		continue; // dont continue with leg detection code, avoiding obsticle more important
    	}
		
	 	double legBearing;
		double legDistance;
		bool found = false;

		std::vector<laserData> points;
		std::vector<laserData> legs;

		//get all points that could be legs...

		laserData fLeg, bLeg;
		states legState = nothing;

		for (unsigned int i = 1; i < lp.GetCount() -1; i++)
		{
			
			if ((legState == nothing) && ((lp[i-1] - lp[i]) > GRAD_STEP))//found start of a leg? could be larger object
			{
				legState = front;
				fLeg.dist = lp[i];
				fLeg.bearing = lp.GetBearing(i); 

			}
			else if ((legState == front) && (lp[i] - lp[i-1] > GRAD_STEP)) //end of leg or object
			{
				//check if object was thin enough to be a leg...
				bLeg.bearing = lp.GetBearing(i-1);
				bLeg.dist = lp[i-1];

				//cout<<"bearing diff  "<<(bLeg.bearing - fLeg.bearing)<<endl;

				if(((bLeg.bearing - fLeg.bearing) < MAX_BEARING_DIFF) && ((bLeg.bearing - fLeg.bearing) > MIN_BEARING_DIFF)) //its a leg?
				{
					points.push_back(fLeg);
					#ifdef debug
						cout<<"added leg"<<endl;
					#endif
				}
				else
				{
					cout << "---------- too wide -----------------"<<endl;
				}

				//reset leg state
				legState = nothing;

			}
			else if((legState == front) && ((lp.GetBearing(i) - fLeg.bearing) > MAX_WIDTH))
			{
				//has got to wide, cant be a leg, bad reading 
				legState = nothing;
			}

		}
		
		int num = points.size();

		for (int i = 0; i < num -1; ++i) //use num -1 as the last point wont have another point to compare to
		{
			cout<< (points.at(i+1).bearing - points.at(i).bearing)<< " "<< (points.at(i+1).bearing - points.at(i).bearing)<<endl;
			 if (((points.at(i+1).bearing - points.at(i).bearing) > MIN_BEARING) &&
			 		((points.at(i+1).bearing - points.at(i).bearing) < MAX_BEARING))
			 {
			 	legs.push_back(points.at(i)); 
			 	i++ ;//found a set so increase i (dont look at next point as it is already part of a set)
			 }
		}
		
		cout<<"<----Found "<<legs.size()<<" Sets of Legs ---------->"<<endl;
			#ifdef debug
				

				for (unsigned int i = 0; i < legs.size(); ++i)
				{
					cout<< legs.at(i).bearing<<" ";
				}
				cout<<endl;
			#endif

		if(legs.size() > 0)
		{

			legPos = legs.at(0);
			for (unsigned int i = 0; i < legs.size(); ++i)
			{
				if(((fabs(legs.at(i).bearing)) - (fabs(previousLegPos.bearing)))
				  < ((fabs(legPos.bearing)) - (fabs(previousLegPos.bearing))))
				{
					legPos = legs.at(i);
				}
				
			}
			legDistance = legPos.dist;
			legBearing =  legPos.bearing;
			previousLegPos = legPos;
			found = true;
		}

		if (found == true)
		{
			
			std::cout<<"<------------GOING------------------>"<<std::endl;
			pp.SetOdometry(0,0,0);

			//if within certain range/bearing set to 0 (reached goal)
			legBearing = (((legBearing < -(BEARING_EPSILON)) || (legBearing > BEARING_EPSILON)) ? legBearing : 0);
			legDistance = (((legDistance < -(DISTANCE_EPSILON)) || (legDistance > DISTANCE_EPSILON)) ? legDistance : 0);

			double yaw = (legBearing < 0 ? -(YAW_RATE) : YAW_RATE );
			//pp.SetSpeed(0,yaw);

			double absLegBearing = (legBearing > 0 ? legBearing : (-1 * legBearing));

			yaw = yaw * ((absLegBearing));

			#ifdef debug
				std::cout<<(SPEED_FACTOR * legDistance)<<" "<< yaw<<" " << abs(legBearing)<<std::endl;
			#endif

			//usleep(10000);
			pp.SetSpeed((SPEED_FACTOR * legDistance * minAll),yaw);

			
		}
		else //no legs detected - dont do anything
		{
			pp.SetSpeed(0,0,0);
		}
        	
       	}
    }

  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << e << std::endl;
    return -1;
  }
}


