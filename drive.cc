/*
 * laserobstacleavoid.cc
 *
 * a simple obstacle avoidance demo
 *
 * @todo: this has been ported to libplayerc++, but not tested
 */

#define debug

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
	if (half < 50)
	{
		std::cout << "fail" << std::endl;
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
    		pp.SetSpeed(-0.5,0);
    		std::cout<<"<------AVOIDING-------->"<<std::endl;

    		double turnRate = 0.4;

    		if (minLeft < MIN_RANGE_AVOID)
    		{
    			turnRate = turnRate * -1;
    		}
    		pp.SetSpeed(0.0, turnRate);

    		usleep(20000);
    		pp.SetSpeed(0.1,0);
    		
    		
    		continue;
    	}
		
	 	double min = 100;
	 	int p = 0;
	 	//std::cout<<"18";
	 	double legBearing;
		double legDistance;
		player_point_2d xy;
		bool found = false;
		double x,y;
		double b = 1000;

		std::vector<laserData> points;
		std::vector<laserData> legs;

	 	//b=lp.GetBearing();//bearing of first point
		//for (int i = (half -170); i < (half + 170 ); i++)

		//get all points that could be legs...

		laserData fLeg, bLeg;
		states legState = nothing;

		for (int i = 1; i < lp.GetCount() -1; i++)
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
					cout<<"one leg"<<endl;
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

			//cout<<legState<<endl;
		}

		/*laserData ld;
				ld.dist = lp[i];
				ld.bearing = lp.GetBearing(i); 
				points.push_back(ld);*/
		//std::cout<<"1";
		
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
		
		
			#ifdef debug
				cout<<"<----Found "<<legs.size()<<" Legs ---------->"<<endl;

				for (int i = 0; i < legs.size(); ++i)
				{
					cout<< legs.at(i).bearing<<" ";
				}
				cout<<endl;
			#endif

		if(legs.size() > 0)
		{

			legPos = legs.at(0);
			for (int i = 0; i < legs.size(); ++i)
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
            //pp.GoTo((pp.GetXPos() + x),(pp.GetYPos() + y), (pp.GetYaw() + legBearing));
			//pp.GoTo(x,y,legBearing);
			//std::cout<<pp.GetXPos()<<" "<< y<<std::endl;

			//if within certain range/bearing set to 0 (reached goal)
			cout<< -(BEARING_EPSILON)<<endl;
			legBearing = (((legBearing < -(BEARING_EPSILON)) || (legBearing > BEARING_EPSILON)) ? legBearing : 0);
			legDistance = (((legDistance < -(DISTANCE_EPSILON)) || (legDistance > DISTANCE_EPSILON)) ? legDistance : 0);

			double yaw = (legBearing < 0 ? -(YAW_RATE) : YAW_RATE );
			//pp.SetSpeed(0,yaw);

			double absLegBearing = (legBearing > 0 ? legBearing : (-1 * legBearing));

			yaw = yaw * ((absLegBearing));


			std::cout<<(SPEED_FACTOR * legDistance)<<" "<< yaw<<" " << abs(legBearing)<<std::endl;

			//usleep(10000);
			pp.SetSpeed((SPEED_FACTOR * legDistance * minAll),yaw);

			
		}
		else
		{
			pp.SetSpeed(0,0,0);
		}

		

		//usleep(500000);
        	
       	}
    }

  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << e << std::endl;
    return -1;
  }
}


