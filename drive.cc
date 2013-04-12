/*
 * laserobstacleavoid.cc
 *
 * a simple obstacle avoidance demo
 *
 * @todo: this has been ported to libplayerc++, but not tested
 */

#include <libplayerc++/playerc++.h>
#include <iostream>
#include <cmath>

#include "args.h"

#define RAYS 32

#include <stdlib.h> //for sleep(sec)

int main(int argc, char **argv)
{
  parse_args(argc,argv);

  // we throw exceptions on creation if we fail
  try
  {
    using namespace PlayerCc;

    PlayerClient robot(gHostname, gPort);
    Position2dProxy pp(&robot, 1);
    LaserProxy lp(&robot, gIndex);

    std::cout << robot << std::endl;

    pp.SetMotorEnable(true);


    sleep(2.0);

	int count, half;

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
		
	 	double min = 100;
	 	int points[1000] = {};
	 	int p = 0;
	 	//std::cout<<"18";
	 	double legBearing;
		double legDistance;
		player_point_2d xy;
		bool found = false;
		double x,y;
		double b = 1000;

	 	//b=lp.GetBearing();//bearing of first point
		//for (int i = (half -170); i < (half + 170 ); i++)
		for (int i = 1; i < lp.GetCount() -1; i++)
		{
			
			if ((lp[i-1] - lp[i]) > 0.10) 	
			{
				if (((lp.GetBearing(i)- b) > 0.1) &&
							 ((lp.GetBearing(i)- b) < 0.3))
				{
				
				//std::cout<<p<<" "<<i<<std::endl; 
					std::cout<<"<--------- LEGS ---------------> "<<std::endl;
					legBearing = lp.GetBearing(i);
					legDistance = lp[i];

					x = (legDistance*(sin(fabs(legBearing))));
					y = (legDistance*(cos(fabs(legBearing))));
					found = true;
					std::cout<<pp.GetXPos()<<" "<< x<<" "<< lp[i]<<" "<<legBearing<< std::endl<<pp.GetYPos()<<" "<<y<<std::endl;
					break;
                

				}
				b= lp.GetBearing(i);
			}
		}
		
		//std::cout<<"1";
		
		
/*
		for (int i = 1; i < 1000; ++i)
		{
			if(points[i]>0)
			{
				//std::cout<<points[i] <<" "<<lp.GetBearing(points[i])<< std::endl;
				//std::cout<<lp.GetBearing(points[i])<<" "<<b<<" " <<abs(lp.GetBearing(points[i])- b)<<std::endl;
				if ((((lp.GetBearing(points[i])- b)) > 0.1) && (((lp.GetBearing(points[i])- b)) < 0.5))
				{
					std::cout<<"<--------- LEGS ---------------> "<<std::endl;
					legBearing = lp.GetBearing(points[i]);
					legDistance = lp[points[i]];

					xy = lp.GetPoint(points[i]);
					x = (legDistance*(sin(abs(legBearing))));
					y = (legDistance*(cos(abs(legBearing))));
					found = true;
					std::cout<<pp.GetXPos()<<" "<< x<<" "<< lp[(points[i])]<<" "<<legBearing<< std::endl<<pp.GetYPos()<<" "<<y<<std::endl;
					break;
				}
				b = lp.GetBearing(points[i]); 
			
			}				
		}*/
		if (found == true)
		{
			
			std::cout<<"<------------GOING------------------>"<<std::endl;
			pp.SetOdometry(0,0,0);
            //pp.GoTo((pp.GetXPos() + x),(pp.GetYPos() + y), (pp.GetYaw() + legBearing));
			//pp.GoTo(x,y,legBearing);
			std::cout<<pp.GetXPos()<<" "<< y<<std::endl;

			//if within certain range/bearing set to 0 (reached goal)
			legBearing = (((legBearing < -0.05) || (legBearing > 0.05)) ? legBearing : 0);
			legDistance = (((legDistance < -0.1) || (legDistance > 0.1)) ? legDistance : 0);

			double yaw = (legBearing < 0 ? -1 : 1 );
			//pp.SetSpeed(0,yaw);

			double absLegBearing = (legBearing > 0 ? legBearing : (-1 * legBearing));

			yaw = yaw * ((absLegBearing));

			//while(pp.GetYaw() != legBearing) //block until yaw is reached
			//{
		//		std::cout<< pp.GetYaw() << " "<< yaw << " "<< legBearing<<std::endl;
		//		robot.Read();
		//	}
			std::cout<<(0.2 * legDistance)<<" "<< yaw<<" " << abs(legBearing)<<std::endl;

			//usleep(10000);
			pp.SetSpeed((0.15 * legDistance),yaw);

			
		}
		else
		{
			pp.SetSpeed(0,0,0);
		}
		if((lp.GetMinRight() < 0.1) || (lp.GetMinLeft() < 0.1))
    	{
    		std::cout<<"<------STOPING-------->"<<std::endl;
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


