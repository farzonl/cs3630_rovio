#include "RobotInterfaceRovio.h"
#include "orientation.h"

void giveOrders() 
{
    int counter = 0;
    double temp_angle = Angle;
    
	if((temp_angle >=40.0) && (temp_angle <=65.0))
	{
		if(side == Direction::left)
			rovio_DiagForLeft(3);

		else
			rovio_DiagForRight(3); 

		return;
	}

	if((temp_angle >=80.0) && (temp_angle <=105.0))
	{
		if(side == Direction::left)
		    rovio_driveLeft(3);

		else
			rovio_driveRight(3);

		return;
	}

	
	while(temp_angle >=0) 
    {
        temp_angle-=20;
        counter++;
    }

	if (side == Direction::left)
    {
		if(counter != 0)
		{
			rovio_turnLeftByDegree(counter);
			rovio_forward(3);
        }
        else if (side == center)
            rovio_forward(3);
        else
        {
			if(counter != 0)
			{
				rovio_turnRightByDegree(counter); 
				rovio_forward(3);
			}
        }    
    
}