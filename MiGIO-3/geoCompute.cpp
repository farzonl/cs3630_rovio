#include <math.h>
#include <assert.h>
#include "curl/curl.h"

class Point
{
    public:
        Point(int x, int y): m_x(x), m_y(y) { }
        virtual ~Point() { }
        
        int getX() { return m_x; }
        int getY() { return m_y; }
        
        int setX(int x) { m_x = x; }
        int setY(int y) { m_y = y; }
        
        virtual double distance(Point* p)
        {
            assert(NULL != p);
            int dx = m_x - p.m_x;
            int dy = m_y - p.m_y;
            return sqrt(dx*dx + dy*dy);
        }
        
    private:
        int m_x;
        int m_y;
};

class LemonPoint : Point
{
    LemonPoint(int x, int y): Point(x, y) { }
    virtual ~LemonPoint()                 { }
};

class RobotPoint : Point
{
    public Robot(int x, int y): Point(x, y) { }
    virtual ~Robot()                        { }
};

class MathGod
{
    public:
    
        MathGod(RobotPoint* R LemonPoint* L):
        {
            assert(NULL != R);
            assert(NULL != L);
            m_intersect     = new Point(L->getX(),R->getY());
            
            m_hypotenuse    = R->distance(L);
            m_parallelLine  = m_intersect->distance(R);
            m_othogonalLine = m_intersect->distance(L);
            
            m_angle         = asin(m_parallelLine/m_hypotenuse);
            
            if(R.getX()> L.getX())
                m_side = left;
            
            else if (R.getX() == L.getX()
                m_side = center;
            
            else
                m_side = right;
            
        }

    double GetAngle()          const { return m_angle;          }
    double GetDirection()      const { return m_side;           }
    Point  GetIntersect()      const { return m_intersect;      }
    double GetHypotenuse()     const { return m_hypotenuse;     }
    double GetParallelLine()   const { return m_parallelLine;   }
    double GetOrthogonalLine() const { return m_orthogonalLine; }
    
    void   SetAngle          (double angle)          { m_angle          = angle;          }
    void   SetIntersect      (Point  intersect)      { m_intersect      = intersect;      }
    void   SetHypotenuse     (double hypotenuse)     { m_hypotenuse     = hypotenuse;     }
    void   SetParallelLine   (double parallelLine)   { m_parallelLine   = parallelLine;   }
    void   SetOrthogonalLine (double orthogonalLine) { m_orthogonalLine = orthogonalLine; }
 

    private:
        double m_hypotenuse;
        double m_parallelLine;
        double m_orthogonalLine;
        double m_angle;
        Point  m_intersect;
        Direction m_side;
};


enum Direction
{
    left =0,
    center =1,
    right = 2,
};


class Decision
{
    public:
        //this will need to be the math function eventually
        Decision(double angle, double pLine, Direction side):
        {
            m_side  = side;
            m_angle = angle;
            m_pLine = pLine;
        }
        
    void giveOrders() 
    {
        CURL *curl;
        curl = curl_easy_init();
        int counter = 0;
        double temp_angle = m_angle;
        while((temp_angle-20) >=0) 
        {
            temp_angle-=20;
            counter++;
        }
        
        if(curl) 
        {
            if (side == left)
            {
                rovio_turnLeftByDegree(curl,counter);
                // add a rovio forward comand here
            }
            else if (side == center)
                rovio_forward(curl,((int) p_line),5);
        
            else
            {
                rovio_turnRightByDegree(curl,counter); 
                //add a rovio forward command here
            }    
        }
    }
    private:
        double m_angle;
        double m_pLine;
        Direction m_side;

};