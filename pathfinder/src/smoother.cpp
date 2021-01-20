#include "include/pathfinder/smoother.hpp"

Smoother::Smoother(Grid* grid)
{
    this->grid = grid;
}
vector<VehicleState> Smoother::smooth(vector<VehicleState> path)
{
    if(path.size() < 5)
    {
        return path;
    }
    int count = 0;
    vector<VehicleState> current = path;
    while(count < maxIterations)
    {
        for(int i = 2; i < current.size()-2; i++)
        {
            if((current[i-2].gear != current[i-1].gear) || (current[i-1].gear != current[i].gear) || (current[i].gear != current[i+1].gear) || (current[i+1].gear != current[i+2].gear))
            {
                continue;
            }
            Coordinates2D position = Coordinates2D{current[i].posX, current[i].posY}; //Xi
            Coordinates2D correction = Coordinates2D();
            //ORIGINAL PATH
            correction.x += weightPath * (path[i].posX - position.x);
            correction.y += weightPath * (path[i].posY - position.y);

            Coordinates2D nearestObstPosition = grid->getNearestObstDist(position.x, position.y); //Oi
            Coordinates2D closestObstVec = Coordinates2D{position.x - nearestObstPosition.x, position.y - nearestObstPosition.y}; //Xi - Oi
            float obstDist = closestObstVec.x * closestObstVec.x + closestObstVec.y*closestObstVec.y; //|Xi - Oi|
            //OBST TERM
            if(obstDist <= obstDMax)
            {
                float scale = weightObst * 2*(obstDist - obstDMax);
                correction.x -= scale*closestObstVec.x/obstDist;
                correction.y -= scale*closestObstVec.y/obstDist;
            }
            //VORO
            if(obstDist < voroDMax && weightVoro > 0)
            {
                Coordinates2D nearestVoroPosition = grid->getNearestVoroDist(position.x, position.y); //Vi
                Coordinates2D closestVoroVec = Coordinates2D{position.x - nearestVoroPosition.x, position.y - nearestVoroPosition.y}; //Xi - Vi
                float voroDist = closestVoroVec.x * closestVoroVec.x + closestVoroVec.y*closestVoroVec.y; //|Xi - Vi|
                float alphaPlusObstDist = grid->alphaDefault + obstDist;
                float obstDistMinusDMax = obstDist - voroDMax;
                float obstDistPlusVoroDist = obstDist + voroDist;
                float dMaxSqrd = voroDMax * voroDMax;
                float pvdv = (grid->alphaDefault/alphaPlusObstDist)*((obstDistMinusDMax*obstDistMinusDMax)/(dMaxSqrd))*(obstDist/(obstDistPlusVoroDist+obstDistPlusVoroDist));
                float pvdo = (grid->alphaDefault/alphaPlusObstDist)*(voroDist/obstDistPlusVoroDist)*((obstDistMinusDMax*obstDistMinusDMax)/(dMaxSqrd))*((-obstDistMinusDMax/alphaPlusObstDist)-(obstDistMinusDMax/obstDistPlusVoroDist)+2);
                
                correction.x -= weightVoro * (pvdo * closestObstVec.x/obstDist + pvdv * closestVoroVec.x/voroDist);
                correction.y -= weightVoro * (pvdo * closestObstVec.y/obstDist + pvdv * closestVoroVec.y/voroDist);

            }
            //SMOOTH
            correction.x -= weightSmooth * (current[i - 2].posX - 4 * current[i - 1].posX + 6 * position.x - 4 * current[i + 1].posX + current[i + 2].posX);
            correction.y -= weightSmooth * (current[i - 2].posY - 4 * current[i - 1].posY + 6 * position.y - 4 * current[i + 1].posY + current[i + 2].posY);
            //CURVE
            Coordinates2D xiMinusOne = Coordinates2D{current[i-1].posX, current[i-1].posY};
            Coordinates2D xiPlusOne = Coordinates2D{current[i+1].posX, current[i+1].posY};
            Coordinates2D negxiPlusOne = Coordinates2D{-current[i+1].posX, -current[i+1].posY};
            Coordinates2D deltaXi = Coordinates2D{position.x-xiMinusOne.x, position.y-xiMinusOne.y};
            Coordinates2D deltaXiPlusOne = Coordinates2D{xiPlusOne.x-position.x, xiPlusOne.y-position.y};

            float distDeltaXi = deltaXi.x*deltaXi.x + deltaXi.y*deltaXi.y;
            float distDeltaXiPlusOne = deltaXiPlusOne.x*deltaXiPlusOne.x + deltaXiPlusOne.y*deltaXiPlusOne.y;
            float deltaPhi = acos((deltaXi.x*deltaXiPlusOne.x+deltaXi.y+deltaXiPlusOne.y) / (distDeltaXi*distDeltaXiPlusOne));
            float k = deltaPhi / distDeltaXi;
            if(k <= kMax)
            {
                float derivdeltaPhi = (-1/sqrt(1-(cos(deltaPhi)*cos(deltaPhi))));
                float diffdenom = distDeltaXi*distDeltaXiPlusOne;
                Coordinates2D ortho1 = orthogonalVector(position, negxiPlusOne);
                Coordinates2D ortho2 = orthogonalVector(negxiPlusOne, position);
                Coordinates2D pOne = Coordinates2D{ortho1.x/diffdenom, ortho1.y/diffdenom};
                Coordinates2D pTwo = Coordinates2D{ortho2.x/diffdenom, ortho2.y/diffdenom};
                
                float firstVar = -1/distDeltaXi;
                float secondVar = deltaPhi/(distDeltaXi*distDeltaXi);

                Coordinates2D kixi = Coordinates2D{firstVar*derivdeltaPhi*(-pOne.x-pTwo.x)-secondVar*1, firstVar*derivdeltaPhi*(-pOne.y-pTwo.y)-secondVar*1};
                Coordinates2D kixim = Coordinates2D{firstVar*derivdeltaPhi*pTwo.x-secondVar*1, firstVar*derivdeltaPhi*pTwo.y-secondVar*1};
                Coordinates2D kixip = Coordinates2D{firstVar*derivdeltaPhi*pOne.x, firstVar*derivdeltaPhi*pOne.y};

                correction.x -= weightCurve * (k-kMax) * (.5*kixi.x + .25*kixim.x + .25*kixip.x);
                correction.y -= weightCurve * (k-kMax) * (.5*kixi.y + .25*kixim.y + .25*kixip.y);
            }


            //NORMALIZATION AND APPLICATION
            correction.x /= (weightSmooth+weightVoro+weightObst+weightPath+weightCurve);
            correction.y /= (weightSmooth+weightVoro+weightObst+weightPath+weightCurve);
            current[i].posX = position.x + .2*correction.x;
            current[i].posY = position.y + .2*correction.y;
        }
        count++;
    }
    //Fix Orientations lost in smoothing
    for(int i = 1; i < current.size()-1; i++)
    {
        float deltay = current[i+1].posY - current[i-1].posY;
        float deltax = current[i+1].posX - current[i-1].posX;
        current[i].ori = atan2(deltay,deltax);
        if(current[i].gear == Backward)
        {
            current[i].ori -= M_PI;
        }

        if(current[i].ori < 0)
        {
            current[i].ori += 2*M_PI;
        }
        else if(current[i].ori > 2*M_PI)
        {
            current[i].ori -= 2*M_PI;
        }
    }

    return current;
}

Coordinates2D Smoother::orthogonalVector(Coordinates2D a, Coordinates2D b)
{
    Coordinates2D newVector = Coordinates2D();
    newVector.x = a.x - (a.x*-b.x + a.y*-b.y)/(b.x*b.x+b.y*b.y)*b.x/(b.x*b.x+b.y*b.y);
    newVector.y = a.y - (a.x*-b.x + a.y*-b.y)/(b.x*b.x+b.y*b.y)*b.y/(b.x*b.x+b.y*b.y);
    return newVector;
}
