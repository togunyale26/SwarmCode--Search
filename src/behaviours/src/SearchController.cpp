﻿#include "SearchController.h"

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    currentLocation.x = 0;
    currentLocation.y = 0;
    currentLocation.theta = 0;

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocation.theta = 0;
    result.PIDMode = FAST_PID;


    result.type = waypoint;

}

void SearchController::Reset() {
    result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork()
{

    int searchState;

    if(!init_A)
    {
        init_A = true;

        centerStart.x = centerLocation.x;
        centerStart.y = centerLocation.y + (roverID * 1.308);

        spiralLocation.x = centerStart.x;
        spiralLocation.y = centerStart.y;
        searchLocation.x = spiralLocation.x;
        searchLocation.y = spiralLocation.y;


        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), centerStart);
        stepsIntoSpiral +=1;
        return result;


    }
    else
    {


        ReachedCenterStart();
        ReachedCheckPoint();
        ReachedSearchLocation();
        if(centerStartReached)
        {
            if(succesfullPickup)
            {
                searchState = INSERT_CHECKPOINT;
            }

            else if (checkpointReached)
            {
                searchState = TARGET_CURRENTCORNER;
                if(searchlocationReached)
                {
                    searchState = TARGET_NEWCORNER;
                }

            }
        }else{
            searchState = TARGET_CENTERSTART;
        }



        switch(searchState)
        {
        case INSERT_CHECKPOINT:{
            succesfullPickup = false;
            result.wpts.waypoints.clear();
            result.wpts.waypoints.insert(result.wpts.waypoints.end(), checkPoint);
            return result;
            break;

        }
        case TARGET_CURRENTCORNER:{
            result.wpts.waypoints.clear();
            result.wpts.waypoints.insert(result.wpts.waypoints.end(), searchLocation);
            return result;
            break;

        }
        case TARGET_NEWCORNER:{
            searchlocationReached = false;
            result.wpts.waypoints.clear();
            searchLocation = SpiralSearching();
            return result;
            break;

        }
        case TARGET_CENTERSTART:{
            result.wpts.waypoints.clear();
            result.wpts.waypoints.insert(result.wpts.waypoints.end(), centerStart);
            return result;
            break;
        }
        }
    }
}























void SearchController::SetCenterLocation(Point centerLocation) {

    if(!centerSet){
        centerSet = true;
        float diffX = this->centerLocation.x - centerLocation.x;
        float diffY = this->centerLocation.y - centerLocation.y;
        this->centerLocation = centerLocation;

        if (!result.wpts.waypoints.empty())
        {
            result.wpts.waypoints.back().x -= diffX;
            result.wpts.waypoints.back().y -= diffY;
        }

    }
}

Point SearchController::SpiralSearching()
{
    cornerNum += 1;
    if( cornerNum == 4)
    {
        cornerNum = 0;
        stepsIntoSpiral++;
    }

    sideLength = spacing * CalculateSides(stepsIntoSpiral, cornerNum);

    spiralLocation.x = spiralLocation.x + (sideLength * cos(corner));
    spiralLocation.y = spiralLocation.y + (sideLength * sin(corner));

    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), spiralLocation);

    corner -= (M_PI/2);

    if(corner <= 0.0)
    {
        corner += 2*M_PI;
    }

    return spiralLocation;
}

float SearchController::CalculateSides(int circuitNum, int slot){
    if( slot == 0 || slot == 1)
    {
        if(circuitNum == 0)
        {
            return roverID;
        }
        else if(circuitNum == 1)
        {
            sideLength = CalculateSides(0,slot) + swarmSize + roverID + 1.308;
            return sideLength;
        }
        else if(circuitNum > 1)
        {
            sideLength = CalculateSides(circuitNum - 1, slot) + 2 * swarmSize;
            return sideLength;
        }
    }
    else if(slot == 2 || slot == 3)
    {
        if(circuitNum == 0)
        {
            sideLength = CalculateSides(circuitNum,0) + roverID;
            return sideLength;
        }
        else if(circuitNum > 0)
        {
            sideLength = CalculateSides(circuitNum, 0) + swarmSize;
            return sideLength;
        }
    }
}

void SearchController::SetCurrentLocation(Point currentLocation) {
    this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
    ProcessData();

    return false;
}

bool SearchController::HasWork() {
    return true;
}

void SearchController::SetSuccesfullPickup() {
    succesfullPickup = true;
    if(checkpointReached){
        SetCheckPoint();
        checkpointReached =false;

    }


}

void SearchController::SetCheckPoint(){
    // or set it to current location
    this->checkPoint = this->currentLocation;
    cout << "tag: locating which side of the spiral am I" << endl;


    if(cornerNum == 0){
        cout << "tag: West side of the square " << endl;
        this->checkPoint.y -= 1.0;
        this->checkPoint.x = searchLocation.x;

    }else if(cornerNum == 1){
        cout << "tag: North side of the square" << endl;
        this->checkPoint.x -= 1.0;
        this->checkPoint.y = searchLocation.y;

    }else if(cornerNum == 2){
        cout << "tag: East side of the square" << endl;
        this->checkPoint.y += 1.0;
        this->checkPoint.x = searchLocation.x;

    }else if(cornerNum == 3){
        cout << "tag: South side of the square" << endl;
        this->checkPoint.x += 1.0;
        this->checkPoint.y = searchLocation.y;
    }
    cout << "tag: CHECKPOINT LOCATION: "<< checkPoint.x << " , "<< checkPoint.y << endl;

}











void SearchController::ReachedCheckPoint(){
    if (hypot(checkPoint.x-currentLocation.x, checkPoint.y-currentLocation.y) < 0.10) {
        checkpointReached = true;
        cout << "tag: reached the checkpoint(): "<< checkPoint.x<< " , "<< checkPoint.y<< endl;
    }

}






void SearchController::ReachedSearchLocation(){
    if (hypot(searchLocation.x-currentLocation.x, searchLocation.y-currentLocation.y) < 0.10) {
        searchlocationReached = true;
        cout << "tag: reached the Searchlocation(): " << searchLocation.x<< " , "<< searchLocation.y<< endl;
    }

}

void SearchController::ReachedCenterStart(){
    if (hypot(centerStart.x-currentLocation.x, centerStart.y-currentLocation.y) < 0.10) {
        centerStartReached = true;
        cout << "tag: reached the Searchlocation(): " << centerStart.x<< " , "<< centerStart.y<< endl;
    }
}



void SearchController::SetRoverIndex(int idx){
    roverID = idx + 1;
    cout << "tag:"<< "RoverIndex: "<< roverID << endl;
}






void SearchController::SetSwarmSize(int size){
    swarmSize = size;
    cout << "tag:"<< "SwarmSize: "<< swarmSize << endl;
}














