#include "SearchController.h"

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
    if(!init_A){
        cout << "tag: Startup -- Setting up the Quadrants" << endl;
        init_A = true;

        // determine how the grid would be set up and what lines the rovers will be confined in
        if(swarmSize > 4)
        {
            cout << "tag: size is 4 or greater " << endl;
            int sub = roverID;

            while(sub > 4){

                sub -= 4;
            }

            quad = sub;
            cout << "tag: swarmize = " << swarmSize <<" , Quadrant = "<< quad <<endl;

            // lines used in cross product math
            oneLine = Calculate(4,quad, 0);
            twoLine = Calculate(4,quad, 1);

        }
        else {

            cout << "tag: size is less than 4" << endl;

            quad = roverID;
            cout << "tag: Quadrant = "<< quad << endl;

            // lines used in cross product math
            oneLine = Calculate(swarmSize,quad, 0);
            //cout << "tag: Location is Good" << endl;
            twoLine = Calculate(swarmSize,quad, 1);

        }
    }

    if(succesfullPickup){
        succesfullPickup = false;
        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), checkPoint );
        searchLocation = checkPoint;

        return result;
    }else{

        Point canidate;

        while(!inQuad){
            float length;
            float angle;
            angle = AngleGenerator();
            length = LengthGenerator();
            cout << "tag: random:" <<rng->uniformInteger(1,4) << endl;
            canidate.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
            canidate.x = currentLocation.x + (length * cos(angle));
            canidate.y = currentLocation.y + (length * sin(angle));
            cout << "tag: Canidate location (X,Y): ("<< canidate.x <<" , "<< canidate.y << ")" << endl;
            if(CheckSearchPoint(canidate)){
                inQuad = true;
                PrevCanidate = canidate;
                cout << "tag: Location is Good" << endl;
            }else{
                cout << "tag: Location is Bad" << endl;
            }
        }

        searchLocation = canidate;

        inQuad = false;

        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

        return result;
    }
}

Point SearchController::Calculate(int grid , int quad, int x_y){
    Point xOrY;

    if(grid == 1){
        xOrY.x = 0;
        xOrY.y = 0;

        return xOrY;


    } else if(grid == 2 ){
        // two rovers -- two quadrants -- dived on the x axis
        switch (quad) {
        case 1:{

            if(x_y == 0){

                xOrY.x = centerLocation.x;
                xOrY.y = centerLocation.y + 100;

                return xOrY;

            }else{

                xOrY.x = centerLocation.x;
                xOrY.y = centerLocation.y - 100;

                return xOrY;
            }

            break;

        }
        case 2:{

            if(x_y == 0){

                xOrY.x = centerLocation.x;
                xOrY.y = centerLocation.y + 100;


                return xOrY;

            }else{

                xOrY.x = centerLocation.x;
                xOrY.y = centerLocation.y - 100;

                return xOrY;
            }


            break;
        }

        }

    }else if(grid == 3 ){
        // three rovers -- three quadrants -- divied along the 30, 150, 270 degree lines
        switch (quad) {
        case 1:{

            if(x_y == 0){
                // 30
                xOrY.x = centerLocation.x + (100 * cos(0.523599));
                xOrY.y = centerLocation.y + (100 + sin(0.523599));

                return xOrY;

            }else{

                // 270
                xOrY.x = centerLocation.x + (100 * cos(4.71239));
                xOrY.y = centerLocation.y + (100 + sin(4.71239));


                return xOrY;
            }

            break;

        }
        case 2:{

            if(x_y == 0){
                // 150
                xOrY.x = centerLocation.x + (100 + cos(2.61799));
                xOrY.y = centerLocation.y + (100 + sin(2.61799));

                return xOrY;

            }else{
                // 30
                xOrY.x = centerLocation.x + (100 * cos(0.523599));
                xOrY.y = centerLocation.y + (100 + sin(0.523599));


                return xOrY;
            }


            break;
        }
        case 3:{
            if(x_y == 0){
                // 150
                xOrY.x = centerLocation.x + (100 + cos(2.61799));
                xOrY.y = centerLocation.y + (100 + sin(2.61799));


                return xOrY;

            }else{
                // 270
                xOrY.x = centerLocation.x + (100 * cos(4.71239));
                xOrY.y = centerLocation.y + (100 + sin(4.71239));

                return xOrY;
            }
        }

        }


    }else if(grid == 4 ){
        // four+ rovers -- four quadrants -- divied along the x and y axis

        switch (quad) {
        case 1:{

            if(x_y == 0){
                // 0
                xOrY.x = centerLocation.x + (100 + cos(0));
                xOrY.y = centerLocation.y + (100 + sin(0));

                return xOrY;

            }else{

                // 90
                xOrY.x = centerLocation.x + (100 * cos(1.5708));
                xOrY.y = centerLocation.y + (100 + sin(1.5708));


                return xOrY;
            }

            break;

        }
        case 2:{

            if(x_y == 0){
                // 90
                xOrY.x = centerLocation.x + (100 * cos(1.5708));
                xOrY.y = centerLocation.y + (100 + sin(1.5708));

                return xOrY;

            }else{
                // 180
                xOrY.x = centerLocation.x + (100 * cos(3.14159));
                xOrY.y = centerLocation.y + (100 + sin(3.14159));

                return xOrY;
            }


            break;
        }
        case 3:{
            if(x_y == 0){
                // 180
                xOrY.x = centerLocation.x + (100 * cos(3.14159));
                xOrY.y = centerLocation.y + (100 + sin(3.14159));


                return xOrY;

            }else{
                // 270
                xOrY.x = centerLocation.x + (100 * cos(4.71239));
                xOrY.y = centerLocation.y + (100 + sin(4.71239));

                return xOrY;
            }
        }
        case 4:{
            if(x_y == 0){
                // 270
                xOrY.x = centerLocation.x + (100 * cos(4.71239));
                xOrY.y = centerLocation.y + (100 + sin(4.71239));


                return xOrY;

            }else{
                // 0
                xOrY.x = centerLocation.x + (100 + cos(0));
                xOrY.y = centerLocation.y + (100 + sin(0));

                return xOrY;
            }
        }

        }

    }

}

bool SearchController::CheckSearchPoint(Point check){

    if(swarmSize == 1){
        return true;

    }
    else if(swarmSize == 2){
        float d = Cross(oneLine,twoLine,check);
        if(quad == 1){
            if(d < 0.0){
                return false;
            }else{
                return true;
            }
        }else{
            if(d < 0.0){
                return true;
            }else{
                return false;
            }
        }

    }
    else if(swarmSize == 3){
        float d1 = Cross(centerLocation,oneLine,check);
        float d2 = Cross(centerLocation,twoLine,check);
        switch (quad) {
        case 1:{
            if(d1 > 0.0 && d2 < 0.0){
                return true;
            }else{
                return false;
            }

            break;
        }
        case 2:{
            if(d1 > 0.0 && d2 < 0.0){
                return true;
            }else{
                return false;
            }
            break;
        }
        case 3:{
            if(d1 < 0.0 && d2 > 0.0){
                return true;
            }else{
                return false;
            }
            break;
        }

        }

    }
    else if(swarmSize == 4){

        float d1 = Cross(centerLocation,oneLine,check);
        float d2 = Cross(centerLocation,twoLine,check);
        switch (quad) {
        case 1:{

            if(d1 < 0.0 && d2 > 0.0){

                return true;

            }else{

                return false;
            }

            break;
        }
        case 2:{

            if(d1 < 0.0 && d2 > 0.0){

                return true;
            }else{

                return false;
            }

            break;
        }
        case 3:{

            if(d1 < 0.0 && d2 > 0.0){

                return true;

            }else{

                return false;

            }

            break;
        }
        case 4:{

            if(d1 < 0.0 && d2 > 0.0){

                return true;

            }else{

                return false;

            }

            break;
        }
        }

    }


}



float SearchController::Cross(Point A, Point B, Point P){

    float product;

    float V = (P.x - A.x)*(B.y - A.y);
    float V1 = (P.y - A.y)*(B.x - A.x);
    product = V - V1;

    return product;
}

float SearchController::AngleGenerator(){

    int random = rng->uniformInteger(1,8);
    float angle;

    switch (random) {
    case 1:{

        // 360 or 0
        angle = 0.000;
        return angle;
        break;

    }
    case 2:{

        // 90
        angle = 1.5708;
        return angle;
        break;

    }
    case 3:{

        //180
        angle = 3.14159;
        return angle;
        break;

    }
    case 4:{

        //0
        angle = 4.71239;
        return angle;
        break;

    }
    case 5:{

        // 45
        angle = 0.785398;
        return angle;
        break;

    }
    case 6:{

        // 135
        angle = 2.35619;
        return angle;
        break;

    }
    case 7:{

        //225
        angle = 3.92699;
        return angle;
        break;

    }
    case 8:{

        //315
        angle = 5.49779;
        return angle;
        break;

    }


    }
}

float SearchController::LengthGenerator(){
    int random = rng->uniformInteger(1,4);
    float len;


    switch (random) {
    case 1:{


        len = 0.50;
        return len;
        break;

    }
    case 2:{


        len = 1.00;
        return len;
        break;

    }
    case 3:{


        len = 1.50;
        return len;
        break;

    }
    case 4:{


        len = 2.00;
        return len;
        break;

    }


    }
}

void SearchController::SetCheckPoint(){
    checkPoint = currentLocation;

}

void SearchController::ReachedCheckPoint(){
    if (hypot(checkPoint.x-currentLocation.x, checkPoint.y-currentLocation.y) < 0.10) {
        checkpointReached = true;
        cout << "tag: reached the checkpoint(): "<< checkPoint.x<< " , "<< checkPoint.y<< endl;
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



void SearchController::ReachedSearchLocation(){
    if (hypot(searchLocation.x-currentLocation.x, searchLocation.y-currentLocation.y) < 0.10) {
        searchlocationReached = true;
        cout << "tag: reached the Searchlocation(): " << searchLocation.x<< " , "<< searchLocation.y<< endl;
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














