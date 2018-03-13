#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include <ros/console.h>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#endif

#define INSERT_CHECKPOINT 0
#define TARGET_CURRENTCORNER 1
#define TARGET_NEWCORNER 2
#define TARGET_CENTERSTART 3





/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

    SearchController();

    void Reset() override;

    // performs search pattern
    Result DoWork() override;
    bool ShouldInterrupt() override;
    bool HasWork() override;

    // sets the value of the current location
    //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
    void SetCurrentLocation(Point currentLocation);
    void SetCenterLocation(Point centerLocation);
    void SetSuccesfullPickup();


    void SetCheckPoint();
    void ReachedCheckPoint();
    void ReachedSearchLocation();
    void SetSwarmSize(int size);
    void SetRoverIndex(int idx);
    void Print( Point x);
    Point Calculate(int grid ,int quad ,int x_y);
    bool CheckSearchPoint(Point check);
    float Cross(Point A, Point B, Point P);
    float AngleGenerator();
    float LengthGenerator();



    protected:

        void ProcessData();

private:


    bool check = false;
    int quad;
    Point oneLine;
    Point twoLine;
    bool init_A = false;
    bool init_B = false;
    random_numbers::RandomNumberGenerator* rng;
    Point currentLocation;
    Point centerLocation;
    Point searchLocation;
    Point PrevCanidate;
    Point checkPoint;
    int attemptCount = 0;
    //struct for returning data to ROS adapter
    Result result;

    // Search state
    // Flag to allow special behaviour for the first waypoint
    bool succesfullPickup = false;
    bool checkpointReached = true;
    bool searchlocationReached = false;

    bool centerSet = false;
    int roverID = 0;
    int swarmSize = 0;
    bool inQuad = false;


};

#endif /* SEARCH_CONTROLLER */
