#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

// helper functions for converting to/from radians/degrees
#define RAD(deg) ((double)(deg) * M_PI / 180.0)
#define DEG(rad) ((double)(rad) * 180.0 / M_PI)

#include "redis_keys.h"