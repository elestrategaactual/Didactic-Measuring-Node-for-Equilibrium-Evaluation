#include <Arduino.h>
#include <Wire.h>
#include "GenImu.h"

#ifdef __MBED__
#include "mbed_events.h"
#include "mbed_shared_queues.h"
#include "drivers/InterruptIn.h"

#endif

