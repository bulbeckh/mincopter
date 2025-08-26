#### MinCopter - State

Currently, the MCState class is instantiated directly and the AHRS and INAV classes can be overriden. This needs to change
as we now have the EKF class and will soon have other state estimation libraries/capabilities (e.g. SLAM, complementary filters).


Option 1: We can have the MCState be the base class and our various AHRS/INAV libraries can implement all of its methods.

Option 2: We keep MCState as the instance class and split up the AHRS and INAV which are themselves overriden. EKF for example
could implement/inherit both methods and be passed as the variable for both.


##### MCState Interface

**Variables**
```c
MC_AHRS_CLASS ahrs;

MC_INAV_CLASS inertial_nav;

Vector3f omega;

float cos_roll_x         = 1.0;
float cos_pitch_x        = 1.0;
float cos_yaw            = 1.0;
float sin_yaw;
float sin_roll;
float sin_pitch;

struct   Location home;
struct   Location current_loc;
```

**Methods**
```c
void update_trig(void);
void read_AHRS(void);
```



