#ifndef PID_H_
#define PID_H_

#define l 0.240
#define k 0.1
#define m 0.98
#define Ix 0.0898
#define Iy 0.0647
#define Iz 0.0647
#define F (F1 + F2 + F3 + F4)
#define M_THETA (F4 - F1)
#define M_DELTA (F2 - F3)
#define M_PHI	(F1 + F4 - F2 - F3) / 4
#define k4 0.0010812

typedef struct {
	float theta;
	float phi;
	float psi;
} Angle;

typedef struct {
	float F1;
	float F2;
	float F3;
	float F4;
} FSet;

typedef struct {
	float n1;
	float n2;
	float n3;
	float n4;
} NSet;

typedef struct {
	float x;
	float y;
	float z;
} USet;

typedef struct {
	int ed;
	USet pos, velocity;
	Angle attitude, angleV;
	Angle prevAngleError;
	Angle queueAngle[10000];
	USet queue[10000];
	USet integral;
	USet prevError;
    float kp;
    float ki;
    float kd;
} pid_controller;

void init(pid_controller *pid);

void setPos(pid_controller *pid, float x, float y, float z);

void setParameter(pid_controller *pid, float kp, float ki, float kd);

void setAttitude(pid_controller *pid, float theta, float phi, float psi);

void workRotorSpeed(NSet prevRotorSpeed);

Angle workAttitudeAngle(USet acc);

void PIDProcess(pid_controller *pid, USet *dst);

Angle innerLoop(pid_controller *pid, Angle *dst);

USet updatePos(pid_controller *pid, USet *dst);

void updateAttitueAngle(pid_controller *pid, Angle *dst);

bool isOK(pid_controller *pid, USet *dst);

void printTrack(pid_controller *pid);

void fprintTrack(pid_controller *pid);

#endif