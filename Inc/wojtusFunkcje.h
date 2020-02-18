#include "i2c-lcd.h"
#include "tim.h"
#include "arm_math.h"
#include "math_helper.h"
#include "bh1750.h"
#include "stdio.h"
#include "string.h"


void obliczPID(int varZ, int varO, arm_pid_instance_f32 * PID);
int odczytBH1750();
void obslugaLCD(int varZ, int varO);
