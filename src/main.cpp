//-------------OWNTECH DRIVERS-------------------
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "DataAPI.h"
#include "opalib_control_pid.h"

//------------ZEPHYR DRIVERS----------------------
#include "zephyr/console/console.h"


//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_application_task();   // Code to be executed in the background task
void loop_communication_task();   // Code to be executed in the background task
void loop_control_task();     // Code to be executed in real time in the critical task

enum serial_interface_menu_mode //LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE =0,
    SERIALMODE,
    POWERMODE,
    BUCKMODE,
};

uint8_t received_serial_char;
uint8_t mode = IDLEMODE;

//--------------USER VARIABLES DECLARATIONS----------------------

static float32_t duty_cycle = 0.5; //[-] duty cycle (comm task)
static float32_t duty_cycle_step = 0.05; //[-] duty cycle step (comm task)
static bool pwm_enable = false; //[bool] state of the PWM (ctrl task)
static uint32_t control_task_period = 50; //[us] period of the control task

//Measurement data
static float32_t V1_low_value; //store value of V1_low (app task)
static float32_t V2_low_value; //store value of V2_low (app task)
static float32_t V_high_value; //store value of Vhigh (app task)

static float32_t I1_low_value; //store value of i1_low (app task)
static float32_t I2_low_value; //store value of i2_low (app task)
static float32_t I_high_value; //store value of ihigh (app task)

static float32_t meas_data; //temp storage meas value (ctrl task)
//
static float32_t voltage_reference = 10; //voltage reference (app task)
static int cpt_step = 0; //counter for voltage reference (app task)

static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

//---------------SETUP FUNCTIONS----------------------------------
void setup_routine()
{
    data.enableTwistDefaultChannels();
    spin.version.setBoardVersion(SPIN_v_0_9);
    twist.setVersion(shield_TWIST_V1_3);
    twist.initLegBuck(LEG1);
    twist.initLegBuck(LEG2);

    float32_t GV1 = 0.044301359147286994;
    float32_t OV1 = -89.8291125470221;
    float32_t GV2 = 0.043891466731813246;
    float32_t OV2 = -89.01321095039089;
    float32_t GVH = 0.029777494874229947;
    float32_t OVH = 0.12805533844297656;

    float32_t GI1 = 0.005510045850270965;
    float32_t OI1 = -11.298753103344417;
    float32_t GI2 = 0.005569903739753797;
    float32_t OI2 = -11.47851441455354;
    float32_t GIH = 0.0052774398156665;
    float32_t OIH = -10.864400298536168;

    data.setParameters(V1_LOW, GV1, OV1);
    data.setParameters(V2_LOW, GV2, OV2);
    data.setParameters(V_HIGH, GVH, OVH);

    data.setParameters(I1_LOW, GI1, OI1);
    data.setParameters(I2_LOW, GI2, OI2);
    data.setParameters(I_HIGH, GIH, OIH);

    uint8_t AppTask_num = task.createBackground(loop_application_task);
    uint8_t CommTask_num = task.createBackground(loop_communication_task);
    task.createCritical(&loop_control_task, control_task_period);

    opalib_control_init_leg1_pid(kp, ki, kd, control_task_period);
    opalib_control_init_leg2_pid(kp, ki, kd, control_task_period);

    task.startBackground(AppTask_num);
    task.startBackground(CommTask_num);
    task.startCritical();
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char) {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press s : serial mode              |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : duty cycle UP            |\n");
            printk("|     press d : duty cycle DOWN          |\n");
            printk("|     press b : closed-loop buck mode    |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------

            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 's':
            printk("serial mode\n");
            mode = SERIALMODE;
            break;
        case 'p':
            printk("power mode\n");
            mode = POWERMODE;
            break;
        case 'u':
            printk("duty cycle UP: %f\n", duty_cycle);
            duty_cycle = duty_cycle + duty_cycle_step;
            if(duty_cycle>1.0) duty_cycle = 1.0;
            break;
        case 'd':
            printk("duty cycle DOWN: %f\n", duty_cycle);
            duty_cycle = duty_cycle - duty_cycle_step;
            if(duty_cycle<0.0) duty_cycle = 0.0;
            break;
        case 'b':
            printk("closed-loop buck mode\n");
            mode = BUCKMODE;
            break;   
        default:
            break;
    }
}


void loop_application_task()
{
    if(mode==IDLEMODE) {
        spin.led.turnOff();
    }else if(mode==SERIALMODE) {
        spin.led.turnOn();
    }else if(mode==POWERMODE) {
        spin.led.toggle();
    }else if(mode==BUCKMODE) {
        spin.led.turnOn();
        if(cpt_step==10) voltage_reference = 15;
        if(cpt_step==20) {
            voltage_reference = 10;
            cpt_step=0;
        }
        cpt_step ++;
    }
       
    printk("%f:", duty_cycle);
    printk("%f:", V_high_value);
    printk("%f:", V1_low_value);
    printk("%f:", V2_low_value);
    printk("%f:", I_high_value);
    printk("%f:", I1_low_value);
    printk("%f\n", I2_low_value);

    task.suspendBackgroundMs(100); 
}


void loop_control_task()
{
    meas_data = data.getLatest(V1_LOW);
    if (meas_data != NO_VALUE)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != NO_VALUE)
        V2_low_value = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE)
        V_high_value = meas_data;

    meas_data = data.getLatest(I1_LOW);
    if (meas_data != NO_VALUE)
        I1_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != NO_VALUE)
        I2_low_value = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != NO_VALUE)
        I_high_value = meas_data;

    if(mode==IDLEMODE || mode==SERIALMODE) {
         pwm_enable = false;
         twist.stopAll();

    }else if(mode==POWERMODE || mode==BUCKMODE) {

        if(!pwm_enable) {
            pwm_enable = true;
            twist.startAll();
        }
    if(mode==BUCKMODE){
        duty_cycle = opalib_control_leg1_pid_calculation(voltage_reference, V1_low_value);
    }
    //Sends the PWM to the switches
    twist.setAllDutyCycle(duty_cycle);
    }
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */

int main(void)
{
    setup_routine();
    return 0;
}