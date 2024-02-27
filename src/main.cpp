/*
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"

//--------------Zephyr Drivers--------------------------------
#include "zephyr/console/console.h"


//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); //code to be executed in the slow communication task
void loop_application_task();   //code to be executed in the fast application task
void loop_critical_task();       //code to be executed in real-time at 20kHz


//--------------USER VARIABLES DECLARATIONS-------------------

enum serial_interface_menu_mode //LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE =0,
    SERIALMODE
    
};

uint8_t received_serial_char;
uint8_t mode = IDLEMODE;
static uint32_t counter = 0; //counter variable

//--------------SETUP FUNCTIONS-------------------------------

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    console_init();

    // Setup the hardware first
    spin.version.setBoardVersion(TWIST_v_1_1_2);

    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t comm_task_number = task.createBackground(loop_communication_task);
    //task.createCritical(loop_critical_task, 500); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(comm_task_number);
    //task.startCritical(); // Uncomment if you use the critical task
}

//--------------LOOP FUNCTIONS--------------------------------

/**
 * This is the code for the communication task.
 * it handles data exchanges with the outside world.
 * It has the lowest priority
*/
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char) {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" _____________________________________\n");
            printk("|     ------- MENU ---------          |\n");
            printk("|     press i : idle mode             |\n");
            printk("|     press s : serial mode           |\n");
            printk("|     press u : counter UP            |\n");
            printk("|     press d : counter DOWN          |\n");
            printk("|_____________________________________|\n\n");
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
        case 'u':
            counter++;
            break;
        case 'd':
            counter--;
            break;
        default:
            break;
    }

}

/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{
    if(mode==IDLEMODE) {
        spin.led.turnOff();

    }else if(mode==SERIALMODE) {
        spin.led.turnOn();
        printk("%d\n", counter);
    }
    
    task.suspendBackgroundMs(100); 
}
/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{

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
