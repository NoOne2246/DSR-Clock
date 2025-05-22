#include "stepper.h"
#include "AccelStepper.h"
#include "TMC2209.h"

void steppers_initialize(stepper_controller_t *controller)
{
	controller->_num_steppers = 0;
}

uint8_t steppers_add_stepper(stepper_controller_t *controller, gpio_num_t stepPin, gpio_num_t dirPin, uart_port_t uartPort, gpio_num_t rxPin, gpio_num_t txPin, tmc_SerialAddress Address)
{

	if (controller->_num_steppers >= MULTISTEPPER_MAX_STEPPERS)
		return 0; // No room for more
	controller->_num_steppers++;
	accelstepper_t *stepper;
	astepper_initialize(stepper, 8, stepPin, dirPin, GPIO_NUM_NC, GPIO_NUM_NC, NULL);
	controller->_steppers[controller->_num_steppers] = stepper;
	TMC2209_t *driver;
	tmc_initialize(driver);
	tmc_setup(driver, uartPort, 115200, Address, rxPin, txPin);
	controller->_steppers[controller->_num_steppers] = driver;
	return 1;
}

uint8_t steppers_enableStepper(stepper_controller_t *controller, uint8_t stepper){
	if (stepper >= controller->_num_steppers)
		return 0; // Stepper does not exist
	tmc_enable(controller->_drivers[stepper]);
	return 1;
}


uint8_t steppers_disableStepper(stepper_controller_t *controller, uint8_t stepper){
	if (stepper >= controller->_num_steppers)
		return 0; // Stepper does not exist
	tmc_disable(controller->_drivers[stepper]);
	return 1;
}


uint8_t steppers_enableAll(stepper_controller_t* controller){
	for (int i = 0; i < controller->_num_steppers;i++){
		tmc_enable(controller->_drivers[i]);
	}
	return 1;
}

uint8_t steppers_disableAll(stepper_controller_t* controller){
	for (int i = 0; i < controller->_num_steppers;i++){
		tmc_disable(controller->_drivers[i]);
	}
	return 1;
}