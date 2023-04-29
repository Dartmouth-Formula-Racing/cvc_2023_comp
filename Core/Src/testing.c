#include "main.h"

void run_tests() {
	canTestLoop();
}

void rtd_button_pressed() {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void rtd_button_released() {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}

void tc_switch_high() {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

}

void tc_switch_low() {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}
