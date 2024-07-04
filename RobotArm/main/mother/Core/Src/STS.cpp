#include "STS.h"
#include "math.h"

STS::STS(UART_HandleTypeDef *uart, uint8_t id)
{
  UART = uart;
  ID = id;
}

void STS::moveCont(uint16_t abs_speed, int16_t goal_position, int16_t now_position)
{
  uint8_t send_Buf[9] = {255, 255, ID, 5, 3, 46, 0, 0, 0};
  uint8_t checksum = 0;
  uint16_t diff = 0;
  int16_t inst_speed = 0;
  int16_t speed = 0;

  diff = abs(goal_position - now_position);

  if(diff < 10){
	  inst_speed = 0;
  }else if(diff < abs_speed * abs_speed / 6000){
	  inst_speed = sqrt(1500 * diff);
  }else{
	  inst_speed = abs_speed;
  }
  if(goal_position < now_position){
	  speed = inst_speed;
  }else{
	  speed = -inst_speed;
  }

  if ((goal_position - now_position < 0 && speed > 0) || (goal_position - now_position > 0 && speed < 0))
  {
    if (speed < 0)
    {
      speed = 32768 - speed;
    }
    send_Buf[6] = speed;
    send_Buf[7] = speed >> 8;

    HAL_Delay(1);
  }

  for (int i = 2; i < 8; i++)
  {
    checksum += send_Buf[i];
  }
  checksum = ~checksum;
  send_Buf[8] = checksum;

  HAL_UART_Transmit(UART, send_Buf, 9, 50);
  HAL_Delay(1);
}

void STS::moveStop1(int16_t speed, int16_t goal_position){
	uint8_t send_Buf[13] = {255, 255, ID, 9, 3, 42, 0, 0, 0, 0, 0, 0, 0};
	uint8_t checksum = 0;

	goal_position = 4095 - goal_position;

	send_Buf[6] = goal_position;
	send_Buf[7] = goal_position >> 8;

	send_Buf[10] = speed;
	send_Buf[11] = speed >> 8;

	for (int i = 2; i < 12; i++)
	{
	checksum += send_Buf[i];
	}
	checksum = ~checksum;
	send_Buf[12] = checksum;

	HAL_UART_Transmit(UART, send_Buf, 13, 50);
	HAL_Delay(1);
}

void STS::moveStop3(int16_t speed, int16_t goal_position){
	uint8_t send_Buf[13] = {255, 255, ID, 9, 3, 42, 0, 0, 0, 0, 0, 0, 0};
	uint8_t checksum = 0;

	send_Buf[6] = goal_position;
	send_Buf[7] = goal_position >> 8;

	send_Buf[10] = speed;
	send_Buf[11] = speed >> 8;

	for (int i = 2; i < 12; i++)
	{
	checksum += send_Buf[i];
	}
	checksum = ~checksum;
	send_Buf[12] = checksum;

	HAL_UART_Transmit(UART, send_Buf, 13, 50);
	HAL_Delay(1);
}

void STS::send()
{
  uint8_t send_Buf[8] = {255, 255, ID, 4, 2, 56, 2, 0};
  uint8_t checksum = 0;

  for (int i = 2; i < 7; i++)
  {
    checksum += send_Buf[i];
  }
  checksum = ~checksum;
  send_Buf[7] = checksum;

  HAL_UART_Transmit(UART, send_Buf, 8, 100);
}


int16_t STS::calculate_position(int16_t now_position)
{
  if ((now_position - pre_position) > 3000)
  {
    rotation--;
  }
  if ((pre_position - now_position) > 3000)
  {
    rotation++;
  }

  position = now_position + rotation * 4096;

  pre_position = now_position;

  return -position;
}
