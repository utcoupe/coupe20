/** Includes **/
/**************/
#include "serial.h"
#include <string.h>
using namespace std;

/** Constructor **/
/*****************/
Serial::Serial(UART_HandleTypeDef* serial)
{
	m_serial_interface_ptr = serial;
	m_cmd_cursor = 0;
	m_cmd_ready = false;
	//__HAL_UART_ENABLE_IT(m_serial_interface_ptr,UART_IT_RXNE);
}

Serial::~Serial()
{
	delete m_serial_interface_ptr;
}

void Serial::write(uint32_t value)
{
	uint8_t buf[sizeof(uint32_t)];
	uint8_t i;
	for( i=0; i < sizeof(uint32_t); i++)
	{
		buf[i] = value >> 8*(sizeof(uint32_t)-i-1);
		write(buf[i]);
	}
}



void Serial::write(uint16_t value)
{
	uint8_t buf[sizeof(uint16_t)];
	uint8_t i;
	for( i=0; i < sizeof(uint16_t); i++)
	{
		buf[i] = value >> 8*(sizeof(uint16_t)-i-1);
		write(buf[i]);
	}
}

void Serial::write(uint8_t value)
{
	HAL_StatusTypeDef status =HAL_UART_Transmit(m_serial_interface_ptr, &value,1,1);
	// switch(status)
	// {

	// 	case HAL_OK:
	// 	    print("SENT OK\n");
	// 		break;

	// 	case HAL_ERROR:
	// 		print("SENT ER\n");
	// 		break;

	// 	case HAL_BUSY:
	// 		print("SENT BY\n");
	// 		break;

	// 	case HAL_TIMEOUT:
	// 		print("SENT TO\n");
	// 		break;

	// 	default:
	// 		print("UNKNOWN\n");
	// 		break;
	// }
}

void Serial::write(uint8_t* msg,uint16_t len)
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(m_serial_interface_ptr, msg, len,1);
	// switch(status)
	// {
	// 	case HAL_OK:
	// 	    print("SENT OK\n");
	// 		break;

	// 	case HAL_ERROR:
	// 		print("SENT ER\n");
	// 		break;

	// 	case HAL_BUSY:
	// 		print("SENT BY\n");
	// 		break;

	// 	case HAL_TIMEOUT:
	// 		print("SENT TO\n");
	// 		break;

	// 	default:
	// 		print("UNKNOWN\n");
	// 		break;
	// }
}
void Serial::print(char* msg)
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(m_serial_interface_ptr, (uint8_t*)msg, strlen(msg),1);
	// switch(status)
	// {


	// case HAL_OK:
	//     print("print OK\n");
	//     // HAL_UART_Transmit(&huart2, "CAN SEND OK", 12, 0xFFFF);
	// 	break;

	// case HAL_ERROR:
	// 	print("print ER\n");
	// 	break;

	// case HAL_BUSY:
	// 	print("print BY\n");
	// 	// HAL_UART_Transmit(&huart2, "CAN SEND BS", 12, 0xFFFF);
	// 	// strcpy(msg, "CAN BUSY\n");
	// 	// HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	// 	break;

	// case HAL_TIMEOUT:
	// 	print("print TO\n");
	// 	// HAL_UART_Transmit(&huart2, "CAN SEND TO", 12, 0xFFFF);
	// 	// strcpy(msg, "CAN TIMEOUT\n");
	// 	// HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
	// 	break;

	// default:
	// 	print("UNKNOWN print\n");
	// 	break;

	// }
}

void Serial::print(uint32_t value)
{
	char buf[11];
	snprintf(buf, sizeof buf, "%lu", (unsigned long)value);
	print(buf);
}

void Serial::print(uint16_t value)
{
	char buf[6];
	snprintf(buf, sizeof buf, "%u", value);
	print(buf);
}

void Serial::print(uint8_t value)
{
	char buf[4];
	snprintf(buf, sizeof buf, "%hu", value);
	print(buf);
}

void Serial::print(int32_t value)
{
	char buf[12];
	snprintf(buf, sizeof buf, "%li", value);
	print(buf);
}

void Serial::print(int16_t value)
{
	char buf[7];
	snprintf(buf, sizeof buf, "%i", value);
	print(buf);
}

void Serial::print(int8_t value)
{
	char buf[5];
	snprintf(buf, sizeof buf, "%hi", value);
	print(buf);
}

void Serial::read()
{
	uint8_t rx_char;
	HAL_StatusTypeDef status;
	status = HAL_UART_Receive(m_serial_interface_ptr, &rx_char, 1, 1);

	if (status == HAL_OK)
	{
		if ((char)rx_char == '\n' || m_cmd_cursor >= MAX_CMD_SIZE - 1)
		{
			m_cmd_ready = true;
			m_cmd[m_cmd_cursor] =0;
			m_cmd_cursor =0;

		}
		else
		{
			m_cmd_ready = false;
			m_cmd[m_cmd_cursor] = (char)rx_char;
			m_cmd_cursor ++;
		}
	}
	// switch(status)
	// {
	//   case HAL_OK:
	//   print("OK\n");
	//   // HAL_GPIO_WritePin(GPIOB, TEST_LED_Pin, GPIO_PIN_RESET);
	//   break;

	//   case HAL_ERROR:
	//   print("ERROR\n");
	//   // HAL_GPIO_WritePin(GPIOB, TEST_LED_Pin, GPIO_PIN_RESET);
	//   break;

	//   case HAL_BUSY:
	//   print("BUSY\n");
	//   // HAL_GPIO_WritePin(GPIOB, TEST_LED_Pin, GPIO_PIN_RESET);
	//   break;

	//   case HAL_TIMEOUT:
	//   print("TIMEOUT\n");
	//   // HAL_GPIO_WritePin(GPIOB, TEST_LED_Pin, GPIO_PIN_SET);
	//   break;

	// }
	// print("\n");
	// __HAL_UART_SEND_REQ(m_serial_interface_ptr, UART_RXDATA_FLUSH_REQUEST);
	// return ((char*)&rx_char);
}

uint16_t Serial::available()
{
	return m_serial_interface_ptr->RxXferSize;
}

bool Serial::cmd_is_ready()
{
	return m_cmd_ready;
}

char* Serial::get_cmd()
{
	m_cmd_ready = false;
	return m_cmd;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(GPIOB, TEST_LED_Pin);
	// __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
	// if (huart->RxState != HAL_UART_STATE_READY)
	// {
 //  		huart->RxState = HAL_UART_STATE_READY;
	// }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, TEST_LED_Pin);
		HAL_Delay(200);
	}
}