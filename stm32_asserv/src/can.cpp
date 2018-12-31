/** Includes **/
/**************/
#include "can.h"

/** Constructor **/
/*****************/
uint16_t g_nb_msg_received = 0;

Can::Can(CAN_HandleTypeDef* can, uint16_t id)
{
	m_can_interface_ptr = can;


	// tx message init
	m_tx_msg.StdId = id;
  	m_tx_msg.ExtId = id;
	m_tx_msg.IDE = CAN_ID_STD;
	m_tx_msg.RTR = CAN_RTR_DATA;
	m_tx_msg.DLC = 8;
	
	// rx message init  	
	m_rx_msg.DLC = 8;
	m_rx_msg.IDE = CAN_ID_STD;
	m_rx_msg.RTR = CAN_RTR_DATA;
	m_rx_msg.FMI = 0;
	m_rx_msg.FIFONumber = CAN_FIFO0;

	m_can_interface_ptr->pTxMsg = &m_tx_msg;
	m_can_interface_ptr->pRxMsg = &m_rx_msg;
	copy_msg(m_tx_msg.Data,prev_msg,8);

	

}

Can::~Can()
{
	delete m_can_interface_ptr;
}
/** Public Methods **/
/********************/

HAL_StatusTypeDef Can::write(uint8_t* msg)
{
 
  copy_msg(m_tx_msg.Data,msg,8);
  
  return HAL_CAN_Transmit_IT(m_can_interface_ptr);
  
}

uint8_t* Can::read()
{
	g_nb_msg_received -- ;
 	return m_rx_msg.Data; 
}

uint16_t Can::available()
{
	HAL_StatusTypeDef _can_status;
 	_can_status = HAL_CAN_Receive_IT(m_can_interface_ptr, CAN_FIFO0);

 	// bool _new_msg = !check_eq_msgs(prev_msg,m_rx_msg.Data,8);
	//if (_new_msg)
	if ( g_nb_msg_received > 0 )
	{
		copy_msg(prev_msg,m_rx_msg.Data,8);
	}

	return g_nb_msg_received;
}

void Can::init()
{
	HAL_CAN_Receive_IT(m_can_interface_ptr, CAN_FIFO0);
}

uint32_t Can::getFrameAddress()
{
	return m_rx_msg.StdId;
}
/** Private Methods **/
/********************/

bool Can::check_eq_msgs(uint8_t* msg1, uint8_t* msg2,uint8_t size)
{
	bool is_equal = true ;
	for ( uint8_t i = 0; i< size; i++)
	{
		if (msg1[i] != msg2[i])
		{
			is_equal = false;
			break;
		}
		
	}

	return is_equal;
}

void Can::copy_msg(uint8_t* dest,uint8_t* src, uint8_t size)
{
	for( int i = 0; i < size; i++)
	{
		dest[i] = src[i];
	} 
}

