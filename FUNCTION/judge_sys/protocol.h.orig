

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f4xx.h"

#define NULL 0

#define UP_REG_ID    0xA0  //up layer regional id		 
#define DN_REG_ID    0xA5  //down layer regional id  
#define HEADER_LEN   sizeof(frame_header_t)					 
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#define PROTOCAL_FRAME_MAX_SIZE  200

/** 
  * @brief  frame header structure definition//帧头结构体定义
  */
typedef __packed struct
{
  uint8_t  sof;//数据帧起始字节，固定值为0xA5
  uint16_t data_length; //数据帧中data的长度
  uint8_t  seq;  //包序号
  uint8_t  crc8;  //帧头CRC8校验	
	
} frame_header_t;

uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);


#endif
