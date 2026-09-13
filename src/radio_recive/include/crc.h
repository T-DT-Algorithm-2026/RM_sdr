#ifndef _CRC_H
#define _CRC_H

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#define NULL 0 /* see <stddef.h> */

	extern const unsigned char CRC8_TAB[256];
	extern const uint16_t wCRC_Table[256];

	/**
	 * @addtogroup TDT_CRC
	 * @{
	 */

	/// 获取CRC8校验位
	unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
	/// 对数据进行CRC8校验
	unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
	/// 对数据追加CRC8数据位
	void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
	/// 获取CRC16校验位
	uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
	/// 对数据进行CRC16校验
	uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
	/// 对数据追加CRC16数据位
	void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

	/** @} */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif