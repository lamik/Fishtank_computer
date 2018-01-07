/*
 * sd_card.h
 *
 *  Created on: 4 sty 2018
 *      Author: Mateusz Salamon
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
#define CMD0 0x00
/* SEND_OP_COND (MMC) */
#define CMD1 0x01
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
#define CMD8 0x08
/** SEND_CSD - read the Card Specific Data (CSD register) */
#define CMD9 0x09
/** SEND_CID - read the card identification information (CID register) */
#define CMD10 0x0A
/** SEND_STATUS - read the card status register */
#define CMD13 0x0D
/* SET_BLOCKLEN */
#define CMD16 0x10
/** READ_BLOCK - read a single data block from the card */
#define CMD17 0x11
/** WRITE_BLOCK - write a single data block to the card */
#define CMD24 0x18
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
#define CMD25 0x19
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
#define CMD32 0x20
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
 range to be erased*/
#define CMD33 0x21
/** ERASE - erase all previously selected blocks */
#define CMD38 0x26
/** APP_CMD - escape for application specific command */
#define CMD55 0x37
/** READ_OCR - read the OCR register of a card */
#define CMD58 0x3A
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
 pre-erased before writing */
#define ACMD23 (0xC0 + 23)
/** SD_SEND_OP_COMD - Sends host capacity support information and
 activates the card's initialization process */
#define ACMD41 (0xC0 + 41)
//------------------------------------------------------------------------------
/** status for card in the ready state */
#define R1_READY_STATE  0x00
/** status for card in the idle state */
#define R1_IDLE_STATE  0x01
/** status bit for illegal command */
#define R1_ILLEGAL_COMMAND  0x04
/** start data token for read or write single block*/
#define DATA_START_BLOCK  0xFE
/** stop token for write multiple blocks*/
#define STOP_TRAN_TOKEN  0xFD
/** start data token for write multiple blocks*/
#define WRITE_MULTIPLE_TOKEN  0xFC
/** mask for data response tokens after a write block operation */
#define DATA_RES_MASK  0x1F
/** write data accepted token */
#define DATA_RES_ACCEPTED  0x05
//------------------------------------------------------------------------------
/* Card type flags (CardType) */
#define CT_MMC				0x01	/* MMC ver 3 */
#define CT_SD1				0x02	/* SD ver 1 */
#define CT_SD2				0x04	/* SD ver 2 */
#define CT_SDC				(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK			0x08	/* Block addressing */

BYTE CardType;

void SD_SelectCard();
void SD_DeselectCard();
void SD_WaitUntilReady();

uint8_t SD_Init();

uint8_t SD_Command(uint8_t command, uint32_t arg);

#endif /* SD_CARD_H_ */
