/*
 * xmodem1k.h
 *
 *  Created on: 18 Ara 2015
 *      Author: admin
 */

#ifndef XMODEM1K_H_
#define XMODEM1K_H_
/*****************************************************************************
 * $Id$
 *
 * Project: 	NXP LPC1100 Secondary Bootloader Example
 *
 * Description: Implements an Xmodem1K client (receiver).
 *
 * Copyright(C) 2010, NXP Semiconductor
 * All rights reserved.
 *
 *****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 *****************************************************************************/
#ifndef __XMODEM1K_H
#define __XMODEM1K_H

#define DOWNLOAD_MAX_TRIALS		(3)
#define DOWNLOAD_ERR_TIMEOUT	(-1)


#define MAX_PACKET_TIMEOUTS		(7)
/*****************************************************************************
** Function name:	XModem1K_Client
**
** Description:		The function starts a download procedure using modified
** 		xmodem1k protocol to download the image.
**
** 		Client 								Server
** 		----->>> POLL
** 		             <<<-- |SOH | 2 bytes frame number(1) | 1k of image | CRC |
** 		----->>> ACK
** 		             <<<-- |SOH | 2 bytes frame number(2) | 1k of image | CRC |
** 		----->>> ACK
** 		             <<<-- |SOH | 2 bytes frame number(3) | 1k of image | CRC |
** 		----->>> NAK
** 		             <<<-- |SOH | 2 bytes frame number(3) | 1k of image | CRC |
** 		----->>> NAK
** 		             <<<-- |SOH | 2 bytes frame number(3) | 1k of image | CRC |
**      Note: when client sends NAK server repeats the last frame in example
**      above frame number(3)repeats
** 		----->>> ACK
** 		             <<<-- |SOH | 2 bytes frame number(4) | 1k of image | CRC |
**      ..........................................
**      ..........................................
**
** 		----->>> ACK
** 		             <<<-- |SOH | 2 bytes frame number(n) | 1k of image | CRC |
**      All image information downloaded
** 		----->>> ACK
** 		             <<<-- |EOT |4 bytes image size | 2 bytes CRC (This CRC
** 		             is calculated for all frames sent |
**
**      Procedure finishes here and bootrom checks image sanity using the CRC
**      and boots the image if CRC does not match bootrom boots the PRIMARY image
**
**      The last CRC in the EOT frame is calculated for all frames. All other CRC
**      is calculated for only each frame.
**
** Parameters:		pointer to a function of type pu32Xmodem1kRxPacketCallback
**                  this functions gets called after each frame received
**                  without error.
** Returned value:	TRUE   downloaded image successfully
** 					DOWNLOAD_ERR_TIMEOUT  download ended with an error
**
******************************************************************************/
int 	XModem1K_Client(char *imei);

#endif /* end __XMODEM1K_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/



#endif /* XMODEM1K_H_ */
