#include <linux/string.h>
#include <linux/module.h>

#include "emuc_parse.h"

#if _DBG_FUNC
extern void print_func_trace (int line, const char *func);
#endif

static void chk_sum_end_byte (unsigned char *frame, int size);

/*----------------------------------------------------------------------------------*/
/**
 * @brief       Encode the Data Length Code.
 *
 * @param[in]   u8NumberOfBytes  Number of bytes in a message.
 *
 * @return      Data Length Code.
 *
 * @details     Converts number of bytes in a message into a Data Length Code.
 */
int EncodeDLC_CANFD(int u8NumberOfBytes)
{
    if (u8NumberOfBytes <= 8) return u8NumberOfBytes;
    else if (u8NumberOfBytes <= 12) return 9;
    else if (u8NumberOfBytes <= 16) return 10;
    else if (u8NumberOfBytes <= 20) return 11;
    else if (u8NumberOfBytes <= 24) return 12;
    else if (u8NumberOfBytes <= 32) return 13;
    else if (u8NumberOfBytes <= 48) return 14;
    else return 15;
}

/*----------------------------------------------------------------------------------*/
/**
 * @brief       Decode the Data Length Code.
 *
 * @param[in]   u8Dlc   Data Length Code.
 *
 * @return      Number of bytes in a message.
 *
 * @details     Converts a Data Length Code into a number of message bytes.
 */
int DecodeDLC_CANFD(int u8Dlc)
{
    if (u8Dlc <= 8) return u8Dlc;
    else if (u8Dlc == 9) return 12;
    else if (u8Dlc == 10) return 16;
    else if (u8Dlc == 11) return 20;
    else if (u8Dlc == 12) return 24;
    else if (u8Dlc == 13) return 32;
    else if (u8Dlc == 14) return 48;
    else return 64;
}


/*---------------------------------------------------------------------------------------*/
void EMUC_CANFD_SendHex (EMUC_CAN_FRAME *frame)
{
  unsigned char  *p;
  unsigned char   func1 = 0x00, func2 = 0x00;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  p = frame->com_buf;
  memset(p, 0, sizeof(frame->com_buf));

  /* head - byte 0 */
  *p = CMD_HEAD_SEND_CANFD;

  /* func1 - byte 1 */
  func1 += frame->CAN_port + 1;
  func1 += ((frame->id_type - 1) << 2);
  func1 += frame->rtr << 3;
  func1 += frame->dlc << 4;

  /* func2 - byte 2 */
  func2 += frame->brs << 0;
  func2 += frame->esi << 1;
  func2 += frame->fdf << 2;

  *(p + 1) = func1;
  *(p + 2) = func2;

  /* id - byte 3 ~ byte 6 */
  memcpy(p+3, frame->id, ID_LEN);

  /* data - byte 7 ~ */
  memcpy(p + CANFD_TX_HEAD_SIZE + 1, frame->data, DecodeDLC_CANFD(frame->dlc));

  /* chk sum & end byte - dlc is flexible */
  chk_sum_end_byte(p, CANFD_TX_HEAD_SIZE + 1 + DecodeDLC_CANFD(frame->dlc) + 3);

}

/*---------------------------------------------------------------------------------------*/
void EMUCSendHex (EMUC_CAN_FRAME *frame)
{
  unsigned char  *p;
  unsigned char   func = 0x00;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  p = frame->com_buf;
  memset(p, 0, sizeof(frame->com_buf));

  /* head - byte 0 */
  *p = CMD_HEAD_SEND;

  /* func - byte 1 */
  func += frame->CAN_port + 1;
  func += ((frame->id_type - 1) << 2);
  func += frame->rtr << 3;
  func += frame->dlc << 4;

  *(p+1) = func;

  /* id - byte 2 ~ byte 5 */
  memcpy(p+2, frame->id, ID_LEN);

  /* data - byte 6 ~ byte 13 */
  memcpy(p+6, frame->data, DATA_LEN);

  /* chk sum & end byte - byte 14 ~ byte 16 */
  chk_sum_end_byte(p, COM_BUF_LEN);
}

/*---------------------------------------------------------------------------------------*/
int EMUC_CANFD_RevHex (EMUC_CAN_FRAME *frame)
{
  int             i;
  unsigned char   chk_sum = 0x00;
  unsigned char   *p;
  unsigned char   func1 = 0, func2 = 0, dataLength;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  p = frame->com_buf;

/* parse 2 function bytes */
func1 = *(p + 1);
func2 = *(p + 2);

/* parse dlc */
dataLength = DecodeDLC_CANFD((func1 & 0b11110000) >> 4);

#if _DBG_RECV_HEX
/*--------------------------------------*/
  if(*p == CMD_HEAD_RECV)
  {
    for(i=0; i<COM_BUF_LEN; i++)
      printk("%02X ", *(p + i));
    printk("\n");
  }
  else
  {
    if(dataLength < 8)
    {
      printk("data len = %d\n", dataLength);
      for(i=0; i < CANFD_RX_HEAD_SIZE + 1 + 8 + 3; i++)
        printk("%02X ", *(p + i));
      printk("\n");
    }
    else
    {
      printk("data len = %d\n", dataLength);
      for(i=0; i < CANFD_RX_HEAD_SIZE + 1 + dataLength + 3; i++)
        printk("%02X ", *(p + i));
      printk("\n");
    }
  }

/*--------------------------------------*/
#endif

  /* verify checksum  */
  int len = 0;
  if(dataLength < 8) len = 8;
  else len = dataLength;
  
  for(i = 0; i < CANFD_RX_HEAD_SIZE + 1 + len; i++)
    chk_sum = chk_sum + *(p + i);

  if(chk_sum != *(p + CANFD_RX_HEAD_SIZE + 1 + len))
    return -2;
  
  if(*p == CMD_HEAD_RECV_CANFD)
  {
    frame->CAN_port = (int) ((func1 & 0b00000011) >> 0) - 1;
    frame->id_type  = (int) ((func1 & 0b00000100) >> 2) + 1;
    frame->rtr      = (int) (func1 & 0b00001000) >> 3;
    frame->dlc      = (int) (func1 & 0b11110000) >> 4;

    frame->brs      = (int) (func2 & 0b00000001) >> 0;
    frame->esi      = (int) (func2 & 0b00000010) >> 1;
    frame->fdf      = (int) (func2 & 0b00000100) >> 2;
    /* parse id */
    memcpy(frame->id, p + 11, ID_LEN);
    /* parse data */
    if(dataLength < 8)
    {
      memcpy(frame->data, p + CANFD_RX_HEAD_SIZE + 1, dataLength);
    }
    else
    {
      memcpy(frame->data, p + CANFD_RX_HEAD_SIZE + 1, dataLength);
    }
    /* parse timestamp */
    memcpy(&frame->timestamp, p + 3, TIMESTAMP_LEN);

    return 0;
  }
  else if(*p == CMD_HEAD_ERROR)
  {
    return 1;
  }
  else
    return -1;

} /* END: EMUC_CANFD_RevHex() */

/*---------------------------------------------------------------------------------------*/
int EMUCRevHex_TS (EMUC_CAN_FRAME *frame) // CAN2.0B with timestamp
{
  int             i;
  unsigned char   chk_sum = 0x00;
  unsigned char  *p;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  p = frame->com_buf;

#if _DBG_RECV_HEX
/*--------------------------------------*/
  for(i=0; i<COM_RX_BUF_LEN_TS; i++)
    printk("%02X ", *(p + i));
  printk("\n");
/*--------------------------------------*/
#endif

  /* check sum - byte 22 */
  for(i=0; i<COM_RX_BUF_LEN_TS-3; i++)
    chk_sum = chk_sum + *(p + i);

  if(chk_sum != *(p+22))
    return -2;

  /* head - byte 0 */
  if(*p == CMD_HEAD_RECV_TS)
  {
    /* func - byte 1 */
    frame->CAN_port = ((int) ( *(p+1) & 0x03)) - 1;
    frame->id_type  = ((int) ((*(p+1) & 0x04) >> 2)) + 1;
    frame->rtr      =  (int) ((*(p+1) & 0x08) >> 3);
    frame->dlc      =  (int) ((*(p+1) & 0xF0) >> 4);

    /* id - byte 2 ~ byte 5 */
    memcpy(frame->id, p+10, ID_LEN);

    /* data - byte 14 ~ byte 21 */
    memcpy(frame->data, p+14, DATA_LEN);

    return 0;
  }
  else if(*p == CMD_HEAD_ERROR)
  {
    return 1;
  }
  else
    return -1;

} /* END: EMUCRevHex_TS() */

/*---------------------------------------------------------------------------------------*/
int EMUCRevHex (EMUC_CAN_FRAME *frame)
{
  int             i;
  unsigned char   chk_sum = 0x00;
  unsigned char  *p;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  p = frame->com_buf;

#if _DBG_RECV_HEX
/*--------------------------------------*/
  for(i=0; i<COM_BUF_LEN; i++)
    printk("%02X ", *(p + i));
  printk("\n");
/*--------------------------------------*/
#endif

  /* check sum - byte 14 */
  for(i=0; i<COM_BUF_LEN-3; i++)
    chk_sum = chk_sum + *(p + i);

  if(chk_sum != *(p+14))
    return -2;

  /* head - byte 0 */
  if(*p == CMD_HEAD_RECV)
  {
    /* func - byte 1 */
    frame->CAN_port = ((int) ( *(p+1) & 0x03)) - 1;
    frame->id_type  = ((int) ((*(p+1) & 0x04) >> 2)) + 1;
    frame->rtr      =  (int) ((*(p+1) & 0x08) >> 3);
    frame->dlc      =  (int) ((*(p+1) & 0xF0) >> 4);

    /* id - byte 2 ~ byte 5 */
    memcpy(frame->id, p+2, ID_LEN);

    /* data - byte 6 ~ byte 13 */
    memcpy(frame->data, p+6, DATA_LEN);

    return 0;
  }
  else if(*p == CMD_HEAD_ERROR)
  {
    return 1;
  }
  else
    return -1;

} /* END: EMUCRevHex() */

/*---------------------------------------------------------------------------------------*/
void EMUCInitHex (int sts, unsigned char *cmd)
{
#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  *(cmd+1) = sts;
  *(cmd+2) = sts;
  *(cmd+3) = *(cmd+0) + *(cmd+1) + *(cmd+2);

  return;

} /* END: EMUCInitHex() */

/*---------------------------------------------------------------------------------------*/
static void chk_sum_end_byte (unsigned char *frame, int size)
{
  int            i;
  unsigned char  chk_sum = 0x00;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  for(i=0; i<size-3; i++)
    chk_sum = chk_sum + *(frame + i);

  *(frame + size - 3) = chk_sum;
  *(frame + size - 2) = 0x0D;
  *(frame + size - 1) = 0x0A;
}