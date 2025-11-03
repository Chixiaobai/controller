#ifndef __EMUC_PARSE_H__
#define __EMUC_PARSE_H__



#define    ID_LEN                   4
#define    DATA_LEN                 8
#define    COM_BUF_LEN              17
#define    COM_RX_BUF_LEN_TS        25
#define    COM_CANFD_TX_BUF_LEN     74
#define    COM_CANFD_RX_BUF_LEN     82
#define    DATA_LEN_ERR             6
#define    TIME_CHAR_NUM            13
#define    CMD_HEAD_INIT            0x61
#define    CMD_HEAD_SEND            0xE0
#define    CMD_HEAD_RECV            0xE1
#define    CMD_HEAD_RECV_TS         0xE2
#define    CMD_HEAD_ERROR           0x99
/* CAN FD Command */
#define    CANFD_DATA_LEN           64
#define    CMD_HEAD_SEND_CANFD      0xF0
#define    CMD_HEAD_RECV_CANFD      0xF1
#define    CANFD_RX_HEAD_SIZE       14
#define    CANFD_TX_HEAD_SIZE       6
#define    TIMESTAMP_LEN            8


/*--------------------------------------*/
enum
{
  EMUC_CAN_1 = 0,
  EMUC_CAN_2
};

enum
{
  EMUC_SID = 1,
  EMUC_EID
};

enum
{
  EMUC_INACTIVE = 0,
  EMUC_ACTIVE
};


/*-----------------------------*/
typedef struct
{
    unsigned int    second;
    unsigned int    micro_second;
} TIME_ST_T;


/*--------------------------------------*/
typedef struct
{
  int            CAN_port;
  int            id_type;
  int            rtr;
  int            dlc;
  /* CANFD */
  int            brs;
  int            esi;
  int            fdf;

  unsigned char  id      [ID_LEN];
  unsigned char  data    [CANFD_DATA_LEN];
  unsigned char  com_buf [COM_CANFD_RX_BUF_LEN];
  TIME_ST_T      timestamp;

} EMUC_CAN_FRAME;

/*--------------------------------------*/
void EMUC_CANFD_SendHex   (EMUC_CAN_FRAME *frame);
int  EMUC_CANFD_RevHex    (EMUC_CAN_FRAME *frame);
void EMUCSendHex          (EMUC_CAN_FRAME *frame);
int  EMUCRevHex           (EMUC_CAN_FRAME *frame);
int  EMUCRevHex_TS        (EMUC_CAN_FRAME *frame);
void EMUCInitHex          (int sts, unsigned char *cmd);
/* CANFD */
int EncodeDLC_CANFD       (int u8NumberOfBytes);
int DecodeDLC_CANFD       (int u8Dlc);

#endif