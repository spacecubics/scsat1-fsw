/*
 * Space Cubics OBC TRCH Software
 *  Definitions for USART
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#define MSG_LEN 30

struct usart_tx_msg {
        char *msg;
        unsigned active :1;
};
extern struct usart_tx_msg tx_msg;

struct usart_rx_msg {
        char msg[MSG_LEN];
        int addr;
        unsigned active :1;
        unsigned err :1;
};
extern struct usart_rx_msg rx_msg;

extern void usart_init (void);
extern void send_char (char msg);
extern void send_msg (char *msg);
extern void start_usart_receive (void);
extern void receive_msg_isr (void);
extern void receive_msg_clear (void);
