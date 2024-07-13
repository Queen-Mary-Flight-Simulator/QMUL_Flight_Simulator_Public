#ifndef FSLINK_H
#define FSLINK_H


/* UDP communication functions */
int  UDP_Start(unsigned short); /* port */
void UDP_Stop(void);
int  UDP_Send(void);
int  UDP_Recv(void);

/* Receiver and Transmit data update function */
void UDP_Data_Set(double *);
void UDP_Data_Get(double *);


#endif

