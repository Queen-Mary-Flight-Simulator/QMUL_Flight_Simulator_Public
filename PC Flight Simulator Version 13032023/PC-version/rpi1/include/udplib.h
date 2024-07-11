#ifndef UDPLib_H
#define UDPLib_H

extern void UDPLib_Connect(unsigned int node, void *pkt, unsigned int size);
extern void UDPLib_Open(unsigned int n);
extern void UDPLib_Close();
extern void UDPLib_SendPkt(void *pkt, unsigned int n);
extern unsigned int UDPLib_GetPkt();
extern void BEGIN_UDPLib();

#endif
