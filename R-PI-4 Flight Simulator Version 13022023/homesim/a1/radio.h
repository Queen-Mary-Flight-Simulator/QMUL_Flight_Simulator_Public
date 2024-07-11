#ifndef Radio_H
#define Radio_H

#include <GL/gl.h>
#include <SIM/iosdefn.h>
#include <SIM/navdefn.h>

extern NavDefn_RadioPanel Radio_Radios[2];

extern void Radio_RestoreRMP(IosDefn_RestoreVectorRecord v);

extern void Radio_SaveRMP(NavDefn_NavDataPkt *pkt);

extern void Radio_CheckRadio(int x, int y, int leftb, int middleb, int rightb);

extern void Radio_UpdateRadio();

extern void Radio_Radio(int x0, int y0, GLuint texobj_stack, GLuint texobj_simpleknob, GLuint texobj_innerknob, GLuint texobj_outerknob);

extern void Radio_SetRadio(unsigned int n, unsigned int chn, float f);

extern void BEGIN_Radio();

#endif

