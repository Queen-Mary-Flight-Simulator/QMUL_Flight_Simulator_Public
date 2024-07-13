#ifndef HUD_H
#define HUD_H

#ifdef __cplusplus
extern "C" {
#endif

#define HUD_HUDX     512
#define HUD_HUDY     384

#define HUD_CompassX HUD_HUDX
#define HUD_CompassY (HUD_HUDY + 130)

#define HUD_AltX     (HUD_HUDX + 175)
#define HUD_AltY     (HUD_HUDY - 70)

#define HUD_AsiX     (HUD_HUDX - 175)
#define HUD_AsiY     (HUD_HUDY - 70)

extern void HUD_SetOrigin(int x, int y);

extern void HUD_DrawHUD(float pitch, float roll, float hdg, float ias, float altitude,
                        float fpa_vertical, float fpa_lateral, float gammadot, float betadot,
                        float UDot, unsigned int baro);

extern void BEGIN_HUD();

#ifdef __cplusplus
}
#endif

#endif
