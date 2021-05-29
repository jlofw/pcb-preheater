#include "encoder.h"

void init_encoder(PENC_t encoder_adr, int clk, int dt, int sw)
{
	encoder_adr->clk = clk;
	encoder_adr->dt = dt;
	encoder_adr->sw = sw;
}
