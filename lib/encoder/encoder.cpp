#include "encoder.h"

void init_encoder(p_encoder_struct encoder_adr, int clk, int dt, int sw)
{
	encoder_adr->clk = clk;
	encoder_adr->dt = dt;
	encoder_adr->sw = sw;
}
