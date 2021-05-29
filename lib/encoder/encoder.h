typedef struct
{
	int clk;
	int dt;
	int sw;
	volatile int count;
} ENC_t, *PENC_t;

void init_encoder(PENC_t encoder_adr, int clk, int dt, int sw);
