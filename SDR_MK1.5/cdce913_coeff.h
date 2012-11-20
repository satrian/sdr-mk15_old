
#define FREQ_IN 27000000
#define FREQ_125MHZ 125000000
#define FREQ_150MHZ 150000000
#define FREQ_175MHZ 175000000
#define FREQ_MAX 320000000
#define FREQ_MIN 27000000

/*
Wrong order from original header file
union pll_conf {
	struct {
		unsigned vco_range: 2;
		unsigned p: 3;
		unsigned q: 6;
		unsigned r: 9;
		unsigned n: 12;
	};
	unsigned int data;
	unsigned char darr[4];
};
*/

union pll_conf {
	struct {
		unsigned n: 12;
		unsigned r: 9;
		unsigned q: 6;
		unsigned p: 3;
		unsigned vco_range: 2;
	};
	unsigned int data;
	unsigned char darr[4];
};

union pll_conf find_coeffs(unsigned int fvco, unsigned int* fvcoreal);
int coeffs_are_valid(union pll_conf pc);
unsigned int uabssub(unsigned int a, unsigned int b);