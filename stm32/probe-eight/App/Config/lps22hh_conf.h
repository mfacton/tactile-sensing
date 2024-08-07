#ifndef CONFIG_LPS22HH_CONF_H_
#define CONFIG_LPS22HH_CONF_H_

//max spi speed 10MHz
#define LPS22HH_HANDLE hspi1
//#define LPS22HH_INT_PIN DRDYP_Pin

#define LPS22HH_LIST T(CS1) T(CS2) T(CS3) T(CS4) T(CS5) T(CS6) T(CS7) T(CS8) T(CS9) T(CS10)

#define LPS22HH_COUNT 10

enum Lps22hhId {
	Lps22hh1,
	Lps22hh2,
	Lps22hh3,
	Lps22hh4,
	Lps22hh5,
	Lps22hh6,
	Lps22hh7,
	Lps22hh8,
	Lps22hh9,
	Lps22hh10,
};

#endif
