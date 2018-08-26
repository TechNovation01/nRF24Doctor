#pragma once

#define COUNT_OF(x) 			((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define CONSTRAIN_HI(amt,high)	((amt)>(high)?(high):(amt))
#define CONSTRAIN_LO(amt,low)	((amt)<(low)?(low):(amt))

#ifdef LOCAL_DEBUG
#define Sprint(a)   (Serial.print(a))			// macro as substitute for print, enable if no print wanted
#define Sprintln(a) (Serial.println(a))
#define Sflush() 	(Serial.flush())
#else											// macro for "no" debug print
#define Sprint(a)
#define Sprintln(a)
#define Sflush()
#endif

const char *pcPaLevelNames[]  = { "MIN", "LOW", "HIGH", "MAX" };
const char *pcDataRateNames[] = { "1MBPS", "2MBPS" , "250KBPS"};

const char* rf24PaLevelToString( const uint8_t level )
{
	const uint8_t idx = CONSTRAIN_HI( level, COUNT_OF(pcPaLevelNames)-1 );
	return pcPaLevelNames[idx];
}

uint8_t rf24PaLevelConstrain( const uint8_t level )
{
	return CONSTRAIN_HI( level, COUNT_OF(pcPaLevelNames)-1 );
}

const char* rf24DataRateToString( const uint8_t rate )
{
	const uint8_t idx = CONSTRAIN_HI( rate, COUNT_OF(pcDataRateNames)-1 );
	return pcDataRateNames[idx];
}

uint8_t rf24DataRateConstrain( const uint8_t rate )
{
	return CONSTRAIN_HI( rate, COUNT_OF(pcDataRateNames)-1 );
}
