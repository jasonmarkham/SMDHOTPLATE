#ifdef NTC
	// QU-BD silicone bed QWG-104F-3950 thermistor
	// (nppc) Table adjusted to reflect Chinese thermistor
	#define OVERSAMPLENR 	4	// 12 bit resolution
	#define tt_len 			50	// items in tt table

	const short TEMPTABLE[][2] PROGMEM = {
	  {    1 * OVERSAMPLENR, 700 },   // 938
	  {   31 * OVERSAMPLENR, 261 },   // 314
	  {   41 * OVERSAMPLENR, 256 },   // 290
	  {   51 * OVERSAMPLENR, 241 },   // 272
	  {   61 * OVERSAMPLENR, 230 },   // 258
	  {   71 * OVERSAMPLENR, 221 },   // 247
	  {   81 * OVERSAMPLENR, 212 },   // 237
	  {   91 * OVERSAMPLENR, 206 },   // 229
	  {  101 * OVERSAMPLENR, 199 },   // 221
	  {  111 * OVERSAMPLENR, 194 },   // 215
	  {  121 * OVERSAMPLENR, 189 },   // 209
	  {  131 * OVERSAMPLENR, 185 },   // 204
	  {  141 * OVERSAMPLENR, 181 },   // 199
	  {  151 * OVERSAMPLENR, 178 },   // 195
	  {  161 * OVERSAMPLENR, 174 },   // 190
	  {  171 * OVERSAMPLENR, 170 },   // 187
	  {  181 * OVERSAMPLENR, 167 },   // 183
	  {  191 * OVERSAMPLENR, 164 },   // 179
	  {  201 * OVERSAMPLENR, 160 },   // 176
	  {  221 * OVERSAMPLENR, 156 },   // 170
	  {  241 * OVERSAMPLENR, 151 },   // 165
	  {  261 * OVERSAMPLENR, 148 },   // 160
	  {  281 * OVERSAMPLENR, 144 },   // 155
	  {  301 * OVERSAMPLENR, 141 },   // 150
	  {  331 * OVERSAMPLENR, 138 },   // 144
	  {  361 * OVERSAMPLENR, 134 },   // 139
	  {  391 * OVERSAMPLENR, 130 },   // 133
	  {  421 * OVERSAMPLENR, 126 },   // 128
	  {  451 * OVERSAMPLENR, 122 },   // 123
	  {  491 * OVERSAMPLENR, 117 },   // 117
	  {  531 * OVERSAMPLENR, 113 },   // 111
	  {  571 * OVERSAMPLENR, 107 },   // 105
	  {  611 * OVERSAMPLENR, 102 },   // 100
	  {  641 * OVERSAMPLENR,  97 },   //  95
	  {  681 * OVERSAMPLENR,  92 },   //  90
	  {  711 * OVERSAMPLENR,  87 },   //  85
	  {  751 * OVERSAMPLENR,  81 },   //  79
	  {  791 * OVERSAMPLENR,  75 },   //  72
	  {  811 * OVERSAMPLENR,  70 },   //  69
	  {  831 * OVERSAMPLENR,  66 },   //  65
	  {  871 * OVERSAMPLENR,  60 },   //  57
	  {  881 * OVERSAMPLENR,  57 },   //  55
	  {  901 * OVERSAMPLENR,  52 },   //  51
	  {  921 * OVERSAMPLENR,  47 },   //  45
	  {  941 * OVERSAMPLENR,  39 },   //  39
	  {  971 * OVERSAMPLENR,  32 },   //  28
	  {  981 * OVERSAMPLENR,  25 },   //  23
	  {  991 * OVERSAMPLENR,  18 },   //  17
	  { 1001 * OVERSAMPLENR,  12 },   //   9
	  { 1021 * OVERSAMPLENR, -24 }    // -27
	};

	static void* heater_ttbl_map = (void*)TEMPTABLE;

	#define PGM_RD_W(x)   (short)pgm_read_word(&x)

	float analog2temp(uint16_t raw) {
		if (raw>(1020 * OVERSAMPLENR)) {	// consider sensor is disconnected
			return 999.0;	// return high but 3 digit number
		}
		float celsius = 0;
		uint8_t i;
		short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map);

		for (i = 1; i < tt_len; i++) {
		  if (PGM_RD_W((*tt)[i][0]) > raw) {
			celsius = PGM_RD_W((*tt)[i - 1][1]) +
					  (raw - PGM_RD_W((*tt)[i - 1][0])) *
					  (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
					  (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
			break;
		  }
		}

		// Overflow: Set to last value in the table
		if (i == tt_len) celsius = PGM_RD_W((*tt)[i - 1][1]);

		return celsius;
	}

	float getTemperature_0(void){
		uint16_t smoothADC = digitalSmooth(collectADCraw(TEMPSENSOR_PIN_0), BSmoothArray);
		return analog2temp(smoothADC);
	}

 float getTemperature_1(void){
   uint16_t smoothADC = digitalSmooth(collectADCraw(TEMPSENSOR_PIN_1), BSmoothArray);
    return analog2temp(smoothADC);
  }
	
	int collectADCraw(uint8_t sensor) {
		int sumADC;
		sumADC = analogRead(sensor);	// dummy read
		sumADC = 0;
		for (uint8_t i = 0; i < OVERSAMPLENR; i++) {
			sumADC+=analogRead(sensor);
		}
		return sumADC;
	}

	void Temperature_init(void) {
		for(uint8_t i=0;i<filterSamples*2;i++){getTemperature_0();}
	}
#endif
