#include "Arduino.h"

#include <StarOddiCTD.h>

StarOddiCTD CTD(&Serial5);

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial5.begin(4800); //Serial port connected to CTD

  Serial.println("Starting!");

  //Set CTD calibration data (Found in .CAT file from SeaStar)
  CTD.calibration.T.C0 =  122.622785746828;
  CTD.calibration.T.C1 = -0.138854530877331;
  CTD.calibration.T.C2 = 0.000108169890868935;
  CTD.calibration.T.C3 = -5.58470579894668E-8;
  CTD.calibration.T.C4 = 1.53702000127998E-11;
  CTD.calibration.T.C5 = -1.81435671827578E-15;

  CTD.calibration.P.C0 =  -1.61597635237222;
  CTD.calibration.P.C1 =  0.00565052106231862;
  CTD.calibration.P.C2 =  -9.09681400791005E-8;
  CTD.calibration.P.C3 =  4.90908801913798E-11;
  CTD.calibration.P.C4 =  -8.71492645777175E-15;
  CTD.calibration.P.C5 =  -4.81753801054678E-19;

  CTD.calibration.Ptc.C1 = 7.02439662414173;
  CTD.calibration.Ptc.C2 = -0.21053250673308;
  CTD.calibration.Ptc.C3 = 0.00980786039230989;
  CTD.calibration.Ptc.C4 = -0.000240862172070564;
  CTD.calibration.Ptc.C5 = 2.17405487656103E-6;

  CTD.calibration.Tpr = 22.4427798102788;

  CTD.calibration.Cond.C0 = 98.0544546827358;
  CTD.calibration.Cond.C1 = -0.263561387205901;
  CTD.calibration.Cond.C2 = 0.000376092808984272;
  CTD.calibration.Cond.C3 = -3.09143413610036E-7;
  CTD.calibration.Cond.C4 = 1.50598310444937E-10;
  CTD.calibration.Cond.C5 = -4.27928881371478E-14;
  CTD.calibration.Cond.C6 = 6.5323443704887E-18;
  CTD.calibration.Cond.C7 = -4.12913234939624E-22;

  CTD.calibration.Ctc.C1 = -0.398142680468083;
  CTD.calibration.Ctc.C2 = -0.00259321862905614;
  CTD.calibration.Ctc.C3 = -0.000684962594896168;
  CTD.calibration.Ctc.C4 = 2.30924943510067E-5;
  CTD.calibration.Ctc.C5 = -1.76713340491716E-7;

  CTD.calibration.Ctc1.C1 = -0.276279974677843;
  CTD.calibration.Ctc1.C2 = -0.0925221052164181;
  CTD.calibration.Ctc1.C3 = 0.00180276949585506;
  CTD.calibration.Ctc1.C4 = -1.54831091575708E-5;
  CTD.calibration.Ctc1.C5 = 2.09671367968997E-7;

  CTD.calibration.Tcr = 23.88;

  CTD.calibration.Conductivity_L = 549;
  CTD.calibration.Conductivity_H = 3146;

  CTD.sample_delay = 1000; //~1Hz I think

  while(Serial5.available()) Serial5.read(); // Clear buffer
}

void loop() {

  if(CTD.update()) { //New data
    Serial.println("\nNew CTD data");
    Serial.print("Temperature  : "); Serial.print(CTD.temperature,5); Serial.println(" C");
    Serial.print("Pressure     : "); Serial.print(CTD.pressure,5); Serial.println(" BAR");
    Serial.print("Conductivity : "); Serial.print(CTD.conductivity,5); Serial.println(" mS/cm");
  }

  delay(100);
  return;


  if(Serial.available()) {
    char c = Serial.read();
    if(c == '1') {
      Serial.println("Enable CTD sampling");
      CTD.enable();
    }
    if(c == '2') {
      Serial.println("Disable CTD sampling");
      CTD.disable();
    }
  }
}
