/* StarOddiCTD
 *
 * An Arduino library for the Star-Oddi DST-CTD
 *
 * Niklas Rolleberg 2020-07-10
 */

#ifndef _STARODDICTD_H
#define _STARODDICTD_H

#include <Stream.h>
#include <stdint.h>
#include <cmath>

class StarOddiCTD {

  enum class CTD_STATE{
    UNKNOWN,
    CONNECTED,
    PC_MODE,
    RECEIVE_DATA
  };
  CTD_STATE STATE = CTD_STATE::UNKNOWN;

  Stream* port;
  uint32_t timer = 0;

  char data_buffer[10];
  uint8_t data_index = 0;
  bool enable_sampling = false;
public:
  uint32_t sample_delay = 1000;
  uint32_t timeouts = 0;

  struct CalibrationConstants {
    double C0,C1,C2,C3,C4,C5,C6,C7;
  };

  struct CalibrationData {

    CalibrationConstants T;
    CalibrationConstants P;
    CalibrationConstants Ptc;
    double Tpr;
    CalibrationConstants Cond;
    CalibrationConstants Ctc;
    CalibrationConstants Ctc1;
    double Tcr;
    double Conductivity_L;
    double Conductivity_H;
  } calibration;


  double conductivity; // mS/cm
  double temperature;  // C
  double pressure;     // Bar
  double depth;        // M
  //double salinity;   //PSU?

  int16_t raw_T;
  int16_t raw_P;
  int16_t raw_C;


  StarOddiCTD(Stream* _port) {
    port = _port;
  }

  void enable() { enable_sampling = true;};
  void disable() { enable_sampling = false;};

  // false = not OK
  // true = OK
  bool status() {
    return STATE == CTD_STATE::PC_MODE || STATE == CTD_STATE::RECEIVE_DATA;
  }

  bool update() {

    if(!enable_sampling) {
      timer = millis();
      return false;
    }

    if(millis() - timer > max(10000, sample_delay*3)) {
      //Serial.println("Timeout");
      timeouts++;
      STATE = CTD_STATE::UNKNOWN;
    }

    switch(STATE) {
      case CTD_STATE::UNKNOWN: {

        while(port->available()) {
          if(port->read() == 0x55) {
            //Serial.println("CTD is alive and connected. Changing state from UNKNOWN to connected");
            STATE = CTD_STATE::CONNECTED;
            break;
          }
        }

        //Serial.println("State = UNKNOWN");
        if(millis() - timer > 1000) {
          //Wake the CTD
          char c = 0;
          port->write(c);
          timer = millis();
        }
      }
      break;

      case CTD_STATE::CONNECTED: {

        while(port->available()) {
          if(port->read() == 0x02) {
            //Serial.println("CTD is in PC mode");
            STATE = CTD_STATE::PC_MODE;
            break;
          }
        }

        //Serial.println("State = CONNECTED");
        if(millis() - timer > 1000) {
          //put the CTD in PC mode
          //Serial.println("Sending command to set CTD to PC mode");
          char c = 12;
          port->write(c);
          timer = millis();
        }
      }
      break;

      case CTD_STATE::PC_MODE: {

        while(port->available()) {
          if(port->read() == 0x01) {
            //Serial.println("CTD is ready to send data");
            //Send Ack back to CTD
            char c = 0x55;
            port->write(c);
            //Change state to receive data and wait for data
            STATE = CTD_STATE::RECEIVE_DATA;
            timer = millis();
            data_index = 0; //Reset buffer
            break;
          }
        }

        if(millis() - timer > sample_delay) {
          //Ask the CTD for data
          //Serial.println("Asking for data");
          char c = 1;
          port->write(c);
          timer = millis();
        }
      }
      break;

      case CTD_STATE::RECEIVE_DATA: {
        while(port->available()) {
          data_buffer[data_index] = port->read();
          data_index++;
          if(data_index >= 6) {
            //Go back to PC_MODE to ask for new data again
            STATE = CTD_STATE::PC_MODE;

            //Serial.println("Measurement received from CTD");
            raw_T = data_buffer[0] + (data_buffer[1] << 8);
            raw_P = data_buffer[2] + (data_buffer[3] << 8);
            raw_C = data_buffer[4] + (data_buffer[5] << 8);
            //Parse the data
            parse_data(raw_T,raw_P,raw_C);
            return true; // New data
          }
        }
        //If no data it received the timeout in the beginning of the function will reset the state to UNKNOWN
      }
      break;

      default:
        Serial.println("Default! not good.");
        STATE = CTD_STATE::UNKNOWN;
    }
    return false; //No new data
  };


  private:

  //Use the calibration to compute the true values
  void parse_data(double T, double P, double C) {

    //Serial.print("\tT: "); Serial.println(T);
    //Serial.print("\tP: "); Serial.println(P);
    //Serial.print("\tC: "); Serial.println(C);

    //////////////// Temperature ////////////////

    //Tv = T.C0 + (T.C1*T) + (T.C2*T^2) + (T.C3*T^3) + (T.C4*T^4) + (T.C5*T^5)

    temperature = calibration.T.C0 +
                (calibration.T.C1* T) +
                (calibration.T.C2*std::pow(T,2.0)) +
                (calibration.T.C3*std::pow(T,3.0)) +
                (calibration.T.C4*std::pow(T,4.0)) +
                (calibration.T.C5*std::pow(T,5.0));

    //////////////// Pressure ////////////////

    //Pc = P + (Ptc.C1*Ptc) + (Ptc.C2*Ptc^2) + (Ptc.C3*Ptc^3) + (Ptc.C4*Ptc^4) +
    //(Ptc.C5*Ptc^5) - ((Ptc.C1*Tv) + (Ptc.C2*Tv^2) + (Ptc.C3*Tv^3) +
    //(Ptc.C4*Tv^4) + (Ptc.C5*Tv^5))

    double Pc = P
                + (calibration.Ptc.C1*calibration.Tpr)
                + (calibration.Ptc.C2*std::pow(calibration.Tpr,2.0))
                + (calibration.Ptc.C3*std::pow(calibration.Tpr,3.0))
                + (calibration.Ptc.C4*std::pow(calibration.Tpr,4.0))
                + (calibration.Ptc.C5*std::pow(calibration.Tpr,5.0))
                - ((calibration.Ptc.C1*temperature)
                + (calibration.Ptc.C2*std::pow(temperature,2.0))
                + (calibration.Ptc.C3*std::pow(temperature,3.0))
                + (calibration.Ptc.C4*std::pow(temperature,4.0))
                + (calibration.Ptc.C5*std::pow(temperature,5.0)));

    pressure = calibration.P.C0
              + (calibration.P.C1*Pc)
              + (calibration.P.C2*std::pow((double) Pc,2.0))
              + (calibration.P.C3*std::pow((double) Pc,3.0))
              + (calibration.P.C4*std::pow((double) Pc,4.0))
              + (calibration.P.C5*std::pow((double) Pc,5.0));

    //////////////// Depth //////////////// //TODO look at this

    //double g = 9.80665;   //standard acceleration of gravity = 9.80665 (m/s^2)
    double Sd = 1.026;    //Seawater density = 1.026
    double gc = 10.19716; //gravity conversion constant = 100/g = 10.19716

    //If measurements are performed in seawater then
    //(8): D = Pv*gc/Sd
    //Else:
    //(9): D = Pv*gc
    depth = pressure*gc/Sd;

    //////////////// Conductivity ////////////////

    //Correction for low load:
    //Cc0 = C+(Ctc.C1*Tcr) + (Ctc.C2*Tcr^2) + (Ctc.C3*Tcr^3) + (Ctc.C4*Tcr^4) +
    //(Ctc.C5*Tcr^5) - ((Ctc.C1*Tv) + (Ctc.C2*Tv^2) + (Ctc.C3*Tv^3) +
    //(Ctc.C4*Tv^4) + (Ctc.C5*Tv^5))

    double Cc0 = C
            +(calibration.Ctc.C1*calibration.Tcr)
            + (calibration.Ctc.C2*std::pow(calibration.Tcr,2.0))
            + (calibration.Ctc.C3*std::pow(calibration.Tcr,3.0))
            + (calibration.Ctc.C4*std::pow(calibration.Tcr,4.0))
            + (calibration.Ctc.C5*std::pow(calibration.Tcr,5.0))
            - ((calibration.Ctc.C1*temperature)
            + (calibration.Ctc.C2*std::pow(temperature,2.0))
            + (calibration.Ctc.C3*std::pow(temperature,3.0))
            + (calibration.Ctc.C4*std::pow(temperature,4.0))
            + (calibration.Ctc.C5*std::pow(temperature,5.0)));

    // Correction for high load:
    //Cc1 = C+(Ctc1.C1*Tcr) + (Ctc1.C2*Tcr^2) + (Ctc1.C3*Tcr^3) + (Ctc1.C4*Tcr^4) +
    //(Ctc1.C5*Tcr^5) - ((Ctc1.C1*Tv) + (Ctc1.C2*Tv^2) + (Ctc1.C3*Tv^3) +
    //(Ctc1.C4*Tv^4) + (Ctc1.C5*Tv^5))

    double Cc1 = C
        +(calibration.Ctc1.C1*calibration.Tcr)
        + (calibration.Ctc1.C2*std::pow(calibration.Tcr,2.0))
        + (calibration.Ctc1.C3*std::pow(calibration.Tcr,3.0))
        + (calibration.Ctc1.C4*std::pow(calibration.Tcr,4.0))
        + (calibration.Ctc1.C5*std::pow(calibration.Tcr,5.0))
        - ((calibration.Ctc1.C1*temperature)
        + (calibration.Ctc1.C2*std::pow(temperature,2.0))
        + (calibration.Ctc1.C3*std::pow(temperature,3.0))
        + (calibration.Ctc1.C4*std::pow(temperature,4.0))
        + (calibration.Ctc1.C5*std::pow(temperature,5.0)));

    //Combining high and low correction ina linear equation:
    //A = (Cc1-Cc0)/(H-L)
    //B = Cc0-A*L
    //Cc= B+A*C
    double A = (Cc1-Cc0)/(calibration.Conductivity_H-calibration.Conductivity_L);
    double B = Cc0-A*calibration.Conductivity_L;
    double Cc = B+A*C;

    //The conductivity unit value (mS/cm):
    //Cv = Cond.C0 + (Cond.C1*Cc) + (Cond.C2*Cc^2) + (Cond.C3*Cc^3) +
    //     (Cond.C4*Cc^4) + (Cond.C5*Cc^5) + (Cond.C6*Cc^6) + (Cond.C7*Cc^7)

    conductivity = calibration.Cond.C0
                + (calibration.Cond.C1*Cc)
                + (calibration.Cond.C2*std::pow(Cc,2.0))
                + (calibration.Cond.C3*std::pow(Cc,3.0))
                + (calibration.Cond.C4*std::pow(Cc,4.0))
                + (calibration.Cond.C5*std::pow(Cc,5.0))
                + (calibration.Cond.C6*std::pow(Cc,6.0))
                + (calibration.Cond.C7*std::pow(Cc,7.0));

    //////////////// Salinity ////////////////
    // Not implemented yet

  }

};

#endif //_STARODDICTD_H
