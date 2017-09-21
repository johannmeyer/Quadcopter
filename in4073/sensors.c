#include "sensors.h"
#include "in4073.h"

typedef struct calibData calibData;

#define BUF_SIZE  225

struct calibData
{
  int16_t mat[BUF_SIZE];
  uint8_t next_pos;
  uint8_t max_pos;
  uint8_t min_pos;
  uint32_t count;
  bool ready;
};

void init_calibData(calibData *data);

void maxElem(calibData *data);

void minElem(calibData *data);

void insert_data(calibData *data, int16_t newValue, uint8_t tolerance);

bool calibration_flag = false;

void calibrate_sensors(void)
{
  /*Use a set of measurements from each sensor to detect when it stabilizes and evaluate its DC offset*/

  uint8_t i;
  calibData calibArray[9];
  uint16_t newSensorValues[9];

  for(i=0; i<9; i++)
  {
    //initialize the struct containing the initialization data for each sensor.
    init_calibData(&calibArray[i]);
  }

  //The process of the identification of the object continues until the dc offset can be determined for all the sensors
  while(!calibArray[0].ready | !calibArray[1].ready |  !calibArray[3].ready | !calibArray[4].ready
  | !calibArray[5].ready | !calibArray[6].ready | !calibArray[7].ready | !calibArray[8].ready)
  {
    //Polling until a sample is successfully acquired.
    while(!get_dmp_data());

    newSensorValues[0] = phi;
    newSensorValues[1] = theta;
    newSensorValues[2] = psi;
    newSensorValues[3] = sp;
    newSensorValues[4] = sr;
    newSensorValues[5] = sq;
    newSensorValues[6] = sax;
    newSensorValues[7] = say;
    newSensorValues[8] = saz;

    for(i=0; i<6; i++)
    {
      //The new sensor value and the biggest difference tolerated between the max and the min value of the
      //sensor buffer, for the sensor to be consider stabilized is passed.
      insert_data(&calibArray[i], newSensorValues[i], 3);
    }
    for(i=6; i<9; i++)
    {
      insert_data(&calibArray[i], newSensorValues[i], 80);
    }
  }

  //When all the sensors are considered stabilized, the average of the maximum and the minimum value of the
  //buffer is used as the DC offset to be subtracted.

  dcphi = (calibArray[0].mat[calibArray[0].max_pos] + calibArray[0].mat[calibArray[0].min_pos]) / 2;
  dctheta = (calibArray[1].mat[calibArray[1].max_pos] + calibArray[1].mat[calibArray[1].min_pos]) / 2;
  dcpsi = (calibArray[2].mat[calibArray[2].max_pos] + calibArray[2].mat[calibArray[2].min_pos]) / 2;
  dcsp = (calibArray[3].mat[calibArray[3].max_pos] + calibArray[3].mat[calibArray[3].min_pos]) / 2;
  dcsq = (calibArray[4].mat[calibArray[4].max_pos] + calibArray[4].mat[calibArray[4].min_pos]) / 2;
  dcsr = (calibArray[5].mat[calibArray[5].max_pos] + calibArray[5].mat[calibArray[5].min_pos]) / 2;
  dcsax = (calibArray[6].mat[calibArray[6].max_pos] + calibArray[6].mat[calibArray[6].min_pos]) / 2;
  dcsay = (calibArray[7].mat[calibArray[7].max_pos] + calibArray[7].mat[calibArray[7].min_pos]) / 2;
  dcsaz = (calibArray[8].mat[calibArray[8].max_pos] + calibArray[8].mat[calibArray[8].min_pos]) / 2;

  calibration_flag = true;
}


bool isCalibrated(void)
{
  /*Allows the calibration_flag to be checked without allowing code outside this file to write it*/
  return calibration_flag;
}

int16_t get_sensor(uint8_t sensor)
{
  /*Returns a calibrated sample of the selected sensor*/
  int16_t senseValue = 0;

  switch (sensor)
  {
    case SP:
      senseValue = sp - dcsp;
      break;

    case SQ:
      senseValue = sq - dcsq;
      break;

    case SR:
      senseValue = sr - dcsr;
      break;

    case SAX:
      senseValue = sax - dcsax;
      break;

    case SAY:
      senseValue = say - dcsay;
      break;

    case SAZ:
      senseValue = saz - dcsaz;
      break;

    case PHI:
      senseValue = phi - dcphi;
      break;

    case THETA:
      senseValue = theta - dctheta;
      break;

    case PSI:
      senseValue = psi - dcpsi;
      break;
  }

  return senseValue;
}

void init_calibData(calibData *data)
{
  /*The structure to be used for the calibration of a sensor is initialized.*/
  data->next_pos = 0;
  data->min_pos = 0;
  data->max_pos = 0;
  data->count = 0;
  data->ready = false;
}

void maxElem(calibData *data)
{
  /*The position of the maximum value of the elements of the sensor's buffer is found and stored in its corresponding member.*/
  uint8_t i;
  int16_t max = 0;

  for(i=0; i<BUF_SIZE && i<data->count; i++)
  {
    if(data->mat[i] > max)
    {
      max = data->mat[i];
      data->max_pos = i;
    }
  }
}

void minElem(calibData *data)
{
  /*The position of the minimum value of the elements of the sensor's buffer is found and stored in its corresponding member.*/
  uint8_t i;
  int16_t min = 0;

  for(i=0; i<BUF_SIZE && i<data->count; i++)
  {
    if(data->mat[i] < min)
    {
      min = data->mat[i];
      data->min_pos = i;
    }
  }
}

void insert_data(calibData *data, int16_t newValue, uint8_t tolerance)
{
  /*A new sensor sample is stored in the buffer, updating the structure member affected by it*/

  //The new value is stored in the appropriate position.
  data->mat[data->next_pos] = newValue;
  //The ready flag is cleared as the new value may lead to a difference that cannot be tolerated
  data->ready = false;

  if(data->count<=BUF_SIZE)
  {//if the buffer is not full, it only needs to be checked whether the max and min positions need to be updated.
    if(data->mat[data->max_pos]<newValue) data->max_pos = data->next_pos;
    if(data->mat[data->min_pos]>newValue) data->min_pos = data->next_pos;
  }
  else
  {
    //if the maximum element was replaced, the position of the max element needs to be updated.
    if(data->next_pos == data->max_pos) maxElem(data);
    else
    {
      //if the minimum element was replaced, the position of the min element needs to be updated.
      if(data->next_pos == data->min_pos) minElem(data);
      else
      {
        //it the new element was inserted in one of the other positions, then only its magnitude needs to be
        //compared with the max and the min element
        if(data->mat[data->max_pos]<newValue) data->max_pos = data->next_pos;
        if(data->mat[data->min_pos]>newValue) data->min_pos = data->next_pos;
      }
    }
    //if the difference of the max and the min elements does not exceed the specified tolerance, then the
    //measurements stabilized and a DC offset can be extracted for this sensor.
    if(data->mat[data->max_pos] - data->mat[data->min_pos] < tolerance) data->ready = true;
  }

  data->count++;
  //The index for the next element is handled in a way to produce a cyclic buffer functionality.
  if(++data->next_pos >= BUF_SIZE) data->next_pos -= BUF_SIZE;
}
