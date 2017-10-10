#include "sensors.h"
#include "in4073.h"

typedef struct calibData calibData;

#define BUF_SIZE  80
#define ACC_TOLERANCE 120
#define GYRO_TOLERANCE 10

struct calibData
{
  int16_t mat[BUF_SIZE];
  uint8_t next_pos;
  uint8_t max_pos;
  uint8_t min_pos;
  uint32_t count;
  bool ready;
};

int16_t dcphi = 0, dctheta = 0, dcpsi = 0;
int16_t dcsp = 0, dcsq = 0, dcsr = 0;
int16_t dcsax = 0, dcsay = 0, dcsaz = 0;

void init_calibData(calibData *data);

void maxElem(calibData *data);

void minElem(calibData *data);

void insert_data(calibData *data, int16_t newValue, uint8_t tolerance);

bool calibration_flag = false;

void calibrate_sensors(void)
{
  /*Use a set of measurements from each sensor to detect when it stabilizes and evaluate its DC offset*/

  uint8_t i;
  uint8_t discardedSamp = 0;
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
    while(!check_sensor_int_flag());
    get_dmp_data();

    if(discardedSamp++ == 9)
    {
      //printf("%ld\n", get_time_us());
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
        insert_data(&calibArray[i], newSensorValues[i], GYRO_TOLERANCE);
      }
      for(i=6; i<9; i++)
      {
        insert_data(&calibArray[i], newSensorValues[i], ACC_TOLERANCE);
      }

      discardedSamp=0;
    }
  }

  //When all the sensors are considered stabilized, the average of the maximum and the minimum value of the
  //buffer is used as the DC offset to be subtracted.

  int32_t sum[9] = {0};
  uint8_t j;

  for(i=0; i<9; i++)
  {
    for(j=0; j<BUF_SIZE; j++) sum[i] += calibArray[i].mat[j];
  }

  dcphi = sum[0] / BUF_SIZE;
  dctheta = sum[1] / BUF_SIZE;
  dcpsi = sum[2] / BUF_SIZE;
  dcsp = sum[3] / BUF_SIZE;
  dcsq = sum[4] / BUF_SIZE;
  dcsr = sum[5] / BUF_SIZE;
  dcsax = sum[6] / BUF_SIZE;
  dcsay = sum[7] / BUF_SIZE;
  dcsaz = sum[8] / BUF_SIZE;

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
  int16_t max = data->mat[0];
  data->max_pos = 0;
  register uint8_t halfBuf = BUF_SIZE >>1;

  for(i=1; i<BUF_SIZE && i<data->count; i++)
  {
    if(data->mat[i] >= max)
    {
      if(!(data->mat[i] == max && (((i > data->next_pos) && (i <= data->next_pos + halfBuf)) || (i + halfBuf < data->next_pos))))
      {
        max = data->mat[i];
        data->max_pos = i;
      }
    }
  }
}

void minElem(calibData *data)
{
  /*The position of the minimum value of the elements of the sensor's buffer is found and stored in its corresponding member.*/
  uint8_t i;
  //Min and min position is initialized with the value of the first element
  int16_t min = data->mat[0];
  data->min_pos = 0;
  register uint8_t halfBuf = BUF_SIZE >>1;

  for(i=1; i<BUF_SIZE && i<data->count; i++)
  {
    if(data->mat[i] <= min)
    {
      //if the element on position i is equal to minimum but is going to be replaced sooner than the current min,
      if(!(data->mat[i] == min && (((i > data->next_pos) && (i <= data->next_pos + halfBuf)) || (i + halfBuf < data->next_pos))))
      {
        min = data->mat[i];
        data->min_pos = i;
      }
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

  if(data->count<BUF_SIZE)
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
      if(data->mat[data->max_pos]<=newValue) data->max_pos = data->next_pos;
    }
      //if the minimum element was replaced, the position of the min element needs to be updated.
    if(data->next_pos == data->min_pos) minElem(data);
    else
    {
      //it the new element was inserted in one of the other positions, then only its magnitude needs to be
      //compared with the max and the min element
      if(data->mat[data->min_pos]>=newValue) data->min_pos = data->next_pos;
    }
    //if the difference of the max and the min elements does not exceed the specified tolerance, then the
    //measurements stabilized and a DC offset can be extracted for this sensor.
    if(data->mat[data->max_pos] - data->mat[data->min_pos] < tolerance) data->ready = true;
  }

  data->count++;
  //printf("%ld | count = %ld | buf[%d] = %d | max[%d] = %d | min[%d] = %d\n", get_time_us(), data->count, data->next_pos, data->mat[data->next_pos], data->max_pos, data->mat[data->max_pos], data->min_pos, data->mat[data->min_pos]);
  //The index for the next element is handled in a way to produce a cyclic buffer functionality.
  if(++data->next_pos >= BUF_SIZE) data->next_pos -= BUF_SIZE;

}
