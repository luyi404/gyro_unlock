#include "mbed.h"
#include "display.hpp"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"


///* Gyro Reading setup parameters
SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs
InterruptIn int2(PA_2,PullDown);

#define OUT_X_L 0x28
//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
//register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22
//configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts                 
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

EventFlags flags;
#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define FILTER_SIZE 10

uint8_t write_buf[32];
uint8_t read_buf[32];

// Callback function
void spi_cb(int event){
  flags.set(SPI_FLAG);
};
void data_cb(){
  flags.set(DATA_READY_FLAG);
};
//*/



// Minimum Recording threshold & Matching error threshold
#define MINIMUM_RECORD_THRESHOLD 1000
int threshold{0};


// Moving Average Filter
float mean_filter_x[FILTER_SIZE] = {0};
float mean_filter_y[FILTER_SIZE] = {0};
float mean_filter_z[FILTER_SIZE] = {0};

int filter_index = 0;

float get_mean_f(float* arr, int size) {
  float sum{0};
  for (int i = 0; i < size; ++i) {
    sum += arr[i];
  }
  return sum / size;
}



// key and real-time buffer
#define DATA_POINTS 600 // recording for 3 seconds
constexpr const int BUFFER_SIZE = 3 * DATA_POINTS; // 3 axes, 600 data points

float key_buffer[BUFFER_SIZE] = {0};  // key buffer. Init this array as 0
int key_buffer_index = 0; // Used to index the key_buffer.

float real_time_buffer[BUFFER_SIZE] = {0}; // Real time value buffer
int real_time_buffer_index = 0; // Used to index the real_time_buffer.



// Filter raw data from read_buf and transfer data to float
void get_data(uint8_t* read_buf, float& gx, float& gy, float& gz) {
  int16_t raw_gx,raw_gy,raw_gz;
  raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
  raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
  raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

  // Add mean filter to the data
  mean_filter_x[filter_index] = (float)raw_gx;
  mean_filter_y[filter_index] = (float)raw_gy;
  mean_filter_z[filter_index] = (float)raw_gz;
  filter_index = (filter_index + 1) % FILTER_SIZE;
  raw_gx = get_mean_f(mean_filter_x, FILTER_SIZE);
  raw_gy = get_mean_f(mean_filter_y, FILTER_SIZE);
  raw_gz = get_mean_f(mean_filter_z, FILTER_SIZE);

  gx=((float)raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
  gy=((float)raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
  gz=((float)raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
}


///* Recording functions

void UI_start_recording(){
  // SETUP FOR LCD DISPLAY
  setup_background_layer();
  setup_foreground_layer();
  display_string_at_line_n<TITLE_1>("Will start in 3s.");
  thread_sleep_for(300);
  display_string_at_line_n<TITLE_1>("Will start in 3s..");
  thread_sleep_for(300);
  display_string_at_line_n<TITLE_1>("Will start in 3s...");
  thread_sleep_for(300);

  display_string_at_line_n<TITLE_1>("Will start in 2s.");
  thread_sleep_for(300);
  display_string_at_line_n<TITLE_1>("Will start in 2s..");
  thread_sleep_for(300);
  display_string_at_line_n<TITLE_1>("Will start in 2s...");
  thread_sleep_for(300);

  display_string_at_line_n<TITLE_1>("Will start in 1s.");
  thread_sleep_for(300);
  display_string_at_line_n<TITLE_1>("Will start in 1s..");
  thread_sleep_for(300);
  display_string_at_line_n<TITLE_1>("Will start in 1s...");
  thread_sleep_for(300);

  display_string_at_line_n<TITLE_1>("Start Recording...");

}

void UI_finish_recording(){ 
  display_string_at_line_n<TITLE_3>("Finish Recording!");
  thread_sleep_for(1000);
}

void UI_recording_failed(){ 
    display_string_at_line_n<TITLE_3>("Too Simple. Try Again");
    thread_sleep_for(500);

    display_string_at_line_n<DATA_1>("   Start Recording    ");
    lcd.DrawRect(23,145,190,40);
    
}

// record 1200 samples
bool record_key(){
  key_buffer_index = 0;
  real_time_buffer_index = 0;
  float gx, gy, gz;
  float sum{0};
  while(1) {
    //wait until new sample is ready and read data from gyro
    flags.wait_all(DATA_READY_FLAG);
    write_buf[0]=OUT_X_L|0x80|0x40;
    spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    get_data(read_buf, gx, gy, gz);
    if (key_buffer_index < BUFFER_SIZE) {
      key_buffer[key_buffer_index++] = gx;
      key_buffer[key_buffer_index++] = gy;
      key_buffer[key_buffer_index++] = gz;
      sum += gx * gx + gy * gy + gz * gz;

      real_time_buffer[real_time_buffer_index++] = 0;
      real_time_buffer[real_time_buffer_index++] = 0;
      real_time_buffer[real_time_buffer_index++] = 0;

    } else {
      real_time_buffer_index = 0;
      // if the recording is too simple, return false, recording failed 
      // if success, save dynamic matching threshold as sum * 0.25
      if (sum >= MINIMUM_RECORD_THRESHOLD) {
        threshold = sum * 0.35;
        return true;
      } else {
        return false;
      }
    }
  }
}
//*/


///* Match functions
void UI_awaiting_match(){ 
    display_string_at_line_n<DATA_2>("Waiting to match...");
}

void UI_matched(){
  setup_background_layer();
  setup_foreground_layer();

  lcd.DrawLine(80,130, 110,180);
  lcd.DrawLine(110,180, 150,85);
  lcd.DrawCircle(120,130,80);

  thread_sleep_for(250);
  display_string_at_line_n<CUSTOM_1>("Match!");
  thread_sleep_for(1000);
  display_string_at_line_n<CUSTOM_2>("UNLOCK!");
  thread_sleep_for(1000);
  display_string_at_line_n<CUSTOM_4>("Bye~");
}

inline int avoid_overflow(int original) {
  if (original >= BUFFER_SIZE) {
    original -= BUFFER_SIZE;
  }
  return original;
}

bool match_data() {
  float sum{0};
  for (int i = 0; i < BUFFER_SIZE; ++i) {
    sum += (key_buffer[i] - real_time_buffer[avoid_overflow(i + real_time_buffer_index)]) * 
    (key_buffer[i] - real_time_buffer[avoid_overflow(i + real_time_buffer_index)]);
  }
  return sum < threshold;
}
//*/


// update real_time_buffer
void updata_real_time_buffer() {
  float gx, gy, gz;
  get_data(read_buf, gx, gy, gz);

  if(real_time_buffer_index >= BUFFER_SIZE) {
    real_time_buffer_index = 0;
  }
  real_time_buffer[real_time_buffer_index++] = gx;

  if(real_time_buffer_index >= BUFFER_SIZE) {
    real_time_buffer_index = 0;
  }
  real_time_buffer[real_time_buffer_index++] = gy;

  if(real_time_buffer_index >= BUFFER_SIZE) {
    real_time_buffer_index = 0;
  }
  real_time_buffer[real_time_buffer_index++] = gz;
}



DigitalIn user_button(USER_BUTTON);


void UI_start(){
  // SETUP FOR LCD DISPLAY
  setup_background_layer();
  setup_foreground_layer();

  display_string_at_line_n<TITLE_1>("Welcome!");
  thread_sleep_for(1000);
  display_string_at_line_n<TITLE_3>("Hit the button to");
  display_string_at_line_n<TITLE_4>("start recording.");
  thread_sleep_for(1000);
  display_string_at_line_n<ACTION_1>("Recording will be");
  display_string_at_line_n<ACTION_2>("3 seconds long.");
  thread_sleep_for(1000);
  display_string_at_line_n<DATA_1>("   Start Recording    ");
  lcd.DrawRect(23,145,190,40);

}


uint16_t ts_x, ts_y;
TS_StateTypeDef TS_State;

bool detect_touch(){
    BSP_TS_GetState(&TS_State);
    if (TS_State.TouchDetected){
        ts_x = TS_State.X;
        ts_y = TS_State.Y;
        if ((23<ts_x && ts_x<213) && (145<ts_y && ts_y<185)){
            return true;
        }
    }
    return false;
}



int main() {

  // lcd and touch screen init
  LCD_DISCO_F429ZI lcd;
  TS_DISCO_F429ZI ts;

  UI_start();

  // gyro config
  spi.format(8,3);
  spi.frequency(1'000'000);

  write_buf[0]=CTRL_REG1;
  write_buf[1]=CTRL_REG1_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG4;
  write_buf[1]=CTRL_REG4_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);


  //configure the interrupt to call our function
  //when the pin becomes high
  int2.rise(&data_cb);


  write_buf[0]=CTRL_REG3;
  write_buf[1]=CTRL_REG3_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  // The gyroscope sensor keeps its configuration between power cycles.
  // This means that the gyroscope will already have it's data-ready interrupt
  // configured when we turn the board on the second time. This can lead to
  // the pin level rising before we have configured our interrupt handler.
  // To account for this, we manually check the signal and set the flag
  // for the first sample.
  if(!(flags.get()&DATA_READY_FLAG)&&(int2.read()==1)){
    flags.set(DATA_READY_FLAG);
  }

  Timer timer;
  bool start_record = false;
  bool have_key = false;
  bool matched = false;
  bool ready_to_record = true;
  
  timer.start();
  
  while (1) {
    
    // // if to use the user button but the touch screen
    // if (user_button.read() == 1) {
    //     while (user_button.read() == 1);
    //     thread_sleep_for(500);
    //     start_record = true;
    // }
    
    // wait until the user hits the button
    if (ready_to_record) {
        while (!detect_touch() && !user_button.read());
        start_record = true;
        ready_to_record = false;
    }

    if (start_record) {
      matched = false;
      
      UI_start_recording();
    
      if(record_key()){
        UI_finish_recording();
        have_key = true;
      } 
      else {
        UI_recording_failed();
        have_key = false;
        ready_to_record = true;
      }
      start_record = false;
      timer.reset();
    }
    
    if(!have_key || matched) {
      continue;
    } else {
      UI_awaiting_match();
    }

    // get new data from gyro
    flags.wait_all(DATA_READY_FLAG);
    write_buf[0]=OUT_X_L|0x80|0x40;
    spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    // convert the new data from read buffer and save to data buffer
    updata_real_time_buffer();

    if (match_data()) {
      matched = true;
      UI_matched();
    }
  }

  // Stop the timer and exit the program
  timer.stop();
  return 0;
}