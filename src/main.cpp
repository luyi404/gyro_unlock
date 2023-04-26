#include "mbed.h"
#include "display.hpp"
// Documents
// Manual for dev board: https://www.st.com/resource/en/user_manual/um1670-discovery-kit-with-stm32f429zi-mcu-stmicroelectronics.pdf
// gyroscope datasheet: https://www.mouser.com/datasheet/2/389/dm00168691-1798633.pdf

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

#define SPI_FLAG 1
#define DATA_READY_FLAG 2


#define DATA_POINTS 400 

#define THRESHOLD 650


uint8_t write_buf[32];
uint8_t read_buf[32];


constexpr int BUFFER_SIZE = 3 * DATA_POINTS; // 3 axes, 400 data points

// Init this array as 0
float key_buffer[BUFFER_SIZE] = {0};

int key_buffer_index = 0; // Used to index the key_buffer.


// Real time value buffer
float real_time_buffer[BUFFER_SIZE] = {0};

int real_time_buffer_index = 0; // Used to index the real_time_buffer.



EventFlags flags;
//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
  
};
void data_cb(){
  flags.set(DATA_READY_FLAG);

};

void get_data(uint8_t* read_buf, float& gx, float& gy, float& gz) {
  //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
  //Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
  int16_t raw_gx,raw_gy,raw_gz;
  raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
  raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
  raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );
  gx=((float)raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
  gy=((float)raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
  gz=((float)raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
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
    (key_buffer[i] - real_time_buffer[avoid_overflow(i++ + real_time_buffer_index)]);
  }
  printf("Sum: %f\n", sum);
  return sum < THRESHOLD;
}


void record_key(){
  key_buffer_index = 0;
  float gx, gy, gz;
  float sum{0};
  while(1) {
    //wait until new sample is ready
    flags.wait_all(DATA_READY_FLAG);
    //prepare the write buffer to trigger a sequential read
    write_buf[0]=OUT_X_L|0x80|0x40;

    //start sequential sample reading
    spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    get_data(read_buf, gx, gy, gz);
    if (key_buffer_index < BUFFER_SIZE) {
      key_buffer[key_buffer_index++] = gx;
      key_buffer[key_buffer_index++] = gy;
      key_buffer[key_buffer_index++] = gz;
      sum += gx * gx + gy * gy + gz * gz;
    } else {
      // printf("Recorded Sum: %f\n", sum);
      display_string_at_line_n<DATA_1>("Recorded Sum: %f\n", sum);
      break;
    }
  }
}

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

void print_key_value(){
  // // print the key_buffer
  for(int i =0; i < 400; i++) {
    // print gx gy gz individually
    printf("point %d: gx %f gy %f gz %f\n", i, key_buffer[i++], key_buffer[i++], key_buffer[i++]);
  }
}

DigitalIn user_button(USER_BUTTON);


int main() {
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate


  // SETUP FOR LCD DISPLAY
  setup_background_layer();
  setup_foreground_layer();
  // call these above setup once
  
  display_string_at_line_n<TITLE_1>("Welcome to the xxx");
  display_string_at_line_n<TITLE_2>("Press 2s to record");
  display_string_at_line_n<TITLE_3>("Will record %d points", DATA_POINTS);
  display_string_at_line_n<CUSTOM_1>("No key recorded");


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
  
  timer.start();
  
  while (1) {
    
    //Until the user button is pressed for 2s, we will be reading the gyroscope
    if (user_button.read() == 0) {
      timer.reset();
    } else {
      if (timer.read() > 2) {
        start_record = true;
      }
    }

    if (start_record) {
      have_key = true;
      matched = false;
      // printf("start recording\n");
      {
      // Clean up display line ACTION_1, ACTION_2, DATA_1, DATA_2
      display_string_at_line_n<ACTION_1>("                 ");
      display_string_at_line_n<ACTION_2>("                 ");
      display_string_at_line_n<DATA_1>("                 ");
      // sleep for 0.2s to show the clean up, hhh
      thread_sleep_for(200);
      }
      
      display_string_at_line_n<ACTION_1>("Start Recording");
      record_key();
      // printf("finish recording\n");
      display_string_at_line_n<ACTION_2>("Finish Recording");
      // print_key_value();
      start_record = false;
      timer.reset();
    }
    if(!have_key || matched) {
      continue;
    } else {
      display_string_at_line_n<CUSTOM_1>("Wating for match");
    }


    //wait until new sample is ready
    flags.wait_all(DATA_READY_FLAG);
    //prepare the write buffer to trigger a sequential read
    write_buf[0]=OUT_X_L|0x80|0x40;

    //start sequential sample reading
    spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    updata_real_time_buffer();
    if (match_data()) {
      // printf("match\n");
      matched = true;
      display_string_at_line_n<CUSTOM_1>("                   ");
      display_string_at_line_n<CUSTOM_2>("Match!");
      display_string_at_line_n<CUSTOM_3>("UNLOCK!");
      display_string_at_line_n<CUSTOM_4>("BUT NOTHING HAPPENED");
    }
    
    // printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",gx,gy,gz);
  }

  // Stop the timer and exit the program
  timer.stop();
  return 0;
}
