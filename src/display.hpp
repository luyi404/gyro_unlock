#include <mbed.h>
//this file has all the functions for interacting
//with the screen
#include "drivers/LCD_DISCO_F429ZI.h"
#define BACKGROUND 1
#define FOREGROUND 0
#define GRAPH_PADDING 5


LCD_DISCO_F429ZI lcd;
//buffer for holding displayed text strings
char display_buf[60];
uint32_t graph_width=lcd.GetXSize()-2*GRAPH_PADDING;
uint32_t graph_height=graph_width;

//sets the background layer 
//to be visible, transparent, and
//resets its colors to all black
void setup_background_layer(){
  lcd.SelectLayer(BACKGROUND);
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_GREEN);
  lcd.SetLayerVisible(BACKGROUND,ENABLE);
  lcd.SetTransparency(BACKGROUND,0x7Fu);
}

//resets the foreground layer to
//all black
void setup_foreground_layer(){
    lcd.SelectLayer(FOREGROUND);
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTGREEN);
}

//draws a rectangle with horizontal tick marks
//on the background layer. The spacing between tick
//marks in pixels is taken as a parameter
void draw_graph_window(uint32_t horiz_tick_spacing){
  lcd.SelectLayer(BACKGROUND);
  
  lcd.DrawRect(GRAPH_PADDING,GRAPH_PADDING,graph_width,graph_width);
  //draw the x-axis tick marks
  for (int32_t i = 0 ; i < graph_width;i+=horiz_tick_spacing){
    lcd.DrawVLine(GRAPH_PADDING+i,graph_height,GRAPH_PADDING);
  }
}

//maps inputY in the range minVal to maxVal, to a y-axis value pixel in the range
//minPixelY to MaxPixelY
uint16_t mapPixelY(float inputY,float minVal, float maxVal, int32_t minPixelY, int32_t maxPixelY){
  const float mapped_pixel_y=(float)maxPixelY-(inputY)/(maxVal-minVal)*((float)maxPixelY-(float)minPixelY);
  return mapped_pixel_y;
}


// ABOVE if from the LCD display example

template <std::size_t line>
void display_string_at_line_n(const char *ptr,  ...) {
    // use static_assert to check if the line is valid at compile-time
    static_assert(line <= 17, "Line number is invalid");

    lcd.ClearStringLine(line);
    va_list args;
    va_start(args, ptr);
    vsnprintf(display_buf, 60, ptr, args);
    va_end(args);
    lcd.SelectLayer(FOREGROUND);
    //display the buffered string on the screen
    lcd.DisplayStringAt(0, LINE(line), (uint8_t *)display_buf, LEFT_MODE);
}

enum DisplayLine {
    TITLE_1 = 1,
    TITLE_2,
    TITLE_3,
    TITLE_4,
    ACTION_1 = 6,
    ACTION_2,
    ACTION_3,
    DATA_1 = 10,
    DATA_2,
    DATA_3,
    CUSTOM_1 = 14,
    CUSTOM_2,
    CUSTOM_3,
    CUSTOM_4,
};