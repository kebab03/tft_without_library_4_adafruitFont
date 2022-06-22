 
#include "pins.h"
#include "variabile.h"
#include "struttu_font.h"
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define RED2    0x4000
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREEN2  0x2FA4
#define CYAN2   0x07FF
//https://electronicseternit.wixsite.com/electronicsforlife/post/using-2-4-tft-lcd-display-without-library
#define LCD_RD   A0 //Serves as read signal/MCU read data at the rising edge. Pg11 Datasheet
#define LCD_WR   A1 //Serves as write signal/command at the rising edge
#define LCD_RS   A2 //D/CX (0=Command/1=Data)       
#define LCD_CS   A3 //Chip Select Pin : Active Low
#define LCD_RST  A4 //Shield Reset
   
  

#include "font.h"
   uint16_t  decodeUTF8(uint8_t *buf, uint16_t *index, uint16_t remaining)
{Serial.println("line 3944 di  decodeUTF8 TFT_eSPI.cpp");
Serial.println("n=index of char in string ");
Serial.println(*index);
  uint16_t c = buf[(*index)++];
  Serial.print(" line 3978 c = buf[(*index)++]=");
Serial.println(c);
  //Serial.print("Byte from string = 0x"); Serial.println(c, HEX);

   

  // 7 bit Unicode
  if ((c & 0x80) == 0x00) return c;

  // 11 bit Unicode
  if (((c & 0xE0) == 0xC0) && (remaining > 1))
    return ((c & 0x1F)<<6) | (buf[(*index)++]&0x3F);

  // 16 bit Unicode
  if (((c & 0xF0) == 0xE0) && (remaining > 2)) {
    c = ((c & 0x0F)<<12) | ((buf[(*index)++]&0x3F)<<6);
    return  c | ((buf[(*index)++]&0x3F));
  }

  // 21 bit Unicode not supported so fall-back to extended ASCII
  // if ((c & 0xF8) == 0xF0) return c;
Serial.println("line 3999  fine decodeuFT8 n=index");
  return c; // fall-back to extended ASCII
}

 void setFreeFont(const GFXfont *f)
{ Serial.println(F("sono in line 5248 di  setfreefont TFT_eSPI.cpp="));
  if (f == nullptr) { // Fix issue #400 (ESP32 crash)
   Serial.println(F("no font")); // Use GLCD font
    return;
  }

  textfont = 1;
  gfxFont = (GFXfont *)f;

  glyph_ab = 0;
  glyph_bb = 0;
  uint16_t numChars = pgm_read_word(&gfxFont->last) - pgm_read_word(&gfxFont->first);
Serial.print(F("line 5336 numChars="));
  Serial.println(numChars );
  Serial.print(F(" line 5336 pgm_read_word(&gfxFont->last)="));
  Serial.println(  pgm_read_word(&gfxFont->last));
  // Find the biggest above and below baseline offsets
  for (uint8_t c = 0; c < numChars; c++) {
    GFXglyph *glyph1  = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c]);
    int8_t ab = -pgm_read_byte(&glyph1->yOffset);
    Serial.print(F("line 5310 yOffset ab="));
  Serial.println(ab);

    if (ab > glyph_ab) glyph_ab = ab;
    int8_t bb = pgm_read_byte(&glyph1->height) - ab;
    if (bb > glyph_bb) glyph_bb = bb;

  Serial.print(F("line 5315 glyph_ab="));
  Serial.println( glyph_ab);
  Serial.print(F("line 5329 (height) - ab, glyph_bb,dist dal baseline="));
  Serial.println( glyph_bb);

  }

}




void LCD_write(uint8_t d){
  // Serves as write signal/command at the rising edge
  digitalWrite(LCD_WR,LOW); // WR 0
//  Serial.print(F("d=DATA/COMMAND IN 8 BIT :LCD_WR,LOW:=="));
//  Serial.println(d,BIN);
//    Serial.print(F("0<<3))=="));
//  Serial.println((0<<3),BIN);
//    Serial.print(F("(d&(1<<0))=="));
//  Serial.println((d&(1<<0)),BIN);
  //static uint16_t PA_Set=((((d)&(1<<0))<<9)|(((d)&(1<<2))<<8)|(((d)&(1<<7))<<1));
  uint16_t PA_Set1=((d&(1<<0))<<9);
//  Serial.print(F("PA_Set1=="));
//  Serial.println(PA_Set1,BIN);
  uint16_t PA_Set2=((d &(1<<2))<<8);
//  Serial.print(F("PA_Set2=="));
//  Serial.println(PA_Set2,BIN);
  uint16_t PA_Set3=((d &(1<<7))<<1);
//  Serial.print(F("PA_Set3=="));
//  Serial.println(PA_Set3,BIN);
  uint16_t PA_Set=PA_Set1|PA_Set2|PA_Set3;

   uint16_t PB_Set=(((d)&(1<<3))|(((d)&(1<<4))<<1)|(((d)&(1<<5))>>1)|(((d)&(1<<6))<<4));
   uint16_t PC_Set=(((d)&(1<<1))<<6);
   uint32_t GPIOA_AD=GPIOA->ODR;
   uint32_t GPIOA_MSK=GPIOA_AD|PA_Set;
   uint32_t GPIOB_AD=GPIOB->ODR;
   uint32_t GPIOC_AD=GPIOC->ODR;
//  Serial.print(F("GPIOC_AD::=="));
//  Serial.println(GPIOC_AD,BIN);
//  Serial.print(F("PC_Set=="));
//  Serial.println(PC_Set,BIN);
//  Serial.print(F("GPIOA_MSK in 8 BIT :IN BIN FORMAT:=="));
//  Serial.println(GPIOA_MSK,BIN); 
//  Serial.print(F("GPIOA_AD CIOè QUELLO CHE HO IN QUESTI REGIS::=="));
//  Serial.println(GPIOA_AD,BIN);
//  Serial.print(F("PA_Set=="));
//  Serial.println(PA_Set,BIN);
//  Serial.print(F("GPIOB_AD::=="));
//  Serial.println(GPIOB_AD,BIN);
//  Serial.print(F("PB_Set=="));
//  Serial.println(PB_Set,BIN);
//  Serial.print(F("GPIOC_AD::=="));
//  Serial.println(GPIOC_AD,HEX);
//  Serial.print(F("PC_Set=="));
//  Serial.println(PC_Set,BIN);
   uint16_t PA_Mask=~((1<<10)|(1<<9)|(1<<8));
   uint16_t PB_Mask=~((1<<3)|(1<<4)|(1<<5)|(1<<10));
   uint16_t PC_Mask=~(1<<7);
  uint16_t TEM_REG;
 TEM_REG =GPIOA->ODR ;
 //**Serial.print(F("TEM_REG =GPIOA->ODR:=="));
  //**Serial.println(TEM_REG,BIN);
     //**Serial.print(F("TEM_REG:PRIMA DI TEM_REG &=PA_Mask; :=="));
  //**Serial.println(TEM_REG,BIN);
   TEM_REG &=PA_Mask;
   GPIOA->ODR =TEM_REG;
  GPIOA->ODR |=PA_Set; //((((d)&(1<<0))<<9)|(((d)&(1<<2))<<8)|(((d)&(1<<7))<<1));
  //**Serial.print(F("TEM_REG:A:=="));
  //**Serial.println(TEM_REG,BIN);
  TEM_REG =GPIOB->ODR;
  TEM_REG &=PB_Mask;
  GPIOB->ODR =TEM_REG;
  GPIOB->ODR  |=(((d)&(1<<3))|(((d)&(1<<4))<<1)|(((d)&(1<<5))>>1)|(((d)&(1<<6))<<4));
  //**Serial.print(F("TEM_REG:B:=="));
  //**Serial.println(TEM_REG,BIN);
  TEM_REG =GPIOC->ODR ;
  TEM_REG &=PC_Mask;
  GPIOC->ODR =TEM_REG;
  GPIOC->ODR  |=PC_Set;
  //**Serial.print(F("TEM_REG:C:=="));
  //**Serial.println(TEM_REG,BIN);
digitalWrite(LCD_WR,HIGH); // WR 1

//   uint32_t GPIOA_ADR=GPIOA->ODR;
//   uint32_t GPIOB_ADR=GPIOB->ODR;
//   uint32_t GPIOC_ADR=GPIOC->ODR;
//**Serial.println(F("******DOPO RESETING I BIT DEI PIN *"));
 //** Serial.print(F("GPIOA_ADR::=="));
  //**Serial.println(GPIOA_ADR,BIN);
  
//**Serial.print(F("GPIOB_ADR::=="));
 //** Serial.println(GPIOB_ADR,BIN);

  
  //**Serial.print(F("GPIOC_ADR::=="));
  //**Serial.println(GPIOC_ADR,BIN);
//**Serial.println(F("******LCD_write line 44*"));
}


void LCD_command_write(uint8_t command) {
  // LCD_RS = 0, A2=0, D/CX (0=Command/1=Data) | DataSheet Page 11
  digitalWrite(LCD_RS, LOW);
  
   //**Serial.println(F("******LCD_command_write line 50*"));
  //** Serial.print(F("******LCD_command======::=="));
   //**Serial.println(command,BIN);
   LCD_write(command);  
}

void LCD_data_write(uint8_t data) {
  // LCD_RS = 1, A2=1, D/CX (0=Command/1=Data) | DataSheet Page 11
  digitalWrite(LCD_RS, HIGH);
  //**Serial.println(F("******LCD_data_write line 55*"));
  LCD_write(data);
}

void Lcd_Init(void) {
  //void does not return any value
  //void only execute instruction within it
  //similar to void setup and loop
  //This function will have LCD initialization measures
  //Only the necessary Commands are covered
  //Eventho there are so many more in DataSheet

  //Reset Signal is Active LOW
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW); //Actual Reset Done Here
  delay(15);
  digitalWrite(LCD_RST, HIGH);
  delay(15);

  //The below is just preparation for Write Cycle Seq
  digitalWrite(LCD_CS, HIGH); //Chip-Select Active Low Signal
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_CS, LOW);  //Chip-Select Active Low Signal
//**Serial.println(F(" Innitiate  LINE--79 FOR Breakout board"));
  LCD_command_write(0xC5);    //Test this Out | VCOM Control 1 : Colour Contrast Maybe
  LCD_data_write(0x54);       //VCOM H 1111111 0x7F
  LCD_data_write(0x00);       //VCOM L 0000000
  //LCD_data_write(B1010011);
  
  LCD_command_write(0x36);    //Memory Access Control | DataSheet Page 127
  ///LCD_data_write(0x48);       //Adjust this value to get right color and starting point of x and y
  LCD_data_write(B0000100);     //Example

  LCD_command_write(0x3A);    //COLMOD: Pixel Format Set | DataSheet Page 134
  LCD_data_write(0x55);       //16 Bit RGB and MCU

  LCD_command_write(0x11);    //Sleep Out | DataSheet Page 245
  delay(6);                  //Necessary to wait 5msec before sending next command

  LCD_command_write(0x29);    //Display on.

  LCD_command_write(0x2c);    //Memory Write | DataSheet Page 245
}

void Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2) {//**Serial.println(F(" Address_set  LINE--100 FOR Breakout board"));
  int diff_x=x2-x1;
  int diff_y=y2-y1;
    Serial.println("line 3111 di TFT_eSPI.cpp diff_x =larghezza dei blicchi:");
          Serial.println(diff_x);
      Serial.print ("line 3109 di TFT_eSPI.cpp diff_y =altezza dei blicchi:");
         Serial.println(diff_y);
        
         Serial.print("line 3064 di TFT_eSPI. larghezza, altezza dei blicchi= ");

          Serial.print(diff_x);

         Serial.print(",");
        Serial.println(diff_y);
  LCD_command_write(0x2a);  //Column Address Set | DataSheet Page 110
  LCD_data_write(y1 >> 8);  //8 Bit Shift Right of y1
  LCD_data_write(y1);       //Value of y1
  LCD_data_write(y2 >> 8);  //8 Bit Shift Right of y2
  LCD_data_write(y2);       //Value of y2

  LCD_command_write(0x2b);  //Page Address Set | DataSheet Page 110
  LCD_data_write(x1 >> 8);  //8 Bit Shift Right of x1
  LCD_data_write(x1);       //Value of x1
  LCD_data_write(x2 >> 8);  //8 Bit Shift Right of x2
  LCD_data_write(x2);       //Value of x2

  LCD_command_write(0x2c); // REG 2Ch = Memory Write
}

void drawPixel(int16_t x, int16_t y, uint16_t color) { //**Serial.println(F("******DRAW PIXEL  LINE--120 FOR Breakout board **** #define write8  *********"));
  digitalWrite(LCD_CS, LOW);// Chip Select active
  Address_set(y, y + 1, x, x + 1);
  //
  LCD_command_write(0x2C);
  LCD_data_write(color >>8);
  LCD_data_write(color);
}

uint16_t selectColour(float redpercentage, float greenpercentage, float bluepercentage) {
  //use uint16_t instead of void as we need some values in return
  //int does not work with division
  //hence, we're using float instead of int

  redpercentage   = (redpercentage / 100) * 31;
  greenpercentage = (greenpercentage / 100) * 63;
  bluepercentage  = (bluepercentage / 100) * 31;

  int red = round(redpercentage);
  int green = round(greenpercentage);
  int blue = round(bluepercentage);

  uint16_t color_16_bit = 0;
  color_16_bit = (red << 11) + (green << 5) + blue;
  Serial.print("color_16_bit in  BIN :=");
  Serial.println(color_16_bit);
  Serial.print("color_16_bit in  HEX :=");
  Serial.println(color_16_bit,HEX);
  return color_16_bit;
}


void drawLine(float x1, float y1, float x2, float y2, uint16_t color) {
 if (x2 > x1) {
   for (int xinc = x1; xinc < x2; xinc++) { //xinc = x=incremental
     float m = (y2 - y1) / (x2 - x1); //m=gradient
     float yinc = m * (xinc - x1) + y1;
     //Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2)
     Address_set(y1, yinc, x1, xinc);
     LCD_data_write(color >> 8);
     LCD_data_write(color);
     y1 = yinc;
     x1 = xinc;
   }
 }
 else {
   float newx1 = x2; float newx2 = x1; float newy1 = y2; float newy2 = y1;
   for (int xinc = newx1; xinc < newx2; xinc++) { //xinc = x=incremental
     float m = (newy2 - newy1) / (newx2 - newx1); //m=gradient
     float yinc = m * (xinc - newx1) + newy1;
     //Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2)
     Address_set(newy1, yinc, newx1, xinc);
     LCD_data_write(color >> 8);
     LCD_data_write(color);
     newy1 = yinc;
     newx1 = xinc;
   }
 }
}

void filledRectangle(int16_t rectx1, int16_t recty1, int16_t wide, int16_t tall, uint16_t color) {
 //draw left line
 int16_t y1 = recty1;
 int16_t x1 = rectx1;
 int16_t y2 = recty1 + tall;
 int16_t x2 = rectx1 + wide;

 Address_set(y1, y2, x1, x2);
 for (int i = 0; i < wide + 1; i++)
   for (int j = 0; j < tall + 1; j++) {
     LCD_data_write(color >> 8);
     LCD_data_write(color);
   }
}



void hollowRectangle(int16_t rectx1, int16_t recty1, int16_t wide, int16_t tall, uint16_t color) {
 //draw left line
 int16_t lefty1 = recty1;
 int16_t leftx1 = rectx1;
 int16_t lefty2 = recty1 + tall;
 int16_t leftx2 = rectx1 + 1;
 for (int i = lefty1; lefty1 < lefty2; lefty1++) {
   //Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2)
   Address_set(lefty1, lefty2, leftx1, leftx2);
   LCD_data_write(color >> 8);
   LCD_data_write(color);
 }

 //draw left line
 int16_t righty1 = recty1;
 int16_t rightx1 = rectx1;
 int16_t righty2 = recty1 + tall;
 int16_t rightx2 = rectx1 + wide;
 for (int i = righty1; righty1 < righty2; righty1++) {
   //Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2)
   Address_set(righty1, righty2, rightx2, rightx2 + 1);
   LCD_data_write(color >> 8);
   LCD_data_write(color);
 }

 //draw top line
 int16_t topy1 = recty1 + tall;
 int16_t topx1 = rectx1;
 int16_t topy2 = recty1 + tall;
 int16_t topx2 = rectx1 + wide;
 for (int i = topx1; topx1 < topx2 + 1; topx1++) {
   //Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2)
   Address_set(topy1, topy2 + 1, topx1, topx2 + 1);
   LCD_data_write(color >> 8);
   LCD_data_write(color);
 }

 //draw bottom line
 int16_t bottomy1 = recty1;
 int16_t bottomx1 = rectx1;
 int16_t bottomy2 = recty1;
 int16_t bottomx2 = rectx1 + wide;
 for (int i = bottomx1; bottomx1 < bottomx2 + 1; bottomx1++) {
   //Address_set(int16_t y1, int16_t y2, int16_t x1, int16_t x2)
   Address_set(bottomy1, bottomy2 + 1, bottomx1, bottomx2 + 1);
   LCD_data_write(color >> 8);
   LCD_data_write(color);
 }
}

void hollowCircle(float x1, float y1, float radius, uint16_t color) {
 float leftx1 = x1 - radius;
 float rightx1 = x1 + radius;
 float bottomy1 = y1 - radius;
 float topy1 = y1 + radius;

 for ( float i = leftx1; i < rightx1; i = i + 0.5) {
   //circle formula (x-h)^2 + (y-k)^2 = r^2
   float movingy = y1 + sqrt(pow(radius, 2) - pow((i - x1), 2));
   drawPixel(i, movingy, color);
   movingy = y1 - sqrt(pow(radius, 2) - pow((i - x1), 2));
   drawPixel(i, movingy, color);
 }

 for ( float j = bottomy1; j < topy1; j = j + 0.5) {
   float movingx = x1 + sqrt(pow(radius, 2) - pow((j - y1), 2));
   drawPixel(movingx, j, color);
   movingx = x1 - sqrt(pow(radius, 2) - pow((j - y1), 2));
   drawPixel(movingx, j, color);
 }
}
//0x67 =100 0011
uint16_t  decodeUTF8(uint8_t c)
{Serial.print("line 3895");
  
Serial.print("line 3900 c:=");
Serial.println(c,HEX);
  // 7 bit Unicode Code Point
  if ((c & 0x80) == 0x00) {Serial.print("line 3901 c:=");
Serial.println(c,HEX);
    decoderState = 0;
    return c;
  }

  if (decoderState == 0) {Serial.print("line 3902 c:=");
Serial.println(c,HEX);
    // 11 bit Unicode Code Point
    if ((c & 0xE0) == 0xC0) {
      decoderBuffer = ((c & 0x1F)<<6);
      decoderState = 1;
      return 0;
    }
    // 16 bit Unicode Code Point
    if ((c & 0xF0) == 0xE0) {
      decoderBuffer = ((c & 0x0F)<<12);
      decoderState = 2;
      return 0;
    }
    // 21 bit Unicode  Code Point not supported so fall-back to extended ASCII
    // if ((c & 0xF8) == 0xF0) return c;
  }
  else {
    if (decoderState == 2) {
      decoderBuffer |= ((c & 0x3F)<<6);
      decoderState--;
      return 0;
    }
    else {Serial.print("line 3904 c:=");
Serial.println(c,HEX);
      decoderBuffer |= (c & 0x3F);
      decoderState = 0;
      return decoderBuffer;
    }
  }

  decoderState = 0;
Serial.print("line 3915 c:=");
Serial.println(c);
  return c; // fall-back to extended ASCII
}
//font =1
int16_t textWidth(const char *string, uint8_t font)
{Serial.println(F(" line 2768 -----------------------------------inizio  di tft textWidth::=di TFT_eSPI_STM32.cpp="));
  int32_t str_width = 0;
  uint16_t uniCode  = 0;
Serial.println(F(" line 2771 di tft textWidth::=di TFT_eSPI_STM32.cpp="));
Serial.println(F(" line 2773 di tft textWidth::=di TFT_eSPI_STM32.cpp="));
  
    if(1) { // New font
      while (*string) {Serial.println(F(" line 2811 di tft textWidth::=di TFT_eSPI_STM32.cpp="));
        uniCode = decodeUTF8(*string++);
        Serial.print(F(" line 2814 di tft textWidth::=di unicodep="));
        Serial.println(uniCode );
        if ((uniCode >= pgm_read_word(&gfxFont->first)) && (uniCode <= pgm_read_word(&gfxFont->last ))) {

          uniCode -= pgm_read_word(&gfxFont->first);
          GFXglyph *glyph  = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[uniCode]);
          // If this is not the  last character or is a digit then use xAdvance
          if (*string  || isDigits) {Serial.println(F(" line 2821 di tft textWidth::=di TFT_eSPI_STM32.cpp="));
            Serial.println( str_width);

            str_width += pgm_read_byte(&glyph->xAdvance);
            Serial.print(F(" line 2825 di tft textWidth::=di str_width="));
            Serial.println( str_width);
          // Else use the offset plus width since this can be bigger than xAdvance
          }
          else{Serial.print(F(" line 2828 di tft textWidth::=di str_width="));
         Serial.println( str_width);
         int widt=pgm_read_byte(&glyph->width);
         Serial.print(F(" line 2829 di tft textWidth::=di width="));
         Serial.println( widt);
           str_width += ((int8_t)pgm_read_byte(&glyph->xOffset) + pgm_read_byte(&glyph->width));
        Serial.print(F(" line 2832 di tft textWidth::=di str_width="));
         Serial.println( str_width);}
        }
      }
    }
   
  
  Serial.println(F(" line 2833-----------------------------------Fine  di tft textWidth::=di TFT_eSPI_STM32.cpp="));
   textsize=4;
  return str_width * textsize;
}

void  drawChar(int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size)
{  Serial.println("line 2881 drawChar( x, y,cha c,color, bg, size) for gfx font  0b11000110,BIN=======******2877**");
   
      Serial.print("line 2967 prima di leggere c-= pgm_read_word(&gfxFont->first)");
      c -= pgm_read_word(&gfxFont->first);
      GFXglyph *glyph  = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c]);
      uint8_t  *bitmap = (uint8_t *)pgm_read_dword(&gfxFont->bitmap);
      Serial.println(c,HEX);
      uint32_t bo = pgm_read_word(&glyph->bitmapOffset);
      Serial.print("line 2973  bo=");
      Serial.println(bo);
      uint8_t  w  = pgm_read_byte(&glyph->width),
               h  = pgm_read_byte(&glyph->height);
               //xa = pgm_read_byte(&glyph->xAdvance);
      int8_t   xo = pgm_read_byte(&glyph->xOffset),
               yo = pgm_read_byte(&glyph->yOffset);
      Serial.print("line 2980  xo=");
      Serial.println(xo);
      Serial.print("line 2982  yo=");
      Serial.println(yo);
      uint8_t  xx, yy, bits=0, bit=0;
      int16_t  xo16 = 0, yo16 = 0;

      Serial.print("line 2981 width of char  w=");
      Serial.println(w);

      Serial.print("line 2984 height of char h=");
      Serial.println(h);

      if(size > 1) {   Serial.println("line 2993 size >1");
        xo16 = xo;
        yo16 = yo;
      Serial.print("line 2980  xo16=");
      Serial.println(xo16);
      Serial.print("line 2982  yo16=");
      Serial.println(yo16);
      }



      uint16_t hpc = 0; // Horizontal foreground pixel count
      for(yy=0; yy<h; yy++) {        Serial.print("line 3000 yy=");
      Serial.println(yy);
               for(xx=0; xx<w; xx++)  {   Serial.print("line 3001 xx=");
      Serial.println(xx);
          if(bit == 0)                {   Serial.print("line 3002  bo++=");      //int32_t BB=bo;
      uint32_t Bo=bo; //Serial.println(Bo,HEX);
            Serial.println(++Bo);
            bits = pgm_read_byte(&bitmap[bo++]);
            Serial.println("line 3014 bits letto bo++ =");
            Serial.print("line 3015 bits letto bo è line di prima TFT_eSPI=");
            Serial.println(bits,HEX);
            Serial.print("line 3017 bits letto bo è line di prima TFT_eSPI=");
            Serial.println(bits,BIN);
            bit  = 0x80;
      Serial.print("line 3020 bit=");Serial.println(bit,HEX);                                   }
          if(bits & bit) { hpc++;Serial.print("line 3024 hpc=");      Serial.println(hpc);                                           }
          else {  if (hpc) {Serial.println("line 3027 hpc>1 ");
      Serial.print("line 3028 size= textsize=");
      Serial.println(size);
              if(size == 1) drawLine(x+xo+xx-hpc, y+yo+yy, x+xo+xx,y+yo+yy, color);
              else {Serial.println("line 3031 chiama filledRectangle(x+(xo16+xx-hpc)*size, y+(yo16+yy)*size, size*hpc, size, color)");
      Serial.println("line 3032 delay(1500) hpc=è il pl di  TFT_eSPI font ");                
      Serial.print("line 3034 yo16=");
      Serial.println(yo16);
      Serial.print("line 3036 yy=");
      Serial.println(yy);
      Serial.print("line 3038 y=");
      Serial.println(y);delay(1500);
      //filledRectangle(80, 80, 160, 80, selectColour(0, 0, 100));
      //filledRectangle(int16_t rectx1, int16_t recty1, int16_t wide, int16_t tall, uint16_t color)
     // filledRectangle(x+(xo16+xx-hpc)*size, y+yo16*size,x+(xo16+xx)*size, y+(yo16+yy)*size, color);
      filledRectangle(x+(xo16+xx-hpc)*size, y+(yo16+yy)*size, size*hpc, size, GREEN);
      Serial.print("line 3043 hpc=");Serial.println(hpc);}
              hpc=0;  }  }
          bit >>= 1;
      Serial.print("line 3050 bit=");
      Serial.println(bit,HEX);
      Serial.print("line 3052 bit=");Serial.println(bit,BIN);
        } // Draw pixels for this line as we are about to increment yy

        
     }
}

// Without font number, uses font set by setTextFont()
int16_t  drawString(const char *string, int32_t poX, int32_t poY)
{Serial.print("line 4842 textfont=");
Serial.println( textfont);
  return drawString4(string, poX, poY, textfont);
}






int16_t drawString4(const char *string, int32_t poX, int32_t poY, uint8_t font)
{   // textsize=4;  
      Serial.println("line 4804 ||||||||||||||||||||||||||||||||||Inizio di drawString(*string,poX,poY,font)");
      Serial.print("line 4882 in drawString TFT_eSPI.cpp::poX:=");
      Serial.println(poX);
      Serial.print("line 4884 in drawString TFT_eSPI.cpp::poY=");
      Serial.println(poY);
      int16_t sumX = 0;
  uint8_t padding = 1, baseline = 0;
  uint16_t cwidth = textWidth(string, font); // Find the pixel width of the string in the font
  uint16_t cheight = 8 * textsize;
      Serial.println("line 4811  cwidth = textWidth( in drawString di TFT_eSPI.cpp==");
      Serial.print("line 4894  cwidth =xOffset +width=");
      Serial.println(cwidth);
bool freeFont = (font == 1 && gfxFont);
 if (freeFont){Serial.println("line 4903 if (freeFont)  vero");
      Serial.println("line 4904 if (freeFont)  vero");
      Serial.print("line 4905  ultimo valore di max yOffset glyph_ab=");
      Serial.println( glyph_ab);
    cheight = glyph_ab * textsize;
    poY += cheight; // Adjust for baseline datum of free fonts

    baseline = cheight;
    padding =101; // Different padding method used for Free Fonts
      Serial.print("line 4908 drawString   TFT_eSPI.cpp::poY +=cheight = glyph_ab * textsize:=");
      Serial.println(poY);
            Serial.print("line 4910 drawString   TFT_eSPI.cpp::baseline:=");
      Serial.println(baseline);

      int8_t xo = 0;

  if (freeFont && (textcolor!=textbgcolor)) {
    Serial.print("line 5004 textcolor=WHILE LINE 369 BY DEFAULT");
    Serial.println("textbgcolor=BLACK LINE 370 BY DEFAULT");
    Serial.print ("line 5006 di drawString TFT_eSPI.cpp glyph_ab=");
    Serial.println(glyph_ab);
    Serial.print("line 5008 di drawString TFT_eSPI.cppglyph_bb=");
    Serial.println(glyph_bb);
      cheight = (glyph_ab + glyph_bb) * textsize;
      Serial.println("line 5011 LOAD_GFXFF & freeFont && (textcolor!=textbgcolor) troe TFT_eSPI.cpp");
      Serial.println(cheight);
      // Get the offset for the first character only to allow for negative offsets
      uint16_t c2 = 0;
      uint16_t len = strlen(string);
      uint16_t n = 0;
Serial.println("line 5017 di drawString TFT_eSPI.cpp");
while (n < len && c2 == 0) c2 = decodeUTF8((uint8_t*)string, &n, len - n);

      if((c2 >= pgm_read_word(&gfxFont->first)) && (c2 <= pgm_read_word(&gfxFont->last) )) {
        c2 -= pgm_read_word(&gfxFont->first);
        GFXglyph *glyph = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c2]);
        xo = pgm_read_byte(&glyph->xOffset) * textsize;
        // Adjust for negative xOffset
        if (xo > 0) xo = 0;
        else cwidth -= xo;
        // Add 1 pixel of padding all round
        //cheight +=2;
        //filledRectangle(poX+xo-1, poY - 1 - glyph_ab * textsize, cwidth+2, cheight, textbgcolor);
    Serial.println("line 5024 di drawString TFT_eSPI.cpp xo=");
    Serial.println(xo );
    int cord =glyph_ab * textsize;
    Serial.print ("line 5027 di drawString TFT_eSPI.cpp cord=");
    Serial.println(cord );
     int cordi =glyph_ab * textsize;
    Serial.print ("line 5030 di drawString TFT_eSPI.cpp poY=");
    Serial.println(poY );
        Serial.println("line 5032 filledRectangle(poX+xo, poY - glyph_ab * textsize, cwidth, cheight, textbgcolor)");
        filledRectangle(poX+xo, poY - glyph_ab * textsize, cwidth, cheight, textbgcolor);
      }
      padding -=100;
    }


  uint16_t len = strlen(string);
  Serial.print("line 4719 di string len = strlen(string) TFT_eSPI_STM.cpp:=");
  Serial.println(len);
  uint16_t n = 0;


  
    while (n < len) {
      Serial.println("line 5070 di drawString ora chiama decodeUTF8 TFT_eSPI_STM.c");
      uint16_t uniCode = decodeUTF8((uint8_t*)string, &n, len - n);
      Serial.print("line 5072 di drawString decodeUTF8 TFT_eSPI_STM.c:=prima di -32 uniCode ==");
      Serial.println(uniCode);
       Serial.print("line 5074 di drawString decodeUTF8 TFT_eSPI_S poY=");
      Serial.println(poY);
Serial.println("line 5076 chiama drawchar(uniCode, poX+sumX, poY, font)");

//drawChar(int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size)
     //
      sumX += drawChar_sumX(uniCode, poX+sumX, poY, font);//drawChar( 2, 4, "g", 0xff21, 0x0321, 4);
      Serial.print("line 5078 di drawString sumX += drawChar(uniCode, poX+sumX, poY, font)  STM.c:==");
      Serial.println(sumX);
    }
  
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DEBUG ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Serial.print("line 4891 di baseline  drawString sumX += drawChar(uniCode, poX+sumX, poY, font)  STM.c:==");
    Serial.println(baseline);

    Serial.print("line 4894 di cheight drawString sumX += drawChar(uniCode, poX+sumX, poY, font)  STM.c:==");
    Serial.println(cheight);

    Serial.print("line 4897 di textdatum drawString sumX += drawChar(uniCode, poX+sumX, poY, font)  STM.c:==");
    Serial.println(textdatum);
    Serial.print("line 5055 |||||||||||||||||||||||||||||| drawString");}
return sumX;
}



int16_t  drawChar_sumX(uint16_t uniCode, int32_t x, int32_t y, uint8_t font)
{   Serial.print("line 4271  drawchar  ==   TFT_eSPI.cpp::x=");
      Serial.println(x);
      Serial.print("line 4273  drawchar   TFT_eSPI.cpp::y=");
      Serial.println(y);
      Serial.println("line 4275 ################################ INIZIO DI DrawChar TFT_eSPI.cpp");
      Serial.println("line 4276 di TFT_eSPI.cpp");
      Serial.print ("line 4277   di TFT_eSPI.cpp uniCode=");
      Serial.println(uniCode);
      Serial.print ("line 4278   di TFT_eSPI.cpp textsize=");
      Serial.println(textsize);
     Serial.println("line 4295 di chiama  drawChar ");
    drawChar(x, y, uniCode, textcolor, textbgcolor, textsize);
   
    Serial.print("line 4298 di chiama  drawChar TFT_eSPI.cpp");
    
      if((uniCode >= pgm_read_word(&gfxFont->first)) && (uniCode <= pgm_read_word(&gfxFont->last) )) {
        uint16_t   c2    = uniCode - pgm_read_word(&gfxFont->first);
        GFXglyph *glyph = &(((GFXglyph *)pgm_read_dword(&gfxFont->glyph))[c2]);
         Serial.print("line 4331 di chiama  drawChar pgm_read_byte(&glyph->xAdvance)=");
         Serial.print(pgm_read_byte(&glyph->xAdvance));
        return pgm_read_byte(&glyph->xAdvance) * textsize;
      }
      else {
        return 0;
      
}}






void setup() { Serial.begin(115200);
// Without font number, uses font set by setTextFont()
 //drawString("e", 5, 12);
 
  // Setting Pin 2-7 as Output, DDR is PinMode Command, Pin0,1 Untouched
//  DDRD = DDRD | B11111100;
//  // Setting Pin 8-9 as Output
//  DDRB = DDRB | B00000011;
 
pinMode(PA8,OUTPUT);
pinMode(PA9,OUTPUT);
pinMode(PA10,OUTPUT);
pinMode(PB3,OUTPUT);
pinMode(PB4,OUTPUT);
pinMode(PB5,OUTPUT);
pinMode(PB10,OUTPUT);
pinMode(PC7,OUTPUT);
//  //Setting Analog Pins A4-A0 as Output
pinMode(LCD_RD,OUTPUT);
pinMode(LCD_WR,OUTPUT);
pinMode(LCD_RS,OUTPUT);
pinMode(LCD_CS,OUTPUT);
pinMode(LCD_RST,OUTPUT);
  //Setting Analog Pins A4-A0 as HIGH
  //PORTC = PORTC | B00011111; setWriteDirInline();
 
//
Lcd_Init();
//       for (int i = 50; i < 300; i++) {
//        drawPixel(i, 60, RED);
//        drawPixel(i, 70, RED);
//        drawPixel(i, 80, GREEN);
//        drawPixel(i, 90, GREEN);
//      }
//Serial.println("0x2B,BIN=======");
//Serial.println("\n");
//Serial.println(0x2B,BIN);
//LCD_command_write(0b00101011);
//Serial.print("0b11000110,BIN=======***************************:");
//Serial.println("\n");
//Serial.println(0b11000110,BIN);
//LCD_command_write(0b11000110);
//Serial.println(0b11111000,BIN);
//LCD_command_write(0b11111000);
// uint16_t color = selectColour(100, 0, 0);
// drawLine(20,20,300,220,color);
// drawLine(20,220,300,20,color);
//uint16_t color = selectColour(100, 0, 0);
//hollowRectangle(20, 20, 280, 200, color);
//
// for(int i=20; i < 280; i = i +20){
// for(int j=20; j < 200; j = j +20){
// uint16_t color = selectColour(100, 0, 0);
// hollowRectangle(20, 20, i, j, color);
// }
// }

////Example 1
// uint16_t color = selectColour(100, 0, 0);
// filledRectangle(0, 0, 320, 240, color);
//
//filledRectangle(40, 40, 240, 160, selectColour(0, 100, 0));
//
// filledRectangle(80, 80, 160, 80, selectColour(0, 0, 100));
//Example 1
 //uint16_t color = selectColour(100, 100, 100);
 //hollowCircle(160, 120, 120, color);
 //filledRectangle(16,16,108,176,0); 
setFreeFont(&Cherry_Cream_Soda_Regular_40);
const char *string;
string="g";
uint16_t cwidth = textWidth(string, 1);
Serial.println("cwidt");
Serial.println(cwidth);
 //filledRectangle(int16_t rectx1, int16_t recty1, int16_t wide, int16_t tall, uint16_t color)
filledRectangle(5,6,128,20,selectColour(60, 100, 0));
 //drawString("g",16,16);
  //drawString("e",16,16);
  //drawString("A",16,16);
  //drawString("Bd",16,16);
  // drawString("qQ",16,16);
    //drawString("Rs",16,16);
    drawString("VL",16,16);
}

void loop() {//LCD_command_write(0b00101011);
//0x2b);
 //for (int i = 50; i < 300; i++) {
//        drawPixel(i,60,RED);
//        drawPixel(i,70,RED);
//        drawPixel(i,80,BLACK);
       // drawPixel(i,90,BLUE);
         //drawPixel(i,90,MAGENTA);
          //drawPixel(i,90,GREEN);}

}
