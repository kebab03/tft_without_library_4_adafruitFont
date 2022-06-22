
bool     isDigits=false; 



   uint32_t textcolor, textbgcolor;         // Text foreground and background colours

   
  uint8_t  textfont,  // Current selected font number
           textsize,  // Current font size multiplier
           textdatum, // Text reference datum
           rotation;  // Display rotation (0-3)

  uint8_t  decoderState = 0;   // UTF8 decoder state        - not for user access
  uint16_t decoderBuffer; 
