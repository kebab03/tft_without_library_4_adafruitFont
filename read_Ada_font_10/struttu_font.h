// We can include all the free fonts and they will only be built into
  // the sketch if they are used
  typedef struct { // Data stored PER GLYPH
  uint32_t bitmapOffset;     // Pointer into GFXfont->bitmap
  uint8_t  width, height;    // Bitmap dimensions in pixels
  uint8_t  xAdvance;         // Distance to advance cursor (x axis)
  int8_t   xOffset, yOffset; // Dist from cursor pos to UL corner
} GFXglyph;



typedef struct { // Data stored for FONT AS A WHOLE:
  uint8_t  *bitmap;      // Glyph bitmaps, concatenated
  GFXglyph *glyph;       // Glyph array
  uint16_t  first, last; // ASCII extents
  uint8_t   yAdvance;    // Newline distance (y axis)
} GFXfont;
  // Call up any user custom fonts
  uint8_t  glyph_ab,   // Smooth font glyph delta Y (height) above baseline
           glyph_bb;   // Smooth font glyph delta Y (height) below baseline
GFXfont  *gfxFont;
