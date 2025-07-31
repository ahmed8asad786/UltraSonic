#ifndef TEXT_H
#define TEXT_H

// Draw a single character using 5x7 font
void draw_char_5x7(char c, int x, int y);
// Draw a string using 5x7 font
void draw_string_5x7(const char *text, int x, int y);

// Draw a single character using 7x9 font
void draw_char_7x9(char c, int x, int y);
// Draw a string using 7x9 font
void draw_string_7x9(const char *text, int x, int y);


void draw_char_8x12(char c, int x, int y);

void draw_string_8x12(const char *text, int x, int y);



#endif
