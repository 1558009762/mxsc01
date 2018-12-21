#ifndef __KEYPAD_H__
#define __KEYPAD_H__

typedef struct _keypad_info
{
    int max_rows;
    int max_cols;
    int StatFilEn;
    int StatFilType;
    int ColFilEn;
    int ColFilType;
    int IoMode;
    int SwapRc;
    int ScanMode;
} iproc_keypad_t;

#endif
