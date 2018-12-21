

#include <stdio.h>
#include <stdlib.h>
 
int main()
{
  char ch, file_name[25];
  unsigned char wbuf[100];
  unsigned char rbuf[100];
  FILE *fp;
 
  printf("Enter file name\n");
  fscanf(stdin, "%s", file_name);
 
  fp = fopen(file_name,"rw"); // read mode
 
  if (fp == NULL) {
    perror("Error while opening the file.\n");
    exit(-1);
  }
 
  /* do a write */

  wbuf[0] = 0x2c;
  wbuf[1] = 0x54;
  wbuf[2] = '\0';

  fwrite(wbuf, 1, sizeof(wbuf) , fp);

  /* do a read */  
  
  fread(rbuf, 1, 2, fp);

  printf("%x  %x\n", rbuf[0], rbuf[1]);

  fclose(fp);
  return 0;
}
