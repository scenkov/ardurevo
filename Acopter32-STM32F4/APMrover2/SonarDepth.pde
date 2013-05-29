
//#include <string.h>
//#include <stdio.h>

void read_Sonar() {

 while (hal.uartC->available()) {
  if (read_tokens(hal.uartC->read(), &Sonar_tokens)) {
   if (!strcmp(Sonar_tokens.token[1],"$SDDPT")) Depth = atof(Sonar_tokens.token[2]);
   if (!strcmp(Sonar_tokens.token[1],"$SDMTW")) Temp = atof(Sonar_tokens.token[2]);
   
   sonar_serial_timer = hal.scheduler->millis(); //timer to detect bad serial or no serial connected
   
  }
  
  if ( hal.scheduler->millis()-sonar_serial_timer > 4000 ) { // check if serial stream is not present for 4 sec
    
    Depth = 999; // debug value that means no serial or bad serial for more than 4 sec
    Temp = 999;  // debug value that means no serial or bad serial for more than 4 sec
    
    }
 }
/*
hal.console->println();
hal.console->printf("--------------------------------------");
hal.console->printf("Sonar Depth : %f Temp: %f",Depth,Temp);        //Debug Echo Data
hal.console->println("--------------------------------------");
*/
}

int read_tokens (char character, struct tokens *buffer) {
 char *p, *token;
 uint8_t i;
 if (character == '\r') {
  buffer->array[buffer->char_index] = 0;
  buffer->char_index = 0;
  if (!checksum(buffer->array)) return 0;
  p = buffer->array;
  i = 0;
  while ((token = strtok_r(p, ",", &p))){
      buffer->token[++i] = token;
  }
  return i;
 }
 if (character == '\n') {
  buffer->char_index = 0;
  return 0;
 }
 buffer->array[buffer->char_index] = character;
 if (buffer->char_index < DIM2) (buffer->char_index)++;
 return 0;
}

int checksum(char *str) {
 int j, len, xor1, xor2;
 len = strlen(str);
 if (str[0] != '$') return 0;
 if (str[len-3] != '*') return 0;
 xor1 = 16*hex2int(str[len-2]) + hex2int(str[len-1]);
 xor2 = 0;
 for (j=1;j<len-3;j++) xor2 = xor2 ^ str[j];
 if (xor1 != xor2) return 0;
 return 1;
}

int hex2int(char a) {
 if (a>='A' && a<='F') return 10+(a-'A');
 if (a>='a' && a<='f') return 10+(a-'a');
 if (a>='0' && a<='9') return a-'0';
 return 0;
}

float fixDM(float DM) {
 int D; float F;
 F = DM /100;
 D = int (F);
 return D + (F - D)/0.6;
}

