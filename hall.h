#ifndef __HALL_H
#define __HALL_H

unsigned char* hallReadAngle(void);
unsigned char* hallReadMag(void);
unsigned char * hallReadAngleMag(void);
void hallDumpData(unsigned char* buffer);
void hallSetup(void);

void sendbyte(void);
#endif // __HALL_H
