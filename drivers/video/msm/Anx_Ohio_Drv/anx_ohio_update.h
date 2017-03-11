
#ifndef OTP_H
#define OTP_H
#include <linux/types.h>

#define OTP_PROGRAM_MAX         1
#define OTP_UPDATE_MAX            16
#define OTP_SIZE                          (unsigned int)(0x20000 / 8) // in otp word
#define OTP_BUF_MAX_SIZE ( 3 * 9 + 32768 + 32768 / 8)
#define OTP_BIN_VERSION_INDEX 0x121b
#define BYTE0(x)    ((unsigned char *)&x)[1]
#define BYTE1(x)    ((unsigned char *)&x)[0]
#define LOBYTE(x)   ((unsigned char)(x))
#define HIBYTE(x)   ((unsigned char)((x) >> 8))

#define TRACE(format)   \
        pr_info(format)
#define TRACE0(format)  \
        pr_info(format)
#define TRACE1(format, arg1)    \
        pr_info(format, arg1)
#define TRACE2(format, arg1, arg2)  \
        pr_info(format, arg1, arg2)
#define TRACE3(format, arg1, arg2, arg3)    \
        pr_info(format, arg1, arg2, arg3)

void ohio_chip_enable(bool value);
unsigned char OhioReadReg(unsigned char RegAddr);
void OhioWriteReg(unsigned char RegAddr, unsigned char RegVal);
int OhioReadBlockReg(u8 RegAddr, u8 len, u8 *dat);
int OhioWriteBlockReg(u8 RegAddr, u8 len, const u8 *dat);        
u8 update_otp(void);
u8 verify_otp(void);
void dump_otp(void);
unsigned char getOtpVer(void);

#endif  // end of OTP_H definition

