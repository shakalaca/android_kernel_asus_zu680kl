#ifndef UPDATE_INTERFACE_H
#define UPDATE_INTERFACE_H
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/delay.h>
#include "anx_ohio_update.h"

extern unsigned char  *OCM_CODE;
static unsigned char  InactiveWord[9] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // ECC is set to 0xFF
static unsigned char  BlankWord[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned char  HammingTable[] =
{// 0x01  0x02  0x04  0x08  0x10  0x20  0x40  0x80
    0x0b, 0x3b, 0x37, 0x07, 0x19, 0x29, 0x49, 0x89, // 0
    0x16, 0x26, 0x46, 0x86, 0x13, 0x23, 0x43, 0x83, // 1
    0x1c, 0x2c, 0x4c, 0x8c, 0x15, 0x25, 0x45, 0x85, // 2
    0x1a, 0x2a, 0x4a, 0x8a, 0x0d, 0xcd, 0xce, 0x0e, // 3
    0x70, 0x73, 0xb3, 0xb0, 0x51, 0x52, 0x54, 0x58, // 4
    0xa1, 0xa2, 0xa4, 0xa8, 0x31, 0x32, 0x34, 0x38, // 5
    0xc1, 0xc2, 0xc4, 0xc8, 0x61, 0x62, 0x64, 0x68, // 6
    0x91, 0x92, 0x94, 0x98, 0xe0, 0xec, 0xdc, 0xd0  // 7
};
       
        
void TRACE_ARRAY(const char *buf, size_t size)
{
	while (size--)
			printk("%0x ", *buf++);
		printk("\n");
	
}
 
void WriteWordReg(unsigned char RegAddr, unsigned int RegVal)
{
	unsigned char pBuf[2];

	pBuf[0] = HIBYTE(RegVal);
	pBuf[1] = LOBYTE(RegVal);
	OhioWriteBlockReg(RegAddr, 2, pBuf);
}
// OTP operation driver
// initialize OTP operation
void OTP_Init(void)
{
    ohio_chip_enable(1);
    OhioWriteReg(0x0D, 0x02); // power down ocm
    OhioWriteReg(0xE5, 0xA8); // disable ocm access otp & wake up otp, bypass ECC
    OhioWriteReg(0xEF, 0x7A); // OTP access key
}

// uninitialize OTP operation
void OTP_Uninit(void)
{
    OhioWriteReg(0xEF, 0x00); // clear OTP access key
    ohio_chip_enable(0);
}

// read OTP with ECC inactive
void OTP_Read(unsigned int WordAddr, unsigned char  *pBuf)
{
    OhioWriteReg(0xE5, 0xA8); // disable ocm access otp & wake up otp, bypass ECC
    WriteWordReg(0xD0, WordAddr); // write otp word address
    OhioWriteReg(0xE5, 0xA9); // start read OTP
    while((OhioReadReg(0xED) & 0x30) != 0); // wait for read done
    OhioReadBlockReg(0xDC, 9, pBuf); // copy to destination buffer
}

// read OTP with ECC active
void OTP_ReadEx(unsigned int WordAddr, unsigned char  *pBuf)
{
    OhioWriteReg(0xE5, 0xA0); // disable ocm access otp & wake up otp, activate ECC
    WriteWordReg(0xD0, WordAddr); // write otp word address
    OhioWriteReg(0xE5, 0xA1); // start read OTP
    while((OhioReadReg(0xED) & 0x30) != 0); // wait for read done
    OhioReadBlockReg(0xDC, 9, pBuf); // copy to destination buffer
}

// program and verify
unsigned char OTP_Program(unsigned int WordAddr, unsigned char *pBuf)
{
    unsigned char  buf[9];
    unsigned char i;
    
    WriteWordReg(0xD0, WordAddr); // write otp word address
    OhioWriteBlockReg(0xD2, 8, pBuf); // data
    OhioWriteReg(0xDB, pBuf[8]); // ECC
    
    for(i = OTP_PROGRAM_MAX; i; i--)
    {
      //  TRACE(".");
        OhioWriteReg(0xE5, 0xAA); // start program OTP
        while((OhioReadReg(0xED) & 0x0F) != 0); // wait for program done
        OTP_Read(WordAddr, buf); // read word WordAddr
        if(memcmp(pBuf, buf, 9) == 0)
        {// ok
         //   TRACE("\n");
            return 1;
        }
    }
   // TRACE("\nTry to activate ECC\n");
    OTP_ReadEx(WordAddr, buf); // read word WordAddr
    if(memcmp(pBuf, buf, 9) == 0)
    {// ok
        return 1;
    }
    else
    {// fail
        TRACE("Read back: ");
        TRACE_ARRAY(buf, 9);
        TRACE("Expecting: ");
        TRACE_ARRAY(pBuf, 9);
        return 0;
    }
}

// verify
char OTP_Verify(unsigned char *pSrc, unsigned int WordAddr)
{
	unsigned char  DataBuf[9];
	unsigned char  DataBufEx[9];

	OTP_Read(WordAddr, DataBuf);
	TRACE1("[%x]   : ", WordAddr);
	if (memcmp(pSrc, DataBuf, 9) == 0) // make sure word 0 is blank
	{// pass
		TRACE("PASS\n");
		return 0;
	}
	else
	{
		OTP_ReadEx(WordAddr, DataBufEx);
		if (memcmp(pSrc, DataBufEx, 9) == 0)
		{// warning
			TRACE("WARNING\n");
			TRACE("ECC OFF  : ");
			TRACE_ARRAY(DataBuf, 9);
			TRACE("Expecting: ");
			TRACE_ARRAY(pSrc, 9);
			return 1;
		}
		else
		{// fail
			TRACE("FAILURE\n");
			TRACE("ECC OFF  : ");
			TRACE_ARRAY(DataBuf, 9);
			TRACE("ECC ON   : ");
			TRACE_ARRAY(DataBufEx, 9);
			TRACE("Expecting: ");
			TRACE_ARRAY(pSrc, 9);
			return 2;
		}
	}
}

unsigned char HammingEncoder(unsigned char *pData)
{
    unsigned char i, j;
    unsigned char result;
    unsigned char c;

    for(result = 0, i = 0; i < 8; i++)
    {
        c = *pData;
        for(j = 0; j < 8; j++)
        {
            if(c & 0x1)
            {
                result ^= HammingTable[(i << 3) + j]; // i * 8 + j
            }
            c >>= 1;
        }
        pData++;
    }
    return result;
}

/**
 * @desc:  update ohio ocm firmware
 *
 *
 * @return:  0: success.  1: fail
 *
 */
u8 update_otp(void)
{
	unsigned char DataBuf[9];
	unsigned char i = 0;
	unsigned char FWHeaderAddr = 0; // in otp word
	unsigned int NewFWStartAddr = 0; // in otp word
	unsigned int ProgAddr = 0;
	unsigned int OldFWStartAddr = 0;
	unsigned int OldFWSize = 0;
	unsigned char count; // program times
	unsigned char val;
	unsigned char  *pOCM;
	unsigned int NewFWSize = 0;
	unsigned int NewFWSize_in = 0;

	OTP_Init(); // init OTP operation
	// word 0
	OTP_ReadEx(0, DataBuf); // read word WordAddr
	if (memcmp(BlankWord, DataBuf, 9) != 0) // make sure word 0 is blank
	{// fail
		TRACE("[0000]   : ");
		TRACE_ARRAY(DataBuf, 9);
		TRACE("Expecting: ");
		TRACE_ARRAY(BlankWord, 9);
		goto uo_exit;
	}

	// word 1
	OTP_ReadEx(1, DataBuf); // read word 1
	// check word 1 status
	if (memcmp(BlankWord, DataBuf, 9) == 0)
	{// word 1 is blank word
		// update word 1
		TRACE("Initializing OTP header\n");
		for(count = 0;count<6;count++)
			{
			val = OTP_Program(1, OCM_CODE + 9);
			}
		if (!val)
		{// fail
			goto uo_exit;
		}
		// ok
		FWHeaderAddr = 2; // init fw header address
		NewFWStartAddr = 2 + OTP_UPDATE_MAX; // init new fw start address
	}
	else
	{// word 1 is expecting


		// word 2 to i
		// search the old fw start address
		i = 1;
		do
		{
			if (++i >= 2 + OTP_UPDATE_MAX - 1)
			{
				TRACE("NO free space for FW header!\n");
				goto uo_exit;
			}
			OTP_ReadEx(i, DataBuf); // read word i
		} while (memcmp(InactiveWord, DataBuf, 9) == 0);
		BYTE1(OldFWStartAddr) = DataBuf[0]; // convert little-endian to big-endian
		BYTE0(OldFWStartAddr) = DataBuf[1];
		BYTE1(OldFWSize) = DataBuf[2]; // convert little-endian to big-endian
		BYTE0(OldFWSize) = DataBuf[3];
		// print old fw information
		TRACE2("OldFWStartAddr=%04X,OldFWSize=%04X\n", OldFWStartAddr, OldFWSize);
              if(OldFWStartAddr<0x12)
			{
				TRACE("wrong start address!\n");
				goto uo_exit;
			}
		FWHeaderAddr = i; // init fw header address
		NewFWStartAddr = OldFWStartAddr + OldFWSize; // init new fw start address = old fw start address + old fw size
              if(NewFWStartAddr+NewFWSize > OTP_SIZE)
			{
				TRACE2("[%04x][%04x]NO free space for FW code!\n", NewFWStartAddr, NewFWSize);
				goto uo_exit;
			}
	}

	// program fw code
	BYTE1(NewFWSize) = OCM_CODE[2 * 9 + 2]; // init OTP programe size
	BYTE0(NewFWSize) = OCM_CODE[2 * 9 + 2 + 1]; // convert little-endian to big-endian
	TRACE2("NewFWStartAddr=%04X,NewFWSize=%04X\n", NewFWStartAddr, NewFWSize);
	pOCM = OCM_CODE + 3 * 9; // init fw code pointer
	ProgAddr = NewFWStartAddr; // init OTP program address
	TRACE("Programming FW\n");
	for (count = 0; count < 6; count++)
	{
		TRACE1("Programming FW times=%d\n\n", count + 1);
		pOCM = OCM_CODE + 3 * 9; // init fw code pointer
		ProgAddr = NewFWStartAddr; // init OTP program address
		NewFWSize_in = NewFWSize;
		do
		{
			if (!OTP_Program(ProgAddr++, pOCM))
			{// fail
				if (count == 5)
				{
					TRACE("Programming Fail,please re-program\n");

					goto uo_exit;
				}
			}
			pOCM += 9;
		} while (--NewFWSize_in != 0);
	}


	// update new fw header
	OTP_ReadEx(FWHeaderAddr, DataBuf); // read fw header
	if (memcmp(BlankWord, DataBuf, 9) != 0) // make sure fw header is blank
	{// Programmed
		// make old fw header inactive
		TRACE("Erasing old FW header\n");
		for (count = 0; count < 6; count++)   //wensen
			val = OTP_Program(FWHeaderAddr, InactiveWord);
		if (!val)

		{// fail
			goto uo_exit;
		}
		// ok
		FWHeaderAddr++; // use next word as new fw header
	}
	{
		unsigned int RealFWSize = 0;

		TRACE("Programming FW header\n");
		memcpy(DataBuf, OCM_CODE + 2 * 9, 9);
		DataBuf[0] = LOBYTE(NewFWStartAddr); // convert big-endian to little-endian
		DataBuf[1] = HIBYTE(NewFWStartAddr);
		RealFWSize = ProgAddr - NewFWStartAddr;
		DataBuf[2] = LOBYTE(RealFWSize);  // convert big-endian to little-endian
		DataBuf[3] = HIBYTE(RealFWSize);
		DataBuf[8] = HammingEncoder(DataBuf);
		for (count = 0; count < 6; count++)	//wensen
			val = OTP_Program(FWHeaderAddr, DataBuf);
		while (!val)
		{// fail
			// make the word inactive
			for (count = 0; count < 6; count++)
				val = OTP_Program(FWHeaderAddr, InactiveWord);
			if (!val)
			{// fail
				goto uo_exit;
			}
			FWHeaderAddr++;
			if (FWHeaderAddr == 2 + OTP_UPDATE_MAX)
			{
				TRACE("NO free space for FW header!\n");
				goto uo_exit;
			}
		}
	}
	TRACE("update OTP success!\n");
	return 0;
uo_exit:
	OTP_Uninit();
	TRACE("update OTP fail!\n\n");
	return 1;
}


unsigned char getOtpVer(void)
{
       unsigned char bver = 0;
	OTP_Init(); // init OTP operation
	bver = OhioReadReg(0x44);
	OTP_Uninit();

	return bver;
}

/**
 * @desc:   verify ohio ocm code with OCM_CODE[]
 *
 *
 * @return:  0: success.  1: fail
 *
 */
u8 verify_otp(void)
{
	unsigned char DataBuf[9];
	unsigned int FailureCnt;
	unsigned int WarningCnt = 0;
	unsigned char *pOCM = NULL;
	unsigned int FWSize = 0;
	char RetVal = 0;
	unsigned int ProgAddr = 0;

	OTP_Init(); // init OTP operation

	FailureCnt = 0, WarningCnt = 0;
	// word 0
	RetVal = OTP_Verify(OCM_CODE, 0);
	if (RetVal == 1)
	{
		WarningCnt++;
	}
	else if (RetVal == 2)
	{
		FailureCnt++;
	}
	// word 1
	/*RetVal = OTP_Verify(OCM_CODE + 9, 1);
	if(RetVal == -1)
	{
	WarningCnt++;
	}
	else if(RetVal == -2)
	{
	FailureCnt++;
	}*/
	// search fw header
	ProgAddr = 2;
	do
	{
		OTP_ReadEx(ProgAddr++, DataBuf); // read word i
	} while (memcmp(InactiveWord, DataBuf, 9) == 0);
	BYTE1(FWSize) = OCM_CODE[2 * 9 + 2]; // convert little-endian to big-endian
	BYTE0(FWSize) = OCM_CODE[2 * 9 + 2 + 1];
	BYTE1(ProgAddr) = DataBuf[0]; // init fw start address
	BYTE0(ProgAddr) = DataBuf[1]; // convert little-endian to big-endian
	pOCM = OCM_CODE + 3 * 9; // init fw code pointer
	do
	{
		RetVal = OTP_Verify(pOCM, ProgAddr++);
		if (RetVal == 1)
		{
			WarningCnt++;
		}
		else if (RetVal == 2)
		{
			FailureCnt++;
		}
		pOCM += 9;
	} while (--FWSize != 0);
	BYTE1(FWSize) = OCM_CODE[2 * 9 + 2]; // restore fw size
	BYTE0(FWSize) = OCM_CODE[2 * 9 + 2 + 1];
	TRACE3("%u Word(s) Verified %u Failure(s) %u Warning(s)\n", FWSize + 2, FailureCnt, WarningCnt);
	OTP_Uninit();
       TRACE("verify OTP finished!\n");
	return 0;
}

// \dump
void dump_otp(void)
{
	unsigned char  buf[16];
	int i = 0;

	OTP_Init(); // init OTP operation
	TRACE("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

	i = 0;
	do
	{
		OhioReadBlockReg(i, 16, buf);
		TRACE1("[%02x]:", i);
		TRACE_ARRAY(buf, 16);
		i += 16;
	} while (i < 0xF0);
	OTP_Uninit();
}

#endif

