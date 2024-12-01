#include "DataType.h"
#include "MorseCode.h"
#include "gd32f30x.h"
#include "string.h"
#include "LED.h"
#include "WS2812.h"

static u16 Buff[1024];
static u16 BufCnt = 0;
static u16 TotalData = 0;


// 仅支持以下字符
const CodeTable_t CodeTable[]=
{
	{'A',2,{DOT,LINE}},
	{'B',4,{LINE,DOT,DOT,DOT}},
	{'C',4,{LINE,DOT,LINE,DOT}},
	{'D',3,{LINE,DOT,DOT}},
	{'E',1,{DOT}},
	{'F',4,{DOT,DOT,LINE,DOT}},
	{'G',3,{LINE,LINE,DOT}},
	{'H',4,{DOT,DOT,DOT,DOT}},
	{'I',2,{DOT,DOT}},
	{'J',4,{DOT,LINE,LINE,LINE}},
	{'K',3,{LINE,DOT,LINE}},
	{'L',4,{DOT,LINE,DOT,DOT}},
	{'M',2,{LINE,LINE}},
	{'N',2,{LINE,DOT}},
	{'O',3,{LINE,LINE,LINE}},
	{'P',4,{DOT,LINE,LINE,DOT}},
	{'Q',4,{LINE,LINE,DOT,LINE}},
	{'R',3,{DOT,LINE,DOT}},
	{'S',3,{DOT,DOT,DOT}},
	{'T',1,{LINE}},
	{'U',3,{DOT,DOT,LINE}},
	{'V',4,{DOT,DOT,DOT,LINE}},
	{'W',3,{DOT,LINE,LINE}},
	{'X',4,{LINE,DOT,DOT,LINE}},
	{'Y',4,{LINE,DOT,LINE,LINE}},
	{'Z',4,{LINE,LINE,DOT,DOT}},
	{'0',5,{LINE,LINE,LINE,LINE,LINE}},//26
	{'1',5,{DOT,LINE,LINE,LINE,LINE}},
	{'2',5,{DOT,DOT,LINE,LINE,LINE}},
	{'3',5,{DOT,DOT,DOT,LINE,LINE}},
	{'4',5,{DOT,DOT,DOT,LINE,LINE}},
	{'5',5,{DOT,DOT,DOT,DOT,DOT}},
	{'6',5,{LINE,DOT,DOT,DOT,DOT}},
	{'7',5,{LINE,LINE,DOT,DOT,DOT}},
	{'8',5,{LINE,LINE,LINE,DOT,DOT}},
	{'9',5,{LINE,LINE,LINE,LINE,DOT}},
	{' ',1,{DOT}},//36
	// {'.',6,{DOT,LINE,DOT,LINE,DOT,LINE}},//37
	// {',',6,{LINE,LINE,DOT,DOT,LINE,LINE}},
	// {':',6,{LINE,LINE,LINE,DOT,DOT,DOT}},
	// {';',6,{LINE,DOT,LINE,DOT,LINE,DOT}},
	// {'?',6,{DOT,DOT,LINE,LINE,DOT,DOT}},
	// {'=',5,{LINE,DOT,DOT,DOT,LINE}},
	// {'\'',6,{DOT,LINE,LINE,LINE,LINE,DOT}},
	// {'/',5,{LINE,DOT,DOT,LINE,DOT}},
	// {'!',6,{LINE,DOT,LINE,DOT,LINE,LINE}},
	// {'-',6,{LINE,DOT,DOT,DOT,DOT,LINE}},
	// {'_',6,{DOT,DOT,LINE,LINE,DOT,LINE}},
	// {'"',6,{DOT,LINE,DOT,DOT,LINE,DOT}},
	// {'(',5,{LINE,DOT,LINE,LINE,DOT}},
	// {')',6,{LINE,DOT,LINE,LINE,DOT,LINE}},

	// //以下未处理
	// {'$',7,{DOT,DOT,DOT,LINE,DOT,DOT,LINE}},
	// {'&',5,{DOT,DOT,DOT,DOT,DOT}},
	// {'@',5,{LINE,DOT,DOT,DOT,DOT}},

};




void MorseCode_Init(void)
{
	BufCnt = 0;
	TotalData = 0;
	// MorseCodeSend("Jack Trust Me");
	// MorseCodeSend("JACK TRUST ME");
}



void MorseCode_Stop()
{
	BufCnt = 0;
	TotalData = 0;
}



static void MorseCodeSet_High()
{
	LED_ON;//高电平
	WS2812_SetAll(20,20,240);
	RGB_SendToLED();
}


static void MorseCodeSet_Low()
{
	LED_OFF;//低电平
	WS2812_SetAll(0,0,0);
	RGB_SendToLED();
}


void MorseCodeSend(char *Data)
{
	u16 i;
	u16 Length;
	if(TotalData == 0)
	{
		Length = strlen(Data);
		for ( i = 0; i < Length; i++)
		{
			if( ( '0' <= (*Data)) && ((*Data) <= '9') )
			{
				Buff[i] = (u8)(*Data - '0')+26;
			}
			else if( ( 'A' <= (*Data)) && ((*Data) <= 'Z') )
			{
				Buff[i] = (u8)(*Data - 'A');
			}
			else if( ( 'a' <= (*Data)) && ((*Data) <= 'z') )
			{
				Buff[i] = (u8)(*Data - 'a');
			}
			else
			{
				// switch (*Data)
				// {
				// 	case '.':
				// 		Buff[i] = 37;
				// 		break;
				// 	case ',':
				// 		Buff[i] = 38;
				// 		break;
				// 	case ':':
				// 		Buff[i] = 39;
				// 		break;
				// 	case ';':
				// 		Buff[i] = 40;
				// 		break;
				// 	case '?':
				// 		Buff[i] = 41;
				// 		break;
				// 	case '=':
				// 		Buff[i] = 42;
				// 		break;
				// 	case '\'':
				// 		Buff[i] = 43;
				// 		break;
				// 	case '/':
				// 		Buff[i] = 44;
				// 		break;
				// 	case '!':
				// 		Buff[i] = 45;
				// 		break;
				// 	case '-':
				// 		Buff[i] = 46;
				// 		break;
				// 	case '_':
				// 		Buff[i] = 47;
				// 		break;
				// 	case '"':
				// 		Buff[i] = 48;
				// 		break;
				// 	case '(':
				// 		Buff[i] = 49;
				// 		break;
				// 	case ')':
				// 		Buff[i] = 50;
				// 		break;
				// 	case '$':
				// 		Buff[i] = 51;
				// 		break;
				// 	case '&':
				// 		Buff[i] = 52;
				// 		break;
				// 	case '@':
				// 		Buff[i] = 53;
				// 		break;
				// 	default:
				// 		Buff[i] = 36;//未识别的均按空格处理
				// 		break;
				// }
						Buff[i] = 36;//未识别的均按空格处理

			}
			Data++;
		}
		BufCnt = 0;
		TotalData = Length;
	}
}


// 10ms调用一次
void MorseCodeTimerTick()
{
	static  u16 TimeCnt = 0;
	static  u8 MCoffset = 0;

	if(TotalData == 0)
	{
		return;
	}

	if(TimeCnt < CodeTable[Buff[BufCnt]].MorseCode[MCoffset])
	{
		if(CodeTable[Buff[BufCnt]].Letter == ' ')
		{
			MorseCodeSet_Low();
		}
		else
		{
			MorseCodeSet_High();
		}
	}
	else
	{

		if( MCoffset < (CodeTable[Buff[BufCnt]].MorseCodeLen -1) )
		{
			if(TimeCnt < (CodeTable[Buff[BufCnt]].MorseCode[MCoffset] + DOT - 1))
			{
				MorseCodeSet_Low();
			}
			else
			{
				TimeCnt = 0xFFFF;
				MCoffset++;
				if(MCoffset >= CodeTable[Buff[BufCnt]].MorseCodeLen)
				{
					BufCnt++;
					MCoffset = 0;
					if( (BufCnt >= sizeof(Buff)) || (BufCnt >=TotalData))
					{
						BufCnt = 0;
						TotalData = 0;
					}
				}
			}
		}
		else if( MCoffset == (CodeTable[Buff[BufCnt]].MorseCodeLen -1) )
		{
			if(TimeCnt < (CodeTable[Buff[BufCnt]].MorseCode[MCoffset] + 3*DOT - 1))
			{
				MorseCodeSet_Low();
			}
			else
			{
				TimeCnt = 0xFFFF;
				MCoffset++;
				if(MCoffset >= CodeTable[Buff[BufCnt]].MorseCodeLen)
				{
					BufCnt++;
					MCoffset = 0;
					if( (BufCnt >= sizeof(Buff)) || (BufCnt >=TotalData))
					{
						BufCnt = 0;
						TotalData = 0;
					}
				}
			}
		}
	}
	TimeCnt++;
}













































