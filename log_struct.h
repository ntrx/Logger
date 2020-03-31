#ifndef LOG_STRUCT_H
#define LOG_STRUCT_H
#define LOG_ID 10601
#pragma pack(push, 1)
struct TRecordData {
 //INT32U Status;   		//режим навигационного решения
//unsigned int size; // размер данного массива
	double T;				//время навигационного решения
	double Fi;				//широта
	double La;				//долгота
	float H;				//высота
	float RMS;			//точность на плоскости
 //FP32 RMSH;			//точность по высоте
 //FP32 VFi;			//скорость по широте
 //FP32 VLa;			//скорость по долготе
 //FP32 VH;			//скорость по высоте
 //INT32S Activate;		//признак активации маршрута
 //INT8U GS[8];			//скорость относительно земли
 //INT8U TRK[8];			//текущий путевой угол
 //INT8U DTK[8];			//заданный путевой угол
 //INT8U BRG[8];			//азимут на точку
 //INT8U DIS[8];			//расстояние до конца активного участка
 //INT8U ATD[8];			//расстояние до конца маршрута
 //INT8U XTK[10];			//отклонение от маршрута
 //INT8U ETE[6];			//время полета до точки
 //INT8U ETA[6];			//время прибытия в точку
 //INT8U TKE[8];			//отклонение от заданного курса
 //FP32 Offset;			//смещение от маршрута
 //char toPoint[8];			//целевая точка участка маршрута
};
//
//
struct TBlockData {
	unsigned int ID;           		//идентификатор
	unsigned int Number;       	//номер записи
	unsigned int NumberWkl;     	//номер включения
	unsigned int LenData;      	//размер данной структуры
 //INT32U LenRecord;      	//размер структуры TRecordData
 	char current_date[12];     		//тек дата 2019-06-21
 	char current_time[10];	// тек время 00:00:00
 //INT32S Time; 			//время
 //INT32U VersiaCP;		//версия ПО
 //INT8U Decl[5];			//магнитное склонение
	struct TRecordData DataComplex;	//навигационные данные, (компл гнсс вор дме ins рсбн)
	struct TRecordData DataGNSS;
	struct TRecordData DataSWS;
	struct TRecordData DataDMEDME;
	struct TRecordData DataINS;
	struct TRecordData DataVORDME;
	char dtk_to[8]; //WindNav.dtk_to;
	char tk[8]; //WindNav.tk;
    unsigned int CS;			//контрольная сумма
};
extern struct TBlockData BlockData1;
#pragma pack(pop)
#endif