/*
	Модуль записи GNSS данных у файл 
	Версия 2.1, от 22.01.2020 11:48
*/

#include "log_struct.h" // <- файл структуры записи 
#include "sys_wnds.h"
#include "fs_utils.h"
#include "rtems.h"
#include "NavParam.h"
#include "Comzdakp.h"
#include "NumForm.h"
#include "Key_cod.h"

#ifndef _WINDOWS
#include <errno.h>
/*
	Пути к сохранению файлов.
	log1.dat - текущий файл, записуемый
	log2.dat - ожидающий записи файл
*/
#define StoreLogDir      "../files/"
char log_file1[] = "../files/log1.dat"; // logging file 1
char log_file2[] = "../files/log2.dat"; // logging file 2
char log_file_work[] = "../files/log1.dat"; // working file
char log_file_wait[] = "../files/log2.dat"; // secondary file
#else
#define StoreLogDir      "../../../Build/files/"
char log_file1[] = "../../../Build/files/log1.dat"; // logging file 1
char log_file2[] = "../../../Build/files/log2.dat"; // logging file 2
char log_file_work[] = "../../../Build/files/log1.dat"; // working file
char log_file_wait[] = "../../../Build/files/log2.dat"; // secondary file
#endif

#define STR_SIZE 50 // максимальный размер заголовочной информации в строке
#define ELEMENTS 45 // максимальное количнство типов данных 
#define BLOCK_SZ 16 // размер блока который описывает размер заголовочной информации

char data_logger[ELEMENTS][STR_SIZE]; // заголовочная структура


struct TBlockData BlockData1;
struct TBlockData TBData;


unsigned int tmp_sum=0;
unsigned int last_element=0; // текущий/последний номер елемента в заголовке
unsigned int current_record; // номер текущей записи в лог
unsigned int launches; // количество запусков устройства
unsigned int first_rec_f1; // 
unsigned int first_rec_f2;
unsigned int last_record_f1; // последняя запись в файле №1 
unsigned int last_record_f2; // последняя запись в файле №2
unsigned int last_record;    // последняя запись в файлах
unsigned int record_total;  // количество записей
unsigned long long file_size1; // размер файла №1
unsigned long long file_size2; // размер файла №2
unsigned long long file_size_work; // размер файла, рабочего
int log_file_size; 
int file_size_max;
unsigned int state = 0; // режим устройства: 0 - при запуске, 1 - в режиме работе
unsigned int state_m = 0; // не используется

extern INT32S KVU_Time, KVU_Week, TimeZone; // время
extern char USB_Flight_Folder[];
extern int ReadFileInBuf(char* fName, INT8U* buff[], unsigned &len);
extern int Expotr_Error[50];
char Save_Name[200];
extern char FPL_Export_Dir[60];
char exp_status[150];

/* list of procedures */
void debug_get_date(char);
void debug_get_time(char);
void logger_start(void);
void swap_files();
void logger_get_record();
void write_data_pre();
void write_date_struct();
unsigned int get_size(char data_type[]);
void add_element(char data_type[], char name[]);
void add_element_char(char data_type[], char name[],unsigned int symbols);
void log_export();


#define FILE_DEBUG 1 // 1-включить/2-выключить запись в файл
#define FILE_WRITE 12 // максимальный размер в часах
#define STRUCT_SIZE sizeof(TBData) // размер структуры
#define FS_SYSTEM INT_MAX-STRUCT_SIZE // 2gb
#define FS_TIME (((FILE_WRITE*60)*60)*STRUCT_SIZE) //максимальный размер файла в байтах в расчете на заданое время
#define FILE_SIZE ((FS_TIME < FS_SYSTEM) ? FS_TIME : FS_SYSTEM)  // решение между макс временем и 2ГБ




// Функция даты
void debug_get_date(char *Mas) {
	INT32S Day,Month,Year;
	INT32S Time=KVU_Time+TimeZone;
	INT32S Week=KVU_Week;
	if ( Time >= 604800) {
		Time -= 604800;
		Week++;
	}
	if ( Time < 0 ) {
		Time += 604800;
		Week--;
	}
	cnvDATE(Week,Time,&Day,&Month,&Year);
	sprintf(Mas,"%04d-%02d-%02d",Year,Month,Day);
}
// Функция времени
void debug_get_time(char* Mas) {
	INT32S Hour,Min,Sec;
	INT32S Time=KVU_Time+TimeZone;
	INT32S Week=KVU_Week;
	if ( Time >= 604800 ) {
		Time -= 604800;
		Week++;
	}
	if ( Time < 0 ) {
		Time+=604800;
		Week--;
	}
	cnvTIME(Time,&Hour,&Min,&Sec);
	sprintf(Mas,"%02d:%02d:%02d\t",Hour,Min,Sec);
}

// compare last record and create work log file
// Функция нахождения последней записи в двух файлах и создание текущего рабочего лог-файла
void logger_get_record() {
	// version 2.1 (22.01.2020 11:48)
	FILE *fp1, *fp2;

	// Проверка на существование файла №1
	if (!fsExists(log_file1)) {
		// Если нету то создание его и обнуление показателей
		fp1 = fopen(log_file1,"ab");
		last_record_f1 = 0;
		launches = 0;
		first_rec_f1 = 0;
	} else {
		// Если есть, считывание с последней записи кол-ва записей в файле и
		// последнее кол-во запусков устройства, и размер файла
		fp1 = fopen(log_file1,"rb");
		fread(&TBData, sizeof(TBData), 1, fp1);
		first_rec_f1 = TBData.Number; // for amount of records
		// file size
		fseek(fp1, -sizeof(TBData), SEEK_END);
		file_size1 = ftell(fp1) + sizeof(TBData);
		// read last struct for last record
		fread(&TBData, sizeof(TBData), 1, fp1);
		last_record_f1 = TBData.Number;
		launches = TBData.NumberWkl+1;
		printf("Logger: found launches in %s: %d\n", log_file1, launches);
	}
	fclose(fp1);
	
	// Проверка совместимости между считанным файлом и текущим вариантом записи
	// Если структура данных отличается от текущего, то начинать новую запись
	if (TBData.LenData != sizeof(TBData)) { 
		fp1=fopen(log_file1,"w"); fclose(fp1);
		first_rec_f1 = 0;
		last_record_f1 = 0;
		launches = 0;
	}
	// Проверка и анализ файла №2 аналогично файлу №1
	if (!fsExists(log_file2)) {
		fp2 = fopen(log_file2,"ab");
		first_rec_f2 = 0;
		last_record_f2 = 0;
		launches = 0;
	} else {
		fp2 = fopen(log_file2,"rb");
		fread(&TBData, sizeof(TBData), 1, fp2);
		first_rec_f2 = TBData.Number;
		// file size
		fseek(fp2, -sizeof(TBData), SEEK_END);
		file_size2 = ftell(fp2) + sizeof(TBData);
		// read last struct for last record
		fread(&TBData, sizeof(TBData), 1, fp2);
		last_record_f2 = TBData.Number;
		launches = TBData.NumberWkl+1 > launches ? TBData.NumberWkl+1 : launches;	
		printf("Logger: found launches in %s: %d\n", log_file2, TBData.NumberWkl+1);
	}
	fclose (fp2);
	// Проверка совместимости аналогично файлу №1
	if (TBData.LenData != sizeof(TBData)) { // if sizes of structures diff, create new log 
		fp2=fopen(log_file2,"w"); fclose(fp2);
		first_rec_f2 = 0;
		last_record_f2 = 0;
		launches = 0;
	}
	
	// Вычисление общего количества записей
	record_total = last_record_f1-first_rec_f1;
	record_total+= last_record_f2-first_rec_f2;
	// Вычисление рабочего файла
	if (last_record_f1 == last_record_f2) {
		strcpy(log_file_work,log_file1); // working file22,180,70,16 
		strcpy(log_file_wait,log_file2); // waiter 
		file_size_work = file_size1;
		last_record = last_record_f1;
		current_record = last_record;
	}
	if (last_record_f2 > last_record_f1) {
		strcpy(log_file_work,log_file2);
		strcpy(log_file_wait,log_file1);
		file_size_work = file_size2;
		last_record = last_record_f2;
		current_record = last_record;
	} else {
		strcpy(log_file_work,log_file1);
		strcpy(log_file_wait,log_file2);
		file_size_work = file_size1;
		last_record = last_record_f1;
		current_record = last_record;
	}
}
// Функция запуска лог записи
void logger_start(void) {
	if (FILE_DEBUG == 1) {
		printf("Logger: max log file size: %d\n", FILE_SIZE);
		launches = 1;
		state = 0; // switch to starting stage
		logger_get_record();
		write_data_pre();
	}
}
// Функция смены файлов
void swap_files() {
	if ( strcmp(log_file_work,log_file1) == 0) {
		strcpy(log_file_work, log_file2);
		strcpy(log_file_wait, log_file1);
		FILE *fp = fopen(log_file2,"w"); fclose(fp);
		return;
	}
	if ( strcmp(log_file_work, log_file2) == 0) {
		strcpy(log_file_work, log_file1);
		strcpy(log_file_wait, log_file2);
		FILE *fp = fopen(log_file1,"w"); fclose(fp);
		return;
	}
}


// Запись данных в память
void write_data_struct() {
	
	BlockData1.ID = LOG_ID;
	BlockData1.Number = ++current_record; record_total++; 
	BlockData1.NumberWkl = launches;
	BlockData1.LenData = sizeof(BlockData1);

	char cur_d[12];
	debug_get_date(cur_d);
	memcpy(BlockData1.current_date,cur_d,sizeof(cur_d));

	char cur_t[10];
	debug_get_time(cur_t);
	memcpy(BlockData1.current_time,cur_t,sizeof(cur_t));


	TmCoordinates Coor;
	Coordinates->ReadData(&Coor);
	BlockData1.DataComplex.Fi = Coor.CRD.Fi; //P1004.Fi; 
	BlockData1.DataComplex.La = Coor.CRD.La; //P1004.La;
	BlockData1.DataComplex.H  = Coor.CRD.H; //P1004.H;
	BlockData1.DataComplex.RMS = Coor.CRD.RMS; //P1004.SKO;
	BlockData1.DataComplex.T  = Coor.T; //P1004.T;	

	BlockData1.DataGNSS.Fi = P1088.Fi; //NavData[ID_GNSS1].Dt.Fi;
	BlockData1.DataGNSS.La = P1088.La;// NavData[ID_GNSS1].Dt.La;
	BlockData1.DataGNSS.H = P1088.H;//NavData[ID_GNSS1].Dt.H;
	BlockData1.DataGNSS.RMS = P1088.SKO; //NavData[ID_GNSS1].Dt.SKO_Fi+NavData[ID_GNSS1].Dt.SKO_La+NavData[ID_GNSS1].Dt.SKO_H;
	BlockData1.DataGNSS.T = P1088.T;//NavData[ID_GNSS1].Dt.T;

	BlockData1.DataSWS.Fi = NavData[ID_SWS1].Dt.Fi;
	BlockData1.DataSWS.La = NavData[ID_SWS1].Dt.La;
	BlockData1.DataSWS.H = NavData[ID_SWS1].Dt.H;
	BlockData1.DataSWS.RMS = NavData[ID_SWS1].Dt.SKO_Fi+NavData[ID_SWS1].Dt.SKO_La+NavData[ID_SWS1].Dt.SKO_H;
	BlockData1.DataSWS.T = NavData[ID_SWS1].Dt.T;

	BlockData1.DataDMEDME.Fi = NavData[ID_DME_DME1].Dt.Fi;
	BlockData1.DataDMEDME.La = NavData[ID_DME_DME1].Dt.La;
	BlockData1.DataDMEDME.H = NavData[ID_DME_DME1].Dt.H;
	BlockData1.DataDMEDME.RMS = NavData[ID_DME_DME1].Dt.SKO_Fi+NavData[ID_DME_DME1].Dt.SKO_La+NavData[ID_DME_DME1].Dt.SKO_H;
	BlockData1.DataDMEDME.T = NavData[ID_DME_DME1].Dt.T;

	BlockData1.DataINS.Fi = NavData[ID_INS1].Dt.Fi;
	BlockData1.DataINS.La = NavData[ID_INS1].Dt.La;
	BlockData1.DataINS.H = NavData[ID_INS1].Dt.H;
	BlockData1.DataINS.RMS = NavData[ID_INS1].Dt.SKO_Fi+NavData[ID_INS1].Dt.SKO_La+NavData[ID_INS1].Dt.SKO_H;
	BlockData1.DataINS.T = NavData[ID_INS1].Dt.T;

	BlockData1.DataVORDME.Fi = NavData[ID_VOR_DME1].Dt.Fi;
	BlockData1.DataVORDME.La = NavData[ID_VOR_DME1].Dt.La;
	BlockData1.DataVORDME.H = NavData[ID_VOR_DME1].Dt.H;
	BlockData1.DataVORDME.RMS = NavData[ID_VOR_DME1].Dt.SKO_Fi+NavData[ID_VOR_DME1].Dt.SKO_La+NavData[ID_VOR_DME1].Dt.SKO_H;
	BlockData1.DataVORDME.T = NavData[ID_VOR_DME1].Dt.T;

	strcpy(BlockData1.dtk_to, (char*)WindNav.dtk_to);
	strcpy(BlockData1.tk, (char*)WindNav.tk);
	BlockData1.CS = 0;

}
// Одно секундная задача записи в файл
void ProcLog() {
	if (FILE_DEBUG == 1) {
		state = 1; // switch to work
		FILE *fp/*, *fp2*/;
		// проверка на существование
		if(!fsExists(log_file_work)) 
			fp = fopen(log_file_work,"rb+");
		else 
			fp = fopen(log_file_work,"ab");

		// сохранение в память
		write_data_struct();
		// контроль размеров файлов и запись
		file_size_work += sizeof(BlockData1);
		fwrite(&BlockData1,sizeof(BlockData1),1,fp);
		if (strcmp(log_file_work,log_file1) == 0) 
			file_size1 = file_size_work;
		if (strcmp(log_file_work,log_file2) == 0)
			file_size2 = file_size_work;

		if (file_size_work > FILE_SIZE) {
			printf("Logger: swap files [max: %d, %d]\n", FILE_SIZE, file_size_work);
			swap_files();
			file_size_work = 0;
		}
		fflush(fp);
		fclose(fp); 
	}
}
// для отображения
void log_exp_load(TmForm* Frm) {
	strcpy(exp_status,__ML_STATUS_LINE_WAIT); // ожидание экспорта
}
// для отображения
void log_exp_work(TmForm* Frm) {
	log_file_size = file_size_work; // отображение размера
	file_size_max = FILE_SIZE; // отображение максимального размера
}


extern std::vector<std::string> UsbDisk;
extern int fsGetUsbDrives(std::vector<std::string> *ret);

// обработка кнопок
INT32S log_exp_onkey(TmRectangle *PRec, INT32S &Key) {
	if (Key == kl_ENT) 
		log_export();
	if (Key == kl_ESC) 
		RequestForm_1(_UsbNext_,-1);
	return 1;
}

// Экспорт лог-файла
void log_export() {
		#ifndef _WINDOWS
		char USB_Folder2[] = "/media/usb2/";
		strcpy(Save_Name, USB_Folder2);
		#endif
		#ifdef _WINDOWS
		strcpy(Save_Name, UsbDisk[0].c_str());
		#endif
		strcat(Save_Name, "log1.dat"); // export name
		FILE *header = fopen(Save_Name,"wb");
		char block_sz[BLOCK_SZ], records_n[BLOCK_SZ];
		sprintf(block_sz,"=[%d]",sizeof(data_logger)); // запись в файл 8 байтов с размером структуры
		sprintf(records_n,"=[%d]",record_total); // запись в файл 8 байтов с количеством записей структур
		fwrite(block_sz,sizeof(block_sz),1,header); // 
		fwrite(records_n,sizeof(records_n),1,header); //
		fwrite(data_logger,sizeof(data_logger),1,header); // запись примера структуры
		fclose(header);
		puts("Logger: header writed");
		INT8U* buf[1]; unsigned len;
		puts("Logger: start write data");
		printf("Logger: path %s\n", Save_Name);
		unsigned int result = ReadFileInBuf(log_file_wait, buf, len);
		// запись первого файла
		if (result == 2)Expotr_Error[14] = len; // len == errno !
		if (result == 1)Expotr_Error[15] = 0xff;
		if (result == 3)Expotr_Error[16] = 0xff;

		if (result == 0) {   // write
			
			FILE *FileTo = fopen(Save_Name, "ab+");
			if (FileTo == 0) 
				Expotr_Error[17] = errno;
			else {

				result = fwrite(buf[0], len, 1, FileTo);
				if (result < 1) { Expotr_Error[18] = 0xff; printf("Logger: ERROR FILE SAVE\n"); }
				else printf("Logger: Saved OK\n");
				fclose(FileTo);
				fsSync();			
			}
			delete[] buf[0];
		}
		// запись второго файла
		result = ReadFileInBuf(log_file_work, buf, len);
		if (result == 2)Expotr_Error[14] = len; // len == errno !
		if (result == 1)Expotr_Error[15] = 0xff;
		if (result == 3)Expotr_Error[16] = 0xff;

		if (result == 0) {   // write
			#ifndef _WINDOWS
			char USB_Folder2[] = "/media/usb2/";
			strcpy(Save_Name, USB_Folder2);
			#endif
			#ifdef _WINDOWS
			strcpy(Save_Name, UsbDisk[0].c_str());
			#endif
			strcat(Save_Name, "log1.dat"); // export name
			FILE *FileTo = fopen(Save_Name, "ab+");
			if (FileTo == 0) 
				Expotr_Error[17] = errno;
			else {
				result = fwrite(buf[0], len, 1, FileTo);
				if (result < 1) { Expotr_Error[18] = 0xff; printf("Logger: ERROR SAVE\n"); }
				else {
					printf("Logger: Saved OK\n");
					strcpy(exp_status,__ML_STATUS_LINE_SUCCESS);
				}
				fclose(FileTo);

				fsSync();			
			}
			delete[] buf[0];
		}
		last_element = 0; // reset 
	return;
}
// Вычисление размеров переменных
unsigned int get_size(char data_type[]) {
	unsigned int result = 0;
	
	if (strcmp("unsigned int",data_type) == 0)
		result = sizeof(unsigned int);
	if (strcmp("int",data_type) == 0)
		result = sizeof(int);
	if (strcmp("float",data_type) == 0)
	result = sizeof(float);
	if (strcmp("double",data_type) == 0)
	result = sizeof(double);
	if (strcmp("unsigned char",data_type) == 0)
	result = sizeof(unsigned char);
	if (strcmp("char",data_type) == 0)
	result = sizeof(char);
	if (strcmp("short int",data_type) == 0)
	result = sizeof(short int);
	if (strcmp("unsigned short int",data_type) == 0)
	result = sizeof(unsigned short int);
	if (strcmp("long int",data_type) == 0)
	result = sizeof(long int);
	
	return result;
}
// добавление переменной
void add_element(char data_type[], char name[]) {
	if (strcmp(data_type,name) == 0) {
		sprintf(data_logger[last_element],"$[%s]![%s]=[%d]",data_type,name,sizeof(data_logger));
		last_element++;
		return;
	}		
	sprintf(data_logger[last_element],"$[%s]![%s]=[%d]",data_type,name,get_size(data_type));
	tmp_sum += get_size(data_type);
	last_element++;
}
// добавление текстовой переменной
void add_element_char(char data_type[], char name[],unsigned int symbols) {
	sprintf(data_logger[last_element],"$[%s]![%s]=[%d]",data_type,name,sizeof(char)*symbols);
	tmp_sum += sizeof(char)*symbols;
	last_element++;
}


// добавление примера структуры
void write_data_pre() {
	add_element("unsigned int","ID");
	add_element("unsigned int","Number");
	add_element("unsigned int","NumberWkl");
	add_element("unsigned int","LenData");
	add_element_char("char","current_date",12);
	add_element_char("char","current_time",10);
	add_element("double","DataComplex.T");
	add_element("double","DataComplex.Fi");
	add_element("double","DataComplex.La");
	add_element("float","DataComplex.H");
	add_element("float","DataComplex.RMS");
	add_element("double","DataGNSS.T");
	add_element("double","DataGNSS.Fi");
	add_element("double","DataGNSS.La");
	add_element("float","DataGNSS.H");
	add_element("float","DataGNSS.RMS");
	add_element("double","DataSWS.T");
	add_element("double","DataSWS.Fi");
	add_element("double","DataSWS.La");
	add_element("float","DataSWS.H");
	add_element("float","DataSWS.RMS");
	add_element("double","DataDMEDME.T");
	add_element("double","DataDMEDME.Fi");
	add_element("double","DataDMEDME.La");
	add_element("float","DataDMEDME.H");
	add_element("float","DataDMEDME.RMS");
	add_element("double","DataINS.T");
	add_element("double","DataINS.Fi");
	add_element("double","DataINS.La");
	add_element("float","DataINS.H");
	add_element("float","DataINS.RMS");
	add_element("double","DataVORDME.T");
	add_element("double","DataVORDME.Fi");
	add_element("double","DataVORDME.La");
	add_element("float","DataVORDME.H");
	add_element("float","DataVORDME.RMS");
	add_element_char("char","dtk_to",8);
	add_element_char("char","tk",8);
	add_element("unsigned int","CRC");


	printf("Logger: struct size %d\n", tmp_sum);
}

