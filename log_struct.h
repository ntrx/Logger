#ifndef LOG_STRUCT_H
#define LOG_STRUCT_H
#define LOG_ID 10601
#pragma pack(push, 1)
struct TRecordData {
 //INT32U Status;   		//����� �������������� �������
//unsigned int size; // ������ ������� �������
	double T;				//����� �������������� �������
	double Fi;				//������
	double La;				//�������
	float H;				//������
	float RMS;			//�������� �� ���������
 //FP32 RMSH;			//�������� �� ������
 //FP32 VFi;			//�������� �� ������
 //FP32 VLa;			//�������� �� �������
 //FP32 VH;			//�������� �� ������
 //INT32S Activate;		//������� ��������� ��������
 //INT8U GS[8];			//�������� ������������ �����
 //INT8U TRK[8];			//������� ������� ����
 //INT8U DTK[8];			//�������� ������� ����
 //INT8U BRG[8];			//������ �� �����
 //INT8U DIS[8];			//���������� �� ����� ��������� �������
 //INT8U ATD[8];			//���������� �� ����� ��������
 //INT8U XTK[10];			//���������� �� ��������
 //INT8U ETE[6];			//����� ������ �� �����
 //INT8U ETA[6];			//����� �������� � �����
 //INT8U TKE[8];			//���������� �� ��������� �����
 //FP32 Offset;			//�������� �� ��������
 //char toPoint[8];			//������� ����� ������� ��������
};
//
//
struct TBlockData {
	unsigned int ID;           		//�������������
	unsigned int Number;       	//����� ������
	unsigned int NumberWkl;     	//����� ���������
	unsigned int LenData;      	//������ ������ ���������
 //INT32U LenRecord;      	//������ ��������� TRecordData
 	char current_date[12];     		//��� ���� 2019-06-21
 	char current_time[10];	// ��� ����� 00:00:00
 //INT32S Time; 			//�����
 //INT32U VersiaCP;		//������ ��
 //INT8U Decl[5];			//��������� ���������
	struct TRecordData DataComplex;	//������������� ������, (����� ���� ��� ��� ins ����)
	struct TRecordData DataGNSS;
	struct TRecordData DataSWS;
	struct TRecordData DataDMEDME;
	struct TRecordData DataINS;
	struct TRecordData DataVORDME;
	char dtk_to[8]; //WindNav.dtk_to;
	char tk[8]; //WindNav.tk;
    unsigned int CS;			//����������� �����
};
extern struct TBlockData BlockData1;
#pragma pack(pop)
#endif