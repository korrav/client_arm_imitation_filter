/*
 * client_arm_name.h
 *
 *  Created on: 20.01.2012
 *      Author: korrav
 */

#ifndef CLIENT_ARM_NAME_H_
#define CLIENT_ARM_NAME_H_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>

//драйверы
#include "pcm4204_driver.h"
#include "pga2500.h"
#include "adg739.h"

//РАЗНОЕ
extern int sockHandle; //дескриптор сокета управления
extern int sockHandle_data; //дескриптор сокета передачи данных
extern int sockHandle_monitor; //дескриптор сокета передачи блоков мониторограмм
extern int id_mad; //идентификатор модуля МАД
#define OK 1	//параметр, сообщающий, что команда успешно выполнена
#define NOT_OK  0//заявление, что команда не выполнена
#define LEN_BUF_SOCKET 1500 //длина приёмного буфера
#define LEN_DSP_ARM 1900	//количество отсчётов, содержащихся в одном блоке данных, передаваемом между DSP и ARM
#define NUM_SAMPL_UNIT 1000	//количество отсчётов, содержащихся в одном блоке данных для режима CONTINIOUS
#define MAX_SIZE_SAMPL 4100//максимальный размер буфера передачи
#define VERSION_SOFT_MAD 1	//текущая версия программного обеспечения МАД
#define INIT_WIDTH_PRIOR_EVENTS 1000 //начальное значение количества отсчётов детектированного сигнала, выбранных до события (DETECT1)
#define INIT_WIDTH_AFTER_EVENTS 2000 //начальное значение количества отсчётов детектированного сигнала, выбранных после события (DETECT1)
union semun {
	int val; //<= value for SETVAL
	struct semid_ds *buf; //<= buffer for IPC_STAT & IPC_SET
	unsigned short int *array; //	<= array for GETALL & SETALL
};

//АДРЕСА
extern int mad_port; //порт локального адреса сокета управления МАДа
extern int mad_port_data; //порт локального адреса сокета передачи данных МАДа
extern int mad_port_monitor; //порт локального адреса сокета передачи блоков мониторограмм МАДа
extern char b_Addr[15]; //строка, задающая  адрес БЭГа
extern int bag_port; //порт локального адреса сокета управления БЭГа
extern int bag_port_data; //порт локального адреса сокета передачи данных БЭГа
extern int bag_port_monitor; //порт локального адреса сокета передачи блоков мониторограмм БЭГа
extern sockaddr_in madAddr_control; //текущий локальный адрес сокета управления  клиента МАДа
extern sockaddr_in madAddr_data; //текущий локальный адрес сокета передачи данных клиента МАДа
extern sockaddr_in madAddr_monitor; //текущий локальный адрес сокета передачи блоков мониторограмм клиента МАДа
extern sockaddr_in bagAddr_control; // адрес сокета управления БЭГа
extern sockaddr_in bagAddr_data; //текущий локальный адрес сокета передачи данных БЭГа
extern sockaddr_in bagAddr_monitor; //текущий локальный адрес сокета передачи блоков мониторограмм БЭГа

//МАКРОСЫ, ОТНОСЯЩИЕСЯ К КОМАНДАМ БЭГА И ОТВЕТАМ МАДА

//запрос о запуске программы на МАДе
#define REQ_START 1	//код оповещения МАДа о включении
extern int req_Start[3]; //дейтаграмма оповещения МАДа о включении

//команда синхронизации часов МАДА
#define COM_SYNC 0
extern int ans_Sync[2]; //Подтверждение МАДом, что синхронизация прошла успешно

//команда сброса
#define COM_RESET_MAD 6
extern int com_RESET_MAD[1]; //команда сбросить программное обеспечение МАД

//установка нового значения режима работы МАДа
#define COM_SET_MODE 4
extern int com_SET_MODE[2]; //команда установить новое значение режима работы
extern int ans_SET_MODE_OK[2]; //ответ на запрос установить новое значение режима работы (OK)
extern int ans_SET_MODE_NOT_OK[2]; //ответ на запрос установить новое значение режима работы (NOT OK)
enum data_collection_mode {
	CONTINUOUS, //непрерывный поток данных
	DETECTION1, //фильтрованный поток данных (1 алгоритм распознавания)
	SILENCE //режим молчания
};

//структура, используемая для определения предыдущего режима работы
union current_mode {
	int mode;
	struct {
		int CONTINUOUS :1; //непрерывный поток данных
		int DETECTION1 :1; //фильтрованный поток данных (1 алгоритм распознавания)
		int SILENCE :1; //режим молчания
	};
};

struct WidthDet1 { //структура содержащая размеры передаваемого пакета зарегистрированного сигнала в режиме DETECT1
	int wp;
	int wa;
	int size;
};
#define MAX_NUM_ID_MODE 2	//самый большой номер идентификатора режима сбора данных
//установка нового значения коэффициента усиления
#define COM_SET_GAIN 2
extern int com_SET_GAIN[5]; //команда установить новое значение коэффициента усиления
extern int ans_SET_GAIN_OK[2]; //ответ на запрос установить новое значение коэффициента усиления (OK)
extern int ans_SET_GAIN_NOT_OK[2]; //ответ на запрос установить новое значение коэффициента усиления (NOT OK)

//установка параметра количество отсчётов для мониторограммы
#define COM_SET_NSAMPL_MONITOR 7
extern int com_SET_NSAMPL_MONITOR[2]; //команда установить новое значение параметра количество отсчётов для мониторограммы
extern int ans_SET_NSAMPL_MONITOR_OK[2]; //ответ на запрос установить новое значение параметра количество отсчётов для мониторограммы (OK)
extern int ans_SET_NSAMPL_MONITOR_NOT_OK[2]; //ответ на запрос установить новое значение параметра количество отсчётов для мониторограммы (NOT OK)

//команда переслать содержимое глобальной структуры состояния МАДа
#define COM_GET_STATUS_MAD 8
extern int com_GET_STATUS_MAD[1]; //команда переслать содержимое глобальной структуры состояния МАДа

//команда установить новое значение шумового порога
#define COM_SET_NOISE 10
extern int com_SET_NOISE[2]; //команда переслать содержимое глобальной структуры состояния МАДа
extern int ans_SET_NOISE_OK[2]; //ответ на запрос установить новое значение шумового порога (OK)
extern int ans_SET_NOISE_NOT_OK[2]; //ответ на запрос установить новое значение шумового порога (NOT_OK)

//команда установить новое значение параметров wp и wa
#define COM_SET_WPWA 11
extern int com_SET_WPWA[3]; //команда переслать содержимое глобальной структуры состояния МАДа
extern int ans_SET_WPWA_OK[2]; //ответ на запрос установить новое значение шумового порога (OK)
extern int ans_SET_WPWA_NOT_OK[2]; //ответ на запрос установить новое значение шумового порога (NOT_OK)

//сообщение об ошибках в драйвере АЦП
#define MES_ERROR_ADC 9
extern int mes_ERROR_ADC[2];
//ГЛОБАЛЬНАЯ СТРУКТУРА СОСТОЯНИЯ МОДУЛЯ МАД
extern struct status_MAD {
	int ident; //идентификатор блока данных (COM_GET_STATUS_MAD)
	int gain[4]; //текущий коэффициент усиления (в абсолютных значениях)
	data_collection_mode modeData_aq; //текущий режим сбора данных МАД
	int NoiseThreshold; //текущий шумовой порог алгоритма распознавания
	int wp; //количество отсчётов до события; используется в режиме DETECT1
	int wa; //количество отсчётов после события; используется в режиме DETECT1
}  *pstatusMAD; //указатель на глобальную структуру состояния модуля МАД;
#define INIT_GAIN1_DB 0	//начальный коэфиициент усиления в дцБ для канала 1
#define INIT_GAIN2_DB 0	//начальный коэфиициент усиления в дцБ для канала 2
#define INIT_GAIN3_DB 0	//начальный коэфиициент усиления в дцБ для канала 3
#define INIT_GAIN4_DB 0	//начальный коэфиициент усиления в дцБ для канала 4
#define NUM_ARG_STATUS_MAD 3	//количество аргументов передаваемых при загрузке образа программы DSP
#define INIT_NOISE_THRESHOLD 0 //величина шумового порога сразу после запуска программы
//ОЧЕРЕДЬ СООБЩЕНИЙ
#define QSIZE 10 //размер очереди сообщений сокета
//структура элемента очереди сообщений сокета
struct dg {
	int* dg_data; //указатель на буфер данных
	int dg_len; //длина буфера
};
extern dg que[QSIZE + 1]; //очередь сообщений
extern int iget; //следующий элемент для обработки в цикле программы
extern int iput; //следующий элемент для обработки в цикле обработчика сигнала SIGIO
extern int nqueue; //количество дейтаграмм в очереди
extern void hand_SIGIO(int signo); //обработчик сигнала SIGIO
extern sigset_t zeromask; //очищенная маска сигналов
extern sigset_t newmask; //маска с установленным флагом SIGIO
extern sigset_t oldmask; //старая маска
struct prev_com {
	int data[250]; //дейтаграмма команды
	int len; //длина дейтограммы команды
};
extern prev_com prevCom; //здесь хранится дейтаграмма предыдущей команды

//СТРУКТУРА БЛОКА ДАННЫХ, КОТОРАЯ ПЕРЕДАЁТСЯ В БЭГ
#define SIGNAL_SAMPL 3	 //код, идентифицирующий блок данных сигнала
//флаги ошибок драйвера АЦП
#define FRAGMENTARY (1 << 0)	//фрагментарность данных. Свидетельствует о том, что программа не успела считать текущий буфер драйвера АЦП,
//до того, как был заполен последующий
#define	PASSED_SYNC (1 << 1)	//пропущенная синхронизация. Если данный флаг установлен, и при этом dataUnit_ADC.first_count == -1, тогда необходимо
//послать запрос о повторной синхронизации, а все данные от АЦП отбраковывать,
//пока не будет встречен буфер с dataUnit_ADC.first_count != -1
struct dataUnit {
	int ident; //идентификатор блока данных
	int mode; //режим сбора данных
	int gain[4]; //текущий коэффициент усиления (в абсолютных значениях)
	unsigned int numFirstCount; //номер первого отсчёта
	unsigned int amountCount; //количество отсчётов (1 отс = 4 x 4 байт)
	int id_MAD; //идентификатор МАДа
	int sampl[MAX_SIZE_SAMPL * 4]; //массив отсчётов
};

//СТРУКТУРА БЛОКА ДАННЫХ, КОТОРАЯ ИСПОЛЬЗУЕТСЯ ДЛЯ ПЕРЕДАЧИ МЕЖДУ DSP и ARM

struct dSPaRM {
	unsigned int numFirstCount; //номер первого отсчёта
	unsigned int amountCount; //количество отсчётов (1 отс = 4 x 4 байт)
	int sampl[LEN_DSP_ARM * 4]; //массив отсчётов
};
//СТРУКТУРА MONITOR
#define ID_MONITOR 4	//код, идентифицирующий блок монитор
#define INIT_NUM_SAMPL_MONITOR	6e+7	//значение по умолчанию количества отсчётов, при котором производится анализ даннных (5 минут)
extern struct Monitor {
	int ident; //идентификатор блока данных
	int id_MAD; //идентификатор МАДа
	int dispersion[4]; //величина дисперсии для каждого канала
	int math_ex[4]; //величина математического ожидания для каждого канала
	int num_sampl; //количество отсчётов, при котором производится анализ даннных
}*monitor_status;
extern void process_monitor(int* buffer, unsigned int size_count); //функция обработки мониторограммы

/*функция проверки на идентичность текущей команды и предыдущей. Если команды идентичны, то возвращается истина.
 * Если команды различны, то возвращается ложь, а также содержимое команды current_dg копируется в prevCom*/
extern bool repeat_com(const dg& current_dg/*текущая команда*/,
		prev_com& prevCom/*предыдущая команда*/);

//СТРУКТУРЫ ИСПОЛЬЗУЕМЫХ УСТРОЙСТВ
extern char node_of_adc[20]; //полное имя узла ацп
extern char node_of_amplifier[20]; //полное имя узла усилителя
extern char node_of_multiplexer[20]; //полное имя узла мультиплексора

//СЕКЦИЯ DSPLINK
#define POOL_ID 0	//идентификатор пула
#define CHNL_ID_INPUT 0 //идентификатор канала, используемого для приёма отфильтрованных буферов от DSP
#define CHNL_ID_OUTPUT 1 //идентификатор канала, используемого для передачи буферов сырых данных в DSP
//NOTIFY
#define IPS_ID 0	//идентификатор экземпляра IPS
#define E_FATAL_ERROR 0                 //фатальная ошибка DSP программы
#define D_WRONG_NUM_ARG 0	            //неверное количество аргументов функции main()
#define D_WRONG_STEP_OR_WINDOW 1		//неверное значение шага или окна скольжения
#define E_START_DSP 1	//номер события, сигнализирующего о полной готовности программы DSP к обработке данных. Посылается от DSP
//ПЕРВЫЙ АЛГОРИТМ РАСПОЗНАВАНИЯ
#define SIZE_BUF_F1	20000	//количество полезных отсчётов в буфере для первого алгоритма распознавания
#define SIZE_PACKAGE	1000 //длина пакета, передаваемого БЭГу в режиме CONTINUOUS (в отсчётах)
struct buf_f1 {	//структура половинного буфера (1 алгоритм распознавания)
	dataUnit* buf;//адрес блока dataUnit, передаваемого в алгоритм распознавания
	int clear_count;//сколько отсчётов  буфера передачи ещё необходимо заполнить
};
void filter1(int* circ_b /*кольцевой буфер*/,
		int numsampl /* количество отсчётов в кольцевом буфере*/, buf_f1* buf, /* буфер, передаваемый БЭГу для заполения*/
		int tres, /*коэффициент умножения на текущий шумовой порог*/
		const WidthDet1& width // размерность пакета передачи
		);

//РАЗНОЕ
void check_error(dataUnit_ADC& buffer_ADC);	//функция проверка буфера на ошибки в драйвере
void fill_buffer(dataUnit_ADC& buffer_ADC); //функция заполнения буфера отсчётами
bool data_transmit(dataUnit& buf, int sampl);//функция передачи блока данных в БЭГ
extern int fd_adc, fd_amp, fd_mux; //дескрипторы узлов ацп, усилителя и мультиплексора
#define AMOUNT_AN 100000	//количество отсчётов, которое учитывается при расчёте мониторограммы
#define SIZE_PACK(x) (sizeof(dataUnit) - (MAX_SIZE_SAMPL - x) * 4 * sizeof(int))	//макрос, определяющий размер пакета режима передачи
#endif/* CLIENT_ARM_NAME_H_ */
