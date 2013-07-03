/*
 * client_arm.cpp
 *
 *  Created on: 20.01.2012
 *      Author: korrav
 */
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <cmath>
#include "client_arm_name.h"
#include <sys/types.h>
#include <cstring>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <pthread.h>
#include <sys/wait.h>

using namespace std;

int mad_port = 31000; //порт локального адреса сокета управления МАДа
int mad_port_data = 31001; //порт локального адреса сокета передачи данных МАДа
int mad_port_monitor = 31003; //порт локального адреса сокета передачи блоков мониторограмм МАДа
char b_Addr[15] = "192.168.203.30"; //строка, задающая  адрес БЭГа
int bag_port = 31002; //порт локального адреса сокета управления БЭГа
int bag_port_data = 31001; //порт локального адреса сокета передачи данных БЭГа
int bag_port_monitor = 31003; //порт локального адреса сокета передачи блоков мониторограмм БЭГа
int id_mad = 1; //идентификатор модуля МАД
int init_NoiseThreshold = 20; //величина шумового порога
int sockHandle = 0; //дескриптор сокета управления МАД
int sockHandle_data = 0; //дескриптор сокета передачи данных
int sockHandle_monitor = 0; //дескриптор сокета передачи блоков мониторограмм
sockaddr_in bagAddr_control; // адрес сокета управления БЭГа
sockaddr_in bagAddr_data; //текущий локальный адрес сокета передачи данных БЭГа
sockaddr_in bagAddr_monitor; //текущий локальный адрес сокета передачи блоков мониторограмм БЭГа
sockaddr_in madAddr_control; //текущий локальный адрес сокета управления клиента МАДа
sockaddr_in madAddr_data; //текущий локальный адрес сокета передачи данных клиента МАДа
sockaddr_in madAddr_monitor; //текущий локальный адрес сокета передачи блоков мониторограмм клиента МАДа
dg que[QSIZE + 1]; //очередь сообщений
extern int iget; //следующий элемент для обработки в цикле программы
extern int iput; //следующий элемент для обработки в цикле обработчика сигнала SIGIO
extern int nqueue; //количество дейтаграмм в очереди
static status_MAD *pstatusMAD = NULL; //указатель на глобальную структуру состояния модуля МАД
int flags = 0; //набор флагов
static int shmid = 0, shmid_mon; // дескриптор разделяемой памяти для глобальной структуры состояния и структуры мониторограммы
static int semid; //дескриптор семафора
//static void data_transfer_proc(void); //дочерний процесс, осуществляющий передачу данных в БЭГ
static void data_acquisition_from_ADC(void); //дочерний процесс, отвечающий за приём данных от АЦП и передачу их для обработки в ядро DSP
char node_of_adc[20] = "/dev/", node_of_amplifier[20] = "/dev/",
		node_of_multiplexer[20] = "/dev/";
int fd_adc, fd_amp, fd_mux; //дескрипторы узлов ацп, усилителя и мультиплексора
Monitor* monitor_status; //мониторограмма
pid_t pid_acquis; //идентификатор процесса data_acquisition_from_ADC

//ПЕРЕМЕННЫЕ ДЛЯ ПЕРВОГО АЛГОРИТМА РАСПОЗНАВАНИЯ
int circ_buf_f1[SIZE_BUF_F1 * 4];	//буфер для первого алгоритма распознавания

//ФУНКЦИИ ATEXIT
void at_IPC(void) //освобождение IPC ресурсов
		{
	//удаление сегментов разделяемой памяти
	shmctl(shmid, IPC_RMID, 0);
	shmctl(shmid_mon, IPC_RMID, 0);
	//удаление семафора
	semctl(semid, 0, IPC_RMID, 0);
}

void at_kill_children(void) //уничтожение дочерних процессов
		{
	int status;
	//убийство процесса data_acquisition_from_ADC
	kill(pid_acquis, SIGTERM);
	waitpid(pid_acquis, &status, 0);
	if (WIFEXITED(status))
		printf(
				"The process data_acquisition_from_ADC normally ended with exit status = %d\n",
				WEXITSTATUS(status));
	else if (WIFSIGNALED(status))
		printf(
				"The process data_acquisition_from_ADC was killed by a signal = %d\n",
				WTERMSIG(status));
}

//ОБРАБОТЧИК СИГНАЛА SIGTERM
void hand_SIGTERM(int signo) {
	printf(
			"The process pid = %d has completed its work when the input signal SIGTERM\n",
			getpid());
	exit(0);
}

//ОБРАБОТЧИК СИГНАЛА SIGCHLD
void hand_SIGCHLD(int signo) {
	printf("One of the children completed before the parent process\n");
	exit(0);
}

int main(int argc, char *argv[]) {
	struct sigaction act_io; //структура, определяющая обработчик сигнала для SIGIO
	struct sigaction act_chld; //структура, определяющая обработчик сигнала для CHLD
	char trans_buffer[4]; //буфер, служащий для коммуникации с драйверами устройств
	unsigned short buf_gain[4] = { INIT_GAIN1_DB, INIT_GAIN2_DB, INIT_GAIN3_DB,
			INIT_GAIN4_DB }; //буфер, служащий для коммуникации с драйвером усилителя
	pid_t tmp_pid = 0;
	printf("Size struct status_MAD = %d\n", sizeof(status_MAD));
	//Инициализация аргументами командной строки соответствующих параметров программы
	for (int i = 1; i < argc; i += 2) {
		if (!strcmp("--mpc", argv[i]))
			mad_port = atoi(argv[i + 1]);
		else if (!strcmp("--mpd", argv[i]))
			mad_port_data = atoi(argv[i + 1]);
		else if (!strcmp("--mpm", argv[i]))
			mad_port_monitor = atoi(argv[i + 1]);
		else if (!strcmp("--bip", argv[i]))
			strcpy(b_Addr, argv[i + 1]);
		else if (!strcmp("--bpc", argv[i]))
			bag_port = atoi(argv[i + 1]);
		else if (!strcmp("--bpd", argv[i]))
			bag_port_data = atoi(argv[i + 1]);
		else if (!strcmp("--bpm", argv[i]))
			bag_port_monitor = atoi(argv[i + 1]);
		else if (!strcmp("--idm", argv[i]))
			id_mad = atoi(argv[i + 1]);
		else if (!strcmp("--ans", argv[i]))
			init_NoiseThreshold = atoi(argv[i + 1]);
		else if (!strcmp("--g1", argv[i]))
			buf_gain[0] = static_cast<unsigned short>(atoi(argv[i + 1]));
		else if (!strcmp("--g2", argv[i]))
			buf_gain[1] = static_cast<unsigned short>(atoi(argv[i + 1]));
		else if (!strcmp("--g3", argv[i]))
			buf_gain[2] = static_cast<unsigned short>(atoi(argv[i + 1]));
		else if (!strcmp("--g4", argv[i]))
			buf_gain[3] = static_cast<unsigned short>(atoi(argv[i + 1]));
		else if (!strcmp("--help", argv[i])) {
			printf(
					"\t--mpc     Port control MAD socket.                           default 31000\n"
							"\t--mpd     Port data MAD socket.                              default 31001\n"
							"\t--mpm     Port monitoring MAD socket.                        default 31003\n"
							"\t--bip     IP address of the BAG.                             default 192.168.203.30\n"
							"\t--bpc     Port control BAG socket.                           default 31002\n"
							"\t--bpd     Port data BAG socket.                              default 31001\n"
							"\t--bpm     Port monitoring BAG socket.                        default 31003\n"
							"\t--idm     Identifier MAD.                                    default 1\n"
							"\t--img     The full name of the executable DSP program.       default ./client_dsp_dsplink2.out\n"
							"\t--lwi     Window length measurements (in samples).           default 300\n"
							"\t--lst     The length of the step of sliding (in samples).    default 10\n"
							"\t--gx     Value gain of channel x (in dB).    default 51\n"
							"\t--ans     The value of the noise floor.                      default 0\n");
			exit(0);
		} else {
			printf("Invalid argument!!!\n");
			exit(1);
		}
	}
	//Создание глобальной структуры состояния модуля и структуры мониторограммы МАД в разделяемой памяти
	shmid = shmget(IPC_PRIVATE, sizeof(status_MAD), 0600);
	if (shmid == -1) {
		perror(
				"Process of creation of a segment of divided memory has failed\n");
		exit(1);
	}
	shmid_mon = shmget(IPC_PRIVATE, sizeof(Monitor), 0600);
	if (shmid == -1) {
		perror(
				"Process of creation of a segment of divided memory for struct Monitor has failed\n");
		exit(1);
	}
	atexit(at_IPC);
	pstatusMAD = reinterpret_cast<status_MAD*>(shmat(shmid, NULL, 0));
	if (pstatusMAD == reinterpret_cast<status_MAD*>(-1)) {
		perror(
				"Process of joining of a segment of divided memory has failed\n");
		exit(1);
	}
	monitor_status = reinterpret_cast<Monitor*>(shmat(shmid_mon, NULL, 0));
	if (monitor_status == reinterpret_cast<Monitor*>(-1)) {
		perror(
				"Process of joining of a segment of divided memory for struct Monitor has failed\n");
		exit(1);
	}

	pstatusMAD->modeData_aq = DETECTION1;
	pstatusMAD->NoiseThreshold = init_NoiseThreshold;
	pstatusMAD->ident = COM_GET_STATUS_MAD;

	//получение полных имён узлов устройств и открытие их файлов
	strcat(node_of_adc, ADC_NAME);
	strcat(node_of_amplifier, AMPLIFIER_NAME);
	strcat(node_of_multiplexer, MULTIPLEXER_NAME);

	fd_adc = open(node_of_adc, O_RDWR);
	if (fd_adc == -1) {
		perror("couldn't open the file for ADC\n");
		exit(1);
	}

	fd_amp = open(node_of_amplifier, O_RDWR);
	if (fd_amp == -1) {
		perror("couldn't open the file for amplifier\n");
		exit(1);
	}

	fd_mux = open(node_of_multiplexer, O_RDWR);
	if (fd_mux == -1) {
		perror("couldn't open the file for multiplexer\n");
		exit(1);
	}

	//установка мультиплексора в режим "сигнал"
	for (int i = 0; i < 4; i++)
		trans_buffer[i] = ARG_W_MULTIPLEXER_SIGNAL;
	if (write(fd_mux, trans_buffer, sizeof(trans_buffer)) == -1) {
		perror("the write function for the multiplexer failed\n");
		exit(1);
	}

	if (write(fd_amp, buf_gain, sizeof(buf_gain)) == -1) {
		perror("the write function for the amplifier failed\n");
		exit(1);
	}
	pstatusMAD->gain[0] = buf_gain[0];
	pstatusMAD->gain[1] = buf_gain[1];
	pstatusMAD->gain[2] = buf_gain[2];
	pstatusMAD->gain[3] = buf_gain[3];

	//создание сокета
	sockHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockHandle == -1) {
		perror("socket not create\n");
		exit(1);
	}
	printf("The management socket is created\n");
	//установка адреса клиента МАД и связывание адреса с сокетом
	madAddr_control.sin_family = AF_INET;
	madAddr_control.sin_port = htons(mad_port);
	madAddr_control.sin_addr.s_addr = htonl(INADDR_ANY );
	if (bind(sockHandle, reinterpret_cast<sockaddr*>(&madAddr_control),
			sizeof(madAddr_control))) {
		perror("socket not bind\n");
		exit(1);
	}
	printf("The management socket is connected with the address\n");
	//установка адреса БЭГа
	bagAddr_control.sin_family = AF_INET;
	bagAddr_control.sin_port = htons(bag_port);
	if (inet_pton(AF_INET, b_Addr,
			reinterpret_cast<void*>(&bagAddr_control.sin_addr.s_addr)) != 1) {
		perror("name of address bag not create\n");
		exit(1);
	}
	sendto(sockHandle, reinterpret_cast<void*>(req_Start), sizeof(req_Start), 0,
			reinterpret_cast<sockaddr*>(&bagAddr_control),
			sizeof(bagAddr_control));
	printf("The inquiry about program start is sent\n");

	//формирование очереди сообщений
	for (int i = 0; i < QSIZE; i++) { //выделение памяти под буферы очереди
		que[i].dg_data = reinterpret_cast<int*>(malloc(LEN_BUF_SOCKET));
	}
	act_io.sa_handler = hand_SIGIO;
	act_io.sa_flags = SA_RESTART;
	sigfillset(&(act_io.sa_mask));
	if (sigaction(SIGIO, &act_io, NULL)) {
		perror("sigaction for SIGIO has not worked\n");
		exit(1);
	}
	if (-1 == fcntl(sockHandle, F_SETOWN, getpid())) { //установка процесса в качестве владельца сокета
		perror("Error set F_SETOWN\n");
		exit(1);
	}
	//установка сокета управления в режим неблокируемого и асинхронного ввода-вывода
	if ((flags = fcntl(sockHandle, F_GETFL, 0)) < 0) {
		perror(
				"The call for a socket of management of function fcntl with parameter F_GETFL has come to the end with failure\n");
		exit(1);
	}
	flags |= O_NONBLOCK | O_ASYNC;
	if ((flags = fcntl(sockHandle, F_SETFL, flags)) < 0) {
		perror(
				"Installation of a socket of management in a mode of not blocked and asynchronous input-output has come to the end with failure\n");
		exit(1);
	}

	//конфигурирование масок
	sigemptyset(&zeromask);
	sigemptyset(&newmask);
	sigemptyset(&oldmask);
	sigaddset(&newmask, SIGIO);

	//вызовы дочерних процессов
	atexit(at_kill_children);
	act_chld.sa_handler = hand_SIGCHLD;
	act_chld.sa_flags = SA_NOCLDSTOP | SA_RESETHAND;
	sigfillset(&(act_chld.sa_mask));
	if (sigaction(SIGCHLD, &act_chld, NULL)) {
		perror("sigaction for SIGCHLD has not worked\n");
		exit(1);
	}
	if (!(tmp_pid = fork()))
		data_acquisition_from_ADC();
	else
		pid_acquis = tmp_pid;

	//установка политики планирования для процесса
	sched_param sp;
	sp.__sched_priority = sched_get_priority_min(SCHED_FIFO) + 1; //статический приоритет планирования
	if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1)
		perror("sched_setscheduler\n");

	//ЦИКЛ ОЖИДАНИЯ КОМАНДЫ И ВЫПОЛНЕНИЕ КОМАНД БЭГА
	sigprocmask(SIG_BLOCK, &newmask, &oldmask); //блокирование сигнала SIGIO
	for (;;) {
		while (nqueue == 0) {
			prevCom.len = 0;
			sigsuspend(&zeromask);
		}
		sigprocmask(SIG_SETMASK, &oldmask, NULL); //разблокирование сигнала SIGIO

		switch (que[iget].dg_data[0]) { //декодирование команды
		case COM_SET_MODE: //УСТАНОВИТЬ НОВОЕ ЗНАЧЕНИЕ РЕЖИМА РАБОТЫ
			if (que[iget].dg_len == sizeof(com_SET_MODE)) {
				if (repeat_com(que[iget], prevCom)) {
					printf("Repeated command: %s\n", "COM_SET_MODE");
					break;
				}
				if (que[iget].dg_data[1]
						< 0|| que[iget].dg_data[1]>MAX_NUM_ID_MODE) {printf("Wrong value mode\n");
				sendto(sockHandle, reinterpret_cast<void *>(ans_SET_MODE_NOT_OK), sizeof(ans_SET_MODE_NOT_OK), 0, reinterpret_cast<sockaddr*>(&bagAddr_control), sizeof(bagAddr_control));
				break;
			}
				pstatusMAD->modeData_aq =
						static_cast<data_collection_mode>(que[iget].dg_data[1]);
				printf(
						"The identifier of operation mode MADa became equal %d\n",
						que[iget].dg_data[1]);
				sendto(sockHandle, reinterpret_cast<void *>(ans_SET_MODE_OK),
						sizeof(ans_SET_MODE_OK), 0,
						reinterpret_cast<sockaddr*>(&bagAddr_control),
						sizeof(bagAddr_control));
			}
			break;
		case COM_SET_GAIN: //УСТАНОВИТЬ НОВОЕ ЗНАЧЕНИЕ КОЭФФИЦИЕНТА УСИЛЕНИЯ
			if (que[iget].dg_len == sizeof(com_SET_GAIN)) {
				printf("Command COM_SET_GAIN is accepted\n");
				if (repeat_com(que[iget], prevCom)) {
					printf("Repeated command: %s\n", "COM_SET_GAIN");
					break;
				}
				for (int i = 1; i < 5; i++)
					if (que[iget].dg_data[i] < 0 || que[iget].dg_data[i] > 63) {
						printf("Wrong value gain\n");
						sendto(sockHandle,
								reinterpret_cast<void *>(ans_SET_GAIN_NOT_OK),
								sizeof(ans_SET_GAIN_NOT_OK), 0,
								reinterpret_cast<sockaddr*>(&bagAddr_control),
								sizeof(bagAddr_control));
						break;
					}

				//изменение значения коэффициента усиления
				for (int i = 0; i < 4; i++) {
					pstatusMAD->gain[i] = buf_gain[i] =
							que[iget].dg_data[i + 1];
				}
				if (write(fd_amp, buf_gain, sizeof(buf_gain)) == -1) {
					perror("the write function for the amplifier failed\n");
					exit(1);
				}
				printf(
						"The gain amount became equal for channel 1 %d decibel, for channel 2 %d decibel, for channel 3 %d decibel, "
								"for channel 4 %d decibel\n",
						pstatusMAD->gain[0], pstatusMAD->gain[1],
						pstatusMAD->gain[2], pstatusMAD->gain[3]);
				sendto(sockHandle, reinterpret_cast<void *>(ans_SET_GAIN_OK),
						sizeof(ans_SET_GAIN_OK), 0,
						reinterpret_cast<sockaddr*>(&bagAddr_control),
						sizeof(bagAddr_control));
			}
			break;
		case COM_SET_NSAMPL_MONITOR: //УСТАНОВИТЬ НОВОЕ ЗНАЧЕНИЕ ДЛЯ ПАРАМЕТРА КОЛИЧЕСТВО ОТСЧЁТОВ ДЛЯ МОНИТОРОГРАММЫ
			if (que[iget].dg_len == sizeof(com_SET_NSAMPL_MONITOR)) {
				if (repeat_com(que[iget], prevCom)) {
					printf("Repeated command: %s\n", "COM_SET_NSAMPL_MONITOR");
					break;
				}
				monitor_status->num_sampl = que[iget].dg_data[1];
				printf("The new parameter value of num_sampl is equal %d\n",
						que[iget].dg_data[1]);
				sendto(sockHandle,
						reinterpret_cast<void *>(ans_SET_NSAMPL_MONITOR_OK),
						sizeof(ans_SET_NSAMPL_MONITOR_OK), 0,
						reinterpret_cast<sockaddr*>(&bagAddr_control),
						sizeof(bagAddr_control));
			}
			break;
		case COM_GET_STATUS_MAD: //КОМАНДА ОТПРАВИТЬ ИНФОРМАЦИЮ О ТЕКУЩЕМ СОСТОЯНИИ МАДа
			if (que[iget].dg_len == sizeof(com_GET_STATUS_MAD)) {
				printf(
						"It is accepted request to send information on a current status of MAD\n");
				sendto(sockHandle, reinterpret_cast<void*>(pstatusMAD),
						sizeof(status_MAD), 0,
						reinterpret_cast<sockaddr*>(&bagAddr_control),
						sizeof(bagAddr_control));
			}
			break;
		case COM_RESET_MAD:
			if (que[iget].dg_len == sizeof(com_RESET_MAD)) {
				//совершаются действия по сбросу+++++++++++++++++++++++
				printf("The reset command is accepted\n");
				system("/sbin/reboot");
			}
			break;
		case COM_SET_NOISE:
			if (que[iget].dg_len == sizeof(com_SET_NOISE)) {
				if (repeat_com(que[iget], prevCom)) {
					printf("Repeated command: %s\n", "COM_SET_NOISE");
					break;
				}
				printf(
						"Accepted a request to change the values ​​of the noise threshold\n");
				if (que[iget].dg_data[1] >= 0) {
					pstatusMAD->NoiseThreshold = que[iget].dg_data[1];
					sendto(sockHandle,
							reinterpret_cast<void*>(ans_SET_NOISE_OK),
							sizeof(ans_SET_NOISE_OK), 0,
							reinterpret_cast<sockaddr*>(&bagAddr_control),
							sizeof(bagAddr_control));
				} else
					sendto(sockHandle,
							reinterpret_cast<void*>(ans_SET_NOISE_NOT_OK),
							sizeof(ans_SET_NOISE_NOT_OK), 0,
							reinterpret_cast<sockaddr*>(&bagAddr_control),
							sizeof(bagAddr_control));
			}
			break;
		}
		if (++iget >= QSIZE)
			iget = 0;
		sigprocmask(SIG_BLOCK, &newmask, &oldmask); //блокирование сигнала SIGIO
		nqueue--;
	}
}

//ОБРАБОТЧИК СИГНАЛА SIGIO
void hand_SIGIO(int signo) {
	for (;;) {
		printf("The message is accepted\n");
		if (nqueue >= QSIZE) {
			if ((que[QSIZE].dg_len = recvfrom(sockHandle,
					reinterpret_cast<void *>(que[QSIZE].dg_data),
					LEN_BUF_SOCKET, 0, NULL, NULL)) == -1)
				return;
			if (que[QSIZE].dg_len == 0) { //команда синхронизации
				//передача драйверу АЦП сообщения о том, что произошла синхронизация
				if (ioctl(fd_adc, IOCTL_ADC_SYNC) == -1)
					perror("Not possible to perform the command sync\n");
				else {
					printf("The signal of synchronisation is accepted\n");
					sendto(sockHandle, reinterpret_cast<void *>(ans_Sync),
							sizeof(ans_Sync), 0,
							reinterpret_cast<sockaddr*>(&bagAddr_control),
							sizeof(bagAddr_control));
				}
				continue;
			}
			switch (que[QSIZE].dg_data[0]) {
			case COM_RESET_MAD: //команда сброса
				if (que[QSIZE].dg_len == sizeof(int)) {
					//совершаются действия по сбросу+++++++++++++++++++++++
					printf("The reset command is accepted\n");
					system("/sbin/reboot");
				}
				break;
			}
			continue;
		}
		if ((que[iput].dg_len = recvfrom(sockHandle,
				reinterpret_cast<void *>(que[iput].dg_data), LEN_BUF_SOCKET, 0,
				NULL, NULL)) == -1)
			return;
		if (que[iput].dg_len == 0) { //команда синхронизации
			//передача драйверу АЦП сообщения о том, что произошла синхронизация
			if (ioctl(fd_adc, IOCTL_ADC_SYNC) == -1)
				perror("Not possible to perform the command sync\n");
			else {
				printf("The signal of synchronisation is accepted\n");
				sendto(sockHandle, reinterpret_cast<void *>(ans_Sync),
						sizeof(ans_Sync), 0,
						reinterpret_cast<sockaddr*>(&bagAddr_control),
						sizeof(bagAddr_control));
			}
			continue;
		}
		nqueue++;
		if (++iput >= QSIZE)
			iput = 0;

	}
}

//ДОЧЕРНИЙ ПРОЦЕСС, ПРИНИМАЮЩИЙ ДАННЫЕ ОТ АЦП И ПЕРЕДАЮЩИЙ ИХ В DSP
void data_acquisition_from_ADC() {
	current_mode cur_mode; //определяет текущий режим работы
	cur_mode.mode = 0;
	struct sigaction act_term; //структура, определяющая обработчик сигнала для SIGTERM
	dataUnit_ADC buffer_ADC; //блок данных, учавствующий в обмене сообщениями с АЦП
	dataUnit dataUnit_toBag; //блок данных, передаваемый БЭГу
	buf_f1 buffer_f1; //блок, использующийся для алгоритма filter1
	buffer_f1.buf = &dataUnit_toBag;
	buffer_f1.clear_count = 0;
	dataUnit_toBag.id_MAD = id_mad;
	dataUnit_toBag.ident = SIGNAL_SAMPL;
	//создание сокета передачи данных
	sockHandle_data = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockHandle_data == -1) {
		perror("The socket for data transmission hasn't been created\n");
		exit(1);
	}
	//связывание адреса с сокетом
	madAddr_data = madAddr_control;
	madAddr_data.sin_port = htons(mad_port_data);
	if (bind(sockHandle_data, reinterpret_cast<sockaddr*>(&madAddr_data),
			sizeof(madAddr_data))) {
		perror("The socket for data transmission hasn't been connecte\n");
		exit(1);
	}
	bagAddr_data = bagAddr_control;
	bagAddr_data.sin_port = htons(bag_port_data);
	//получение размера приёмного и передающего буфера
	int sizeRec = 0, sizeSend = 0;
	unsigned int sizeofRect = sizeof(int), sizeofSend = sizeof(int);
	sizeRec = getsockopt(sockHandle_data, SOL_SOCKET, SO_RCVBUF, &sizeRec,
			&sizeofRect);
	sizeRec = getsockopt(sockHandle_data, SOL_SOCKET, SO_SNDBUF, &sizeSend,
			&sizeofSend);
	sizeRec = sizeSend = 2 * MAX_SIZE_SAMPL * 4 * sizeof(int);
	if (setsockopt(sockHandle_data, SOL_SOCKET, SO_RCVBUF, &sizeRec,
			sizeof(int)) == -1)
		perror("Change the size of the receive buffer to crash");
	if (setsockopt(sockHandle_data, SOL_SOCKET, SO_SNDBUF, &sizeSend,
			sizeof(int)) == -1)
		perror("Change the size of the transmit buffer to crash");
	printf("size buffer Rec= %d, size buffer Send =%d\n", sizeRec, sizeSend);
	printf("sizeof size package = %d\n", SIZE_PACK(NUM_SAMPL_UNIT));
	//регистрация обработчика для сигнала SIGTERM
	act_term.sa_handler = hand_SIGTERM;
	sigfillset(&(act_term.sa_mask));
	if (sigaction(SIGTERM, &act_term, NULL)) {
		perror("sigaction for SIGTERM has not worked in data_transfer_proc\n");
		exit(1);
	}
	//инициализация некоторых полей мониторограммы
	monitor_status->ident = ID_MONITOR;
	monitor_status->id_MAD = id_mad;
	monitor_status->num_sampl = INIT_NUM_SAMPL_MONITOR;
	//создание сокета передачи данных мониторограммы
	sockHandle_monitor = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockHandle_monitor == -1) {
		perror("The socket for monitoring data transfer hasn't been created\n");
		exit(1);
	}
	//связывание адреса с сокетом
	madAddr_monitor = madAddr_control;
	madAddr_monitor.sin_port = htons(mad_port_monitor);
	if (bind(sockHandle_monitor, reinterpret_cast<sockaddr*>(&madAddr_monitor),
			sizeof(madAddr_monitor))) {
		perror(
				"The socket for monitoring data transfer hasn't been connected\n");
		exit(1);
	}
	bagAddr_monitor = bagAddr_control;
	bagAddr_monitor.sin_port = htons(bag_port_monitor);
	//запуск АЦП
	if (ioctl(fd_adc, IOCTL_ADC_START) < 0) {
		printf("Start of ADC failed\n");
		exit(1);
	}

	//ЦИКЛ ОБРАБОТКИ СООБЩЕНИЙ, ПОЛУЧАЕМЫХ ОТ АЦП
	for (;;) {
		//какой режим передачи данных используется
		switch (pstatusMAD->modeData_aq) {
		case DETECTION1: //РЕЖИМ ФИЛЬТРОВАННОГО ПОТОКА ДАННЫХ (1 алгоритм распознавания)
			if (!cur_mode.DETECTION1) { //если предыдущая передача была не в режиме DETECTION1
				buffer_f1.clear_count = 0;
				cur_mode.mode = 0;
				cur_mode.DETECTION1 = 1;
			}
			buffer_f1.buf->mode = DETECTION1;
			filter1(circ_buf_f1, SIZE_BUF_F1, &buffer_f1,
					pstatusMAD->NoiseThreshold);
			break;
		case SILENCE:
			/* no break */
		case CONTINUOUS: //РЕЖИМ НЕПРЕРЫВНОГО  ПОТОКА  ДАННЫХ
			if (pstatusMAD->modeData_aq == SILENCE)
				dataUnit_toBag.mode = SILENCE;
			else
				dataUnit_toBag.mode = CONTINUOUS;
			//передача незаполненного блока данных в драйвер АЦП
			buffer_ADC.pUnit = dataUnit_toBag.sampl;
			buffer_ADC.amountCount = NUM_SAMPL_UNIT;
			fill_buffer(buffer_ADC);
			//заполнение полей буфера, передаваемого для обработки в DSP
			dataUnit_toBag.amountCount = buffer_ADC.amountCount;
			dataUnit_toBag.numFirstCount = buffer_ADC.count;
			// получение данных для мониторограммы, а также передача мониторограмм в БЭГ
			process_monitor(buffer_ADC.pUnit, buffer_ADC.amountCount);
			//передача данных в БЭГ
			if (dataUnit_toBag.mode != SILENCE) {
				if (sendto(sockHandle_data,
						reinterpret_cast<void*>(&dataUnit_toBag),
						SIZE_PACK(NUM_SAMPL_UNIT), 0,
						reinterpret_cast<struct sockaddr*>(&bagAddr_data),
						sizeof(bagAddr_data)) == -1)
					perror("Not transmit data\n");
				/*				sendto(sockHandle_data,
				 reinterpret_cast<void*>(&dataUnit_toBag),
				 sizeof(dataUnit), 0,
				 reinterpret_cast<struct sockaddr*>(&bagAddr_data),
				 sizeof(bagAddr_data));*/
				/*								printf(
				 "The data unit with number of the first counting is sent %u\n",
				 dataUnit_toBag.numFirstCount);*/
			}
			break;
		}
	}
}

