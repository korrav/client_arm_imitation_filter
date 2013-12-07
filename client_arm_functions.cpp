/*
 * client_arm_functions.cpp
 *
 *  Created on: 21.01.2012
 *      Author: korrav
 */
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <string.h>
#include <cmath>
#include "client_arm_name.h"

//КОМАНДЫ БЭГА И ОТВЕТЫ МАДА
//оповещение МАДа о включении
int req_Start[3] =
		{ REQ_START,
				VERSION_SOFT_MAD /*текущая версия программного обеспечения МАД*/,
				id_mad /*идентификатор модуля МАД*/};

//команда синхронизации часов МАДов
int ans_Sync[2] = { COM_SYNC, OK };

//установка нового значения режима работы МАД
int com_SET_MODE[2] = { COM_SET_MODE, 0 /*новое значение режима работы*/}; //команда установить новое значение режима работы
int ans_SET_MODE_OK[2] = { COM_SET_MODE, OK }; //ответ на запрос установить новое значение режима работы (OK)
int ans_SET_MODE_NOT_OK[2] = { COM_SET_MODE, NOT_OK }; //ответ на запрос установить новое значение режима работы (NOT OK)

//установка нового значения коэффициента усиления
int com_SET_GAIN[5] = { COM_SET_GAIN, 0 /*новое значение gain канала 1 */,
		0 /* канала 2*/, 0 /*канала 3*/, 0 /*канала 4*/}; //команда установить новое значение коэффициента усиления
int ans_SET_GAIN_OK[2] = { COM_SET_GAIN, OK }; //ответ на запрос установить новое значение коэффициента усиления (OK)
int ans_SET_GAIN_NOT_OK[2] = { COM_SET_GAIN, NOT_OK }; //ответ на запрос установить новое значение коэффициента усиления (NOT OK)

//комада сброса программы МАД
int com_RESET_MAD[1] = { COM_RESET_MAD };

//установка параметра количество отсчётов для мониторограмм
int com_SET_NSAMPL_MONITOR[2] =
		{ COM_SET_NSAMPL_MONITOR, 0 /*новое значение параметра*/}; //команда установить новое значение параметра количество отсчётов для мониторограмм
int ans_SET_NSAMPL_MONITOR_OK[2] = { COM_SET_NSAMPL_MONITOR, OK }; //ответ на запрос установить новое значение параметра количество отсчётов для мониторограммы (OK)
int ans_SET_NSAMPL_MONITOR_NOT_OK[2] = { COM_SET_NSAMPL_MONITOR, NOT_OK }; //ответ на запрос установить новое значение параметра количество отсчётов для мониторограммы (NOT OK)

//пересылка содержимого глобальной структуры состояния МАДа
int com_GET_STATUS_MAD[1] = { COM_GET_STATUS_MAD }; //команда переслать содержимое глобальной структуры состояния МАДа

//установка  нового  значения шумового порога
int com_SET_NOISE[2] = { COM_SET_NOISE, 0 /*значение шумового порога*/}; //команда переслать содержимое глобальной структуры состояния МАДа
int ans_SET_NOISE_OK[2] = { COM_SET_NOISE, OK }; //ответ на запрос установить новое значение шумового порога (OK)
int ans_SET_NOISE_NOT_OK[2] = { COM_SET_NOISE, NOT_OK }; //ответ на запрос установить новое значение шумового порога (NOT_OK)

//установка нового значения параметров wp и wa
int com_SET_WPWA[3] = {COM_SET_WPWA, 0, 0}; //команда переслать содержимое глобальной структуры состояния МАДа
int ans_SET_WPWA_OK[2] = { COM_SET_WPWA, OK };
int ans_SET_WPWA_NOT_OK[2] = { COM_SET_WPWA, NOT_OK };

//сообщение об ошибках в драйвере АЦП
int mes_ERROR_ADC[2] =
		{ MES_ERROR_ADC, 0 /*битовое поле с установленными флагами ошибок*/};

//МАСКИ СИГНАЛОВ
sigset_t zeromask, newmask, oldmask;

//ОЧЕРЕДЬ СООБЩЕНИЙ
int iget = 0, iput = 0, nqueue = 0;
prev_com prevCom = { { 0 }, 0 }; //здесь хранится дейтограмма предыдущей команды
bool repeat_com(const dg& current_dg/*текущая команда*/,
		prev_com& prevCom/*предыдущая команда*/) //функция проверки на идентичность текущей команды и предыдущей
		{
	if (current_dg.dg_data[0] == prevCom.data[0]
			&& current_dg.dg_len == prevCom.len) {
		if (!memcmp(current_dg.dg_data, prevCom.data, prevCom.len)) {
			return (true);
		}
	}
	memcpy(prevCom.data, current_dg.dg_data, current_dg.dg_len);
	prevCom.len = current_dg.dg_len;
	return (false);
}

//функция проверка буфера на ошибки в драйвере
void check_error(dataUnit_ADC& buffer_ADC) {
	if (buffer_ADC.error.error != 0) {
		mes_ERROR_ADC[1] = buffer_ADC.error.error;
		sendto(sockHandle, reinterpret_cast<void*>(mes_ERROR_ADC),
				sizeof(mes_ERROR_ADC), // сообщение БЭГу об ошибке, произошедшей
				0, reinterpret_cast<sockaddr*>(&bagAddr_control),
				sizeof(bagAddr_control)); // в драйвере АЦП
		if (buffer_ADC.error.is_error_cc_edma == 1
				|| buffer_ADC.error.is_error_tc_edma == 1)
			exit(1);
	}
}

//функция заполнения буфера отсчётами
void fill_buffer(dataUnit_ADC& buffer_ADC) {
	int* pos_buffer = buffer_ADC.pUnit;
	int full_count = buffer_ADC.amountCount;
	if (ioctl(fd_adc, IOCTL_ADC_GET_MESSAGE, &buffer_ADC) < 0) { //получение данных от драйвера АЦП
		printf(
				"Receiving a data unit from ADC failed in function fill_buffer\n");
		exit(1);
	}
	//check_error(buffer_ADC);
	int first_count = buffer_ADC.count; //получение первого отсчёта
	int fill_count = buffer_ADC.amountCount; //сколько отсчётов уже получено
	while (fill_count < full_count) {
		buffer_ADC.pUnit += buffer_ADC.amountCount * 4;
		buffer_ADC.amountCount = full_count - fill_count;
		if (ioctl(fd_adc, IOCTL_ADC_GET_MESSAGE, &buffer_ADC) < 0) { //получение данных от драйвера АЦП
			printf(
					"Receiving a data unit from ADC failed in function fill_buffer\n");
			exit(1);
		}
//		check_error(buffer_ADC);
		fill_count += buffer_ADC.amountCount;
	}
	buffer_ADC.amountCount = full_count;
	buffer_ADC.count = first_count;
	buffer_ADC.pUnit = pos_buffer;
	process_monitor(buffer_ADC.pUnit, full_count); //съём мониторограммы
	return;
}
//МОНИТОРОГРАММА
void process_monitor(int* buffer, unsigned int size_count) //функция обработки мониторограммы
		{
//	printf("Input in process_monitor with monitor_status.numsampl = %d\n", monitor_status->num_sampl);
	static int num_sampl_monitorogram = 0; // количество отсчётов, уже учтённых при при вычисление текущей мониторограммы
	static double sum[4] = { 0, 0, 0, 0 }; //сумма выборок
	static long double sumsquares[4] = { 0, 0, 0, 0 }; //сумма квадратов выборок
	static int ct = 0; //счётчик
	if (monitor_status->num_sampl != 0) {
		//вычисление суммы значений отсчётов и суммы квадратов значений отсчётов
		for (unsigned int i = 0; i < size_count; i++) { //интерации по отсчётам
			for (int j = 0; j < 4; j++) { //интерации по каналам
				sum[j] += buffer[4 * i + j];
				sumsquares[j] += pow(static_cast<double>(buffer[4 * i + j]), 2);
			}
			num_sampl_monitorogram++;
			ct++;
			//вычисление мат. ожидания и СКО
			if (num_sampl_monitorogram > AMOUNT_AN) {
				for (int k = 0; k < 4; k++) {
					monitor_status->math_ex[k] = sum[k] / AMOUNT_AN; //математическое ожидание
					monitor_status->dispersion[k] = sqrt(
							(sumsquares[k] - pow(sum[k], 2) / AMOUNT_AN)
									/ (AMOUNT_AN - 1)); //дисперсия
				}
				num_sampl_monitorogram = 0;
				for (int k = 0; k < 4; k++) {
					sum[k] = 0;
					sumsquares[k] = 0;
				}
			}
			if (ct > monitor_status->num_sampl) {
				//передача мониторограммы БЭГу
				sendto(sockHandle_monitor,
						reinterpret_cast<void*>(monitor_status),
						sizeof(Monitor), 0,
						reinterpret_cast<struct sockaddr*>(&bagAddr_monitor),
						sizeof(bagAddr_monitor));
				ct = 0;
			}
		}
	}
}
