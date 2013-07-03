/*
 * filter1.cpp
 *
 *  Created on: 13 Sep 2012
 *      Author: andrej
 */
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "client_arm_name.h"

//функция фильтра
void filter1(int* circ_b /*кольцевой буфер*/,
		int numsampl /* количество отсчётов в кольцевом буфере*/,
		buf_f1* buf /* буфер, передаваемый БЭГу для заполения*/, int tres /*коэффициент умножения на текущий шумовой порог*/
		) {
	int* half_buf[2] = { circ_b, circ_b + 2 * numsampl }; //массив, содержащий адреса половин кольцевого буфера
	unsigned int first_count = 0; /*номер первого отсчёта в буфере*/
	dataUnit_ADC buffer_ADC; //буфер, передаваемый АЦП
	//передача незаполненного кольцевого полубуфера  данных в драйвер АЦП
	for (int half = 0; half < 2; half++) { //интерация по половинам кольцевого буфера
		int i = 0;
		buffer_ADC.pUnit = half_buf[half];
		buffer_ADC.amountCount = numsampl / 2;
		fill_buffer(buffer_ADC);
		if (half == 0)
			first_count = buffer_ADC.count;
		//если не заполнен буфер передачи полезным сигналом, часть которого располагается в предыдущем полубуфере
		if (buf->clear_count != 0) {
			memcpy(buf->buf->sampl + (SIZE_PACKAGE - buf->clear_count) * 4,
					half_buf[half], buf->clear_count * 4 * sizeof(int));
			i += buf->clear_count;
			buf->clear_count = 0;
			//printf("Last time the buffer was not filled: i = %d\n", i);
			//передача буфера БЭГу
			sendto(sockHandle_data, reinterpret_cast<void*>(buf->buf),
					SIZE_PACK(SIZE_PACKAGE), 0,
					reinterpret_cast<struct sockaddr*>(&bagAddr_data),
					sizeof(bagAddr_data));
		}
		//проверка имеется ли превышение шумового порога
		for (; i < numsampl / 2; i++) { //интерация по отсчётам
			for (int j = 0; j < 4; j++) { //интерация по каналам
				if (buffer_ADC.pUnit[4 * i + j]
						> tres * monitor_status->dispersion[j]
						|| buffer_ADC.pUnit[4 * i + j]
								< -tres * monitor_status->dispersion[j]) { //превышение порога обнаружено
								//ЗАПОЛЕНИЕ  БУФЕРА , ПЕРЕДАВАЕМОГО В БЭГ, В ЗАВИСИМОСТИ ОТ РАСПОЛОЖЕНИЯ ПОЛЕЗНОГО СИГНАЛА
								//полезный сигнал располагается у нижней границы полубуфера
					if (i > (numsampl - SIZE_PACKAGE) / 2) {
						int fill_count = (SIZE_PACKAGE + numsampl) / 2 - i; //столько данных будет заполенно от данного полубуфера
						buf->clear_count = SIZE_PACKAGE - fill_count;
						memcpy(reinterpret_cast<void*>(buf->buf->sampl),
								reinterpret_cast<void*>(half_buf[half]
										+ (numsampl / 2 - fill_count) * 4),
								fill_count * 4 * sizeof(int));
						buf->buf->numFirstCount = first_count
								+ (half * numsampl / 2) + (i - SIZE_PACKAGE / 2);
						buf->buf->amountCount = SIZE_PACKAGE;
						i = numsampl / 2;
					/*	printf(
								"desired signal is located at the lower boundary buffer: fill_count = %d, first_count = %d, i =%d\n",
								fill_count, buf->buf->numFirstCount, i); */
						break;
						//полезный сигнал располагается у верхней границы полубуфера
					} else if (i < SIZE_PACKAGE / 2 && half == 0) {
						int rest = SIZE_PACKAGE / 2 - i;
						memcpy(&buf->buf->sampl,
								half_buf[1] + (numsampl / 2 - rest) * 4,
								4 * rest * sizeof(int));
						memcpy(&buf->buf->sampl[rest * 4], half_buf[0],
								4 * (SIZE_PACKAGE - rest) * sizeof(int));
						buf->buf->numFirstCount = first_count + i
								- SIZE_PACKAGE / 2;
						i += SIZE_PACKAGE / 2 - 1;
						buf->buf->amountCount = SIZE_PACKAGE;
					/*	printf(
								"desired signal is located at the upper boundary buffer: first_count = %d, i =%d\n",
								buf->buf->numFirstCount, i);  */
						//полезный сигнал располагается далеко от границ буфера
					} else {
						memcpy(&buf->buf->sampl,
								half_buf[half] + i * 4 - 2 * SIZE_PACKAGE,
								4 * SIZE_PACKAGE * sizeof(int));
						buf->buf->numFirstCount = first_count
								+ (half * numsampl / 2) + (i - SIZE_PACKAGE/2);
						buf->buf->amountCount = SIZE_PACKAGE;
						i += SIZE_PACKAGE / 2 - 1;
/*						printf(
								"desired signal is located in the middle of the buffer: first_count = %d, i =%d\n",
								buf->buf->numFirstCount, i);*/
					}
					//передача буфера БЭГу
					sendto(sockHandle_data, reinterpret_cast<void*>(buf->buf),
							SIZE_PACK(SIZE_PACKAGE), 0,
							reinterpret_cast<struct sockaddr*>(&bagAddr_data),
							sizeof(bagAddr_data));
					break;
				}
			}
		}
	}
}

