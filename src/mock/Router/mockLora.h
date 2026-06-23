#pragma once
#if defined(MOCK_TEST_MODE)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Lora/loramesh.h"

// Declaração da função da tarefa
void vTaskSend2Tcp(void *pvParameters);
void vTaskSend2Lora(void *pvParameters);
#endif