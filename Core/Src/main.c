/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/********* ejemplo con con datos de 16 bits ***********/

#define EJEMPLO_1
uint16_t datoGuardado;

/********* ejemplo con con datos de 32 bits ***********/

//#define EJEMPLO_2
//uint32_t datoGuardado;
/*******************************************************/

uint32_t direccionFlash = 0x0800FC00UL; //primera direccion de la ultima pagina
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
static void MX_GPIO_Init( void );
/* USER CODE BEGIN PFP */

void flash_writeData32( uint32_t address, uint32_t data );
void flash_writeData16( uint32_t address, uint16_t data );

void flash_erasedPag( uint32_t address, uint8_t numPag );

uint32_t flash_readData32( uint32_t address );
uint16_t flash_readData16( uint32_t address );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( void )
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */

    /**
     *  NOTA_1: al hacer el debug paso a paso podremos observar
     *          como cambian los valores de la variable -> datoGuardado.
     *  NOTA_2: debes reservar memoria flash en el archivo STM32F103C8TX_FLASH.ld
     *          disminuyendo la memoria de 64K a 63K, con esto aseguramos que el
     *          compilador no sobreescriba en la pagina de memoria que reservamos.
     */

#ifdef EJEMPLO_1  /********* ejemplo con con datos de 16 bits ***********/

    /* borramos la ultima pagina de la flash */
    flash_erasedPag( direccionFlash, 1 );
    /* leemos las direcciones de memoria para verificar que esten borradas
     * volor despues de borrar el registro de la flash 0xffffffff
     */
    datoGuardado = flash_readData16( direccionFlash );
    datoGuardado = flash_readData16( direccionFlash + 2 );

    /* guardamos 2 valores en la flash 7878 y 23232 */
    flash_writeData16( direccionFlash, 7878 );         // dato 1
    flash_writeData16( direccionFlash + 2, 23232 );    // dato 2
    /* leemos los datos guardados */
    datoGuardado = flash_readData16( direccionFlash );
    datoGuardado = flash_readData16( direccionFlash + 2 );

#else             /********* ejemplo con con datos de 32 bits ***********/

       /* borramos la ultima pagina de la flash */
       flash_erasedPag( direccionFlash, 1 );

       /* leemos las direcciones de memoria para verificar que esten borradas
        * volor despues de borrar el registro de la flash 0xffffffff
        */
       datoGuardado = flash_readData32( direccionFlash );
       datoGuardado = flash_readData32( direccionFlash + 4 );

       /* guardamos 2 valores en la flash 1234567 y 6789012 */
       flash_writeData32( direccionFlash, 1234567 );      // dato 1
       flash_writeData32( direccionFlash + 4, 6789012 );  // dato 2
       /* leemos los datos guardados */
       datoGuardado = flash_readData32( direccionFlash );
       datoGuardado = flash_readData32( direccionFlash + 4 );


#endif
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while ( 1 )
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_2 ) != HAL_OK )
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    __HAL_RCC_GPIOD_CLK_ENABLE( );
    __HAL_RCC_GPIOA_CLK_ENABLE( );

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

}

/* USER CODE BEGIN 4 */

/**
 *
 * @param address direccion donde se guardara el dato
 * @param data dato de 32bits para almacenar en la memoria
 */
void flash_writeData32( uint32_t address, uint32_t data )
{

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    if ( HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, address, data ) != HAL_OK )
    {

    }

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}

/**
 *
 * @param address direccion donde se guardara el dato
 * @param data dato de 16bits para almacenar en la memoria
 */
void flash_writeData16( uint32_t address, uint16_t data )
{

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    if ( HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, address, data )
            != HAL_OK )
    {

    }

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}

/**
 *
 * @param address direccion de la flash
 * @return retorna el dato de 32bits guardado en la flash
 */
uint32_t flash_readData32( uint32_t address )
{

    return *(uint32_t*) address;

}

/**
 *
 * @param address direccion de la flash
 * @return retorna el dato de 16bits guardado en la flash
 */
uint16_t flash_readData16( uint32_t address )
{
    uint16_t dato = *(uint32_t*) address;
    return dato;

}

/**
 *
 * @param address direccion inicial de la pagina
 * @param numPag numero de paginas para borrar
 */
void flash_erasedPag( uint32_t address, uint8_t numPag )
{

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    FLASH_EraseInitTypeDef ErasedStruct;

    ErasedStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    ErasedStruct.PageAddress = address;
    ErasedStruct.NbPages = numPag;

    uint32_t PageError;
    if ( HAL_FLASHEx_Erase( &ErasedStruct, &PageError ) != HAL_OK )
    {

    }

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
