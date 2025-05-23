/*
 * SPI1 - Port A
 * SPI1_NSS - CS - PA9
 * SPI1_SCK- CLK - PA5
 * SPI1_MISO- MISO - PA6
 * SPI1_MOSI- MOSI - PA7
 * AF05
 */


#include "stm32f411xx.h"
#include <stdint.h>
#include <string.h>

#define GPIOAEN (1U<<0)
#define SPI1EN (1U<<12)

#define SR_TXE (1U<<1)
#define SR_RXNE (1U<<0)
#define SR_BUSY (1U<<7)

void SPI_GPIO_INIT(void);
void SPI_CONFIG(void);
void CS_ENABLE(void);
void SPI_TRANSMIT(char *data,uint32_t size);
void CS_DISABLE();

int main(void)
{
	char data[]="ABCD";

	SPI_GPIO_INIT();
	SPI_CONFIG();
	while(1)
	{
		CS_ENABLE();
		SPI_TRANSMIT(data,strlen(data));
		CS_DISABLE();
	}
}

void SPI_GPIO_INIT(void)
{
	//1) Enable Clock Access for GPIOA
	RCC->AHB1ENR |= GPIOAEN;

	//2) set alternate function(10) & output Mode(01) in MODER
	//set Output Mode(01) for PA9
	GPIOA->MODER &= ~(1U<<19); //SET 0
	GPIOA->MODER |= (1U<<18); //SET 1

	//set alternate function(10) for PA7
	GPIOA->MODER |= (1U<<15); //SET 1
	GPIOA->MODER &= ~(1U<<14); //SET 0

	//set alternate function(10) for PA6
	GPIOA->MODER |= (1U<<13); //SET 1
	GPIOA->MODER &= ~(1U<<12); //SET 0

	//set alternate function(10) for PA5
	GPIOA->MODER |= (1U<<11); //SET 1
	GPIOA->MODER &= ~(1U<<10); //SET 0

	//3) set alternate function(AF05 - 0101) in AFRL
	//set 0101 for PA5
	GPIOA->AFR[0] |= (1U<<20);
	GPIOA->AFR[0] &= ~(1U<<21);
	GPIOA->AFR[0] |= (1U<<22);
	GPIOA->AFR[0] &= ~(1U<<23);

	//set 0101 for PA6
	GPIOA->AFR[0] |= (1U<<24);
	GPIOA->AFR[0] &= ~(1U<<25);
	GPIOA->AFR[0] |= (1U<<26);
	GPIOA->AFR[0] &= ~(1U<<27);

	//set 0101 for PA7
	GPIOA->AFR[0] |= (1U<<28);
	GPIOA->AFR[0] &= ~(1U<<29);
	GPIOA->AFR[0] |= (1U<<30);
	GPIOA->AFR[0] &= ~(1U<<31);
}

void SPI_CONFIG(void)
{
	//1) Enable clock access for SPI1
	RCC->APB2ENR |= SPI1EN;

	//2) Set the  Clock - Baud rate - fPCLK/4(001)
	SPI1->CR1 |= (1U<<3);
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 &= ~(1U<<5);

	//3) Set CPOL=1, CPHA=1
	//SPI1->CR1 |= (1U<<0); //CPHA
 	SPI1->CR1 &= ~(1U<<1); //CPOL

	//4) Set MSB Transmitted First
	SPI1->CR1 &= ~(1U<<7);

	//5) Set STM32 as Master
	SPI1->CR1 |= (1U<<2);

	//6) Set DFF as 8-bit
	SPI1->CR1 &= ~(1U<<11);

	//7) Select Software Slave Management
	SPI1->CR1 |= (1U<<9); //SSM
	SPI1->CR1 |= (1U<<8); //SSI

	//8) SPI Peripheral Enable
	SPI1->CR1 |= (1U<<6); //SPI

}

void CS_ENABLE(void)
{
	GPIOA->ODR &= ~(1U<<9);
}

void CS_DISABLE(void)
{
	GPIOA->ODR |= (1U<<9);
}

void SPI_TRANSMIT(char *data,uint32_t size)
{
	uint32_t len=0;
	uint8_t temp;
	while(len<size)
	{
		//0: Tx buffer not empty ;1: Tx buffer empty
		while(!( SPI1->SR & SR_TXE)) {} // Wait until Tx Buffer gets Empty - Wait until Data Completely Transmitted from Transmitt Buffer
		SPI1->DR = data[len];
		len++;
	}

	//while(!( SPI1->SR & SR_TXE)) {}
	//SPI1->DR = data;

	while(!( SPI1->SR & SR_TXE)) {} // Wait until Tx Buffer gets Empty - Wait until Data Completely Transmitted from Transmitt Buffer

	/*0: SPI (or I2S) not busy ;1: SPI (or I2S) is busy in communication or Tx buffer is not empty*/
    while(!( SPI1->SR & SR_BUSY)) {} // Wait until SPI is busy in Communication //wait till busy Flag is Set

	//Clear the Overflow Flag
	//Clearing the OVR bit is done by a read operation on the SPI_DR register followed by a read access to the SPI_SR register
	temp=SPI1->DR;
	temp=SPI1->SR;

	(void) temp;

}

