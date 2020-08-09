## Welcome

I am Developing a Custom HAL Driver For STM32F446xx and STM32F407xx MCU.It Currently Supports

- GPIO
- Interrupts

And Support for the following are yet to come

- USART
- SPI
- I2C

### Documentation

For initializing GPIO Pin as OUTPUT -
```markdown
	GPIO_PinOutput(pGPIOx, PinNumber, PinSpeed, PinOPType, PinPUPDC)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)

2. PinNumber -
  Pin Number of the GPIO Port (Parameters:0,1,2,....,15)

3. PinSpeed - 
  Pin Speed of the GPIO Pin (Parameters:LOW,MEDIUM,FAST,FULL)

4. PinOPType -
  Pin Output like Push-Pull and Open-Drain(Parameters:OD,PP)

5. PinPUPDC -
  Used For Activating Internal PushDown-PullUp resistor(Parameters:NO_PUPD,PUSHDOWN,PULLUP)
  
Example : 
```markdown
	GPIO_PinOutput(GPIOA, 5, HIGH, PP, NO_PUPD);
```
<br>


For initializing GPIO Pin as INPUT -
```markdown
	GPIO_PinInput(pGPIOx, PinNumber, PinSpeed, PinPUPDC)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)

2. PinNumber -
  Pin Number of the GPIO Pin (Parameters:0,1,2,....,15)

3. PinSpeed - 
  Pin Speed of the GPIO Pin (Parameters:LOW,MEDIUM,FAST,FULL)

4. PinPUPDC -
  Used For Activating Internal PushDown-PullUp resistor(Parameters:NO_PUPD,PUSHDOWN,PULLUP)
  
Example : 
```markdown
	GPIO_PinInput(GPIOC, 13, HIGH, NO_PUPD);
```





For initializing GPIO Pin as OTHER -
```markdown
	GPIO_PinSetup(pGPIOx, PinNumber, PinMode, PinSpeed, PinOPType, PinPUPDC)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)

2. PinNumber -
  Pin Number of the GPIO Pin (Parameters:0,1,2,....,15)

3. PinMode -
  The Mode of the GPIO Pin (Parameters:INPUT,OUTPUT,ANALOG,ALTFN,IT_FT,IT_RT_IT_RFT)
  _NOTE : The IT_FT,IT_RT_IT_RFT Modes uses Interrupt Mode Of the MCU_

4. PinSpeed - 
  Pin Speed of the GPIO Pin (Parameters:LOW,MEDIUM,FAST,FULL)

5. PinOPType - 
  Sets the GPIO Pin as Open-Drain , Push-Pull (Parameters:OD,PP,NONE)

6. PinPUPDC -
Used For Activating Internal PushDown-PullUp resistor(Parameters:NO_PUPD,PUSHDOWN,PULLUP)

_**NOTE : SET THE PinOPType AS NONE WHEN SETTING THE PIN AS INPUT**_

Example : 
```markdown
	GPIO_PinSetup(GPIOB, 14, ANALOG, HIGH, 0,NO_PUPD);
```




For Writing to a GPIO Pin-
```markdown
	GPIO_WriteToOutputPin(pGPIOx, PinNumber, Value)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)

2. PinNumber -
  Pin Number of the GPIO Port (Parameters:0,1,2,....,15)

3. Value - 
  Value to Write to the GPIO Pin (Parameters:HIGH,LOW)
  
Example : 
```markdown
	GPIO_WriteToOutputPin(GPIOA, 5, 0);
```



For Writing to a GPIO Port-
```markdown
	GPIO_WriteToOutputPin(pGPIOx, Value)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)

2. Value - 
  Value to Write to the GPIO Pin (Parameters:HIGH,LOW)
  
Example : 
```markdown
	GPIO_WriteToOutputPort(GPIOA, 0);
```

For Toggling a GPIO Pin-
```markdown
	GPIO_ToggleOutputPin(pGPIOx, PinNumber)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)

2. PinNumber -
  Pin Number of the GPIO Port (Parameters:0,1,2,....,15)
  
Example : 
```markdown
	GPIO_ToggleOutputPin(GPIOA, 5);
```
For Toggling a GPIO Port-
```markdown
	GPIO_ToggleOutputPort(pGPIOx)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)
  
Example : 
```markdown
	GPIO_ToggleOutputPin(GPIOA);
```

For Reading from a GPIO Pin-
```markdown
	GPIO_ReadFromInputPin(pGPIOx, PinNumber)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)

2. PinNumber -
  Pin Number of the GPIO Port (Parameters:0,1,2,....,15)
  
Example : 
```markdown
	GPIO_ReadFromInputPin(GPIOC, 13);
```
For Reading from a GPIO Port-
```markdown
	GPIO_ReadFromInputPort(pGPIOx)
```
Parameters -
1. pGPIOx-
  Gpio Port (Parameters:GPIOA,GPIOB,....,GPIOH)
  
Example : 
```markdown
	GPIO_ReadFromInputPort(GPIOC);
```
