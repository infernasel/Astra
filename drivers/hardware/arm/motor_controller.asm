;
; ASTRA Hardware Driver: Motor Controller
; Architecture: ARM Cortex-M4
;
; This driver provides low-level access to motor controllers for drones and spacecraft
; using PWM signals to control brushless motors via ESCs (Electronic Speed Controllers).
;

.syntax unified
.cpu cortex-m4
.thumb

; Timer Peripheral Base Address (STM32F4xx example)
.equ TIM1_BASE,            0x40010000
.equ TIM2_BASE,            0x40000000
.equ TIM3_BASE,            0x40000400
.equ TIM4_BASE,            0x40000800

; Timer Register Offsets
.equ TIM_CR1,              0x00    ; Control register 1
.equ TIM_CR2,              0x04    ; Control register 2
.equ TIM_SMCR,             0x08    ; Slave mode control register
.equ TIM_DIER,             0x0C    ; DMA/Interrupt enable register
.equ TIM_SR,               0x10    ; Status register
.equ TIM_EGR,              0x14    ; Event generation register
.equ TIM_CCMR1,            0x18    ; Capture/compare mode register 1
.equ TIM_CCMR2,            0x1C    ; Capture/compare mode register 2
.equ TIM_CCER,             0x20    ; Capture/compare enable register
.equ TIM_CNT,              0x24    ; Counter
.equ TIM_PSC,              0x28    ; Prescaler
.equ TIM_ARR,              0x2C    ; Auto-reload register
.equ TIM_CCR1,             0x34    ; Capture/compare register 1
.equ TIM_CCR2,             0x38    ; Capture/compare register 2
.equ TIM_CCR3,             0x3C    ; Capture/compare register 3
.equ TIM_CCR4,             0x40    ; Capture/compare register 4
.equ TIM_BDTR,             0x44    ; Break and dead-time register

; GPIO Peripheral Base Address (STM32F4xx example)
.equ GPIOA_BASE,           0x40020000
.equ GPIOB_BASE,           0x40020400
.equ GPIOC_BASE,           0x40020800
.equ GPIOD_BASE,           0x40020C00

; GPIO Register Offsets
.equ GPIO_MODER,           0x00    ; Mode register
.equ GPIO_OTYPER,          0x04    ; Output type register
.equ GPIO_OSPEEDR,         0x08    ; Output speed register
.equ GPIO_PUPDR,           0x0C    ; Pull-up/pull-down register
.equ GPIO_IDR,             0x10    ; Input data register
.equ GPIO_ODR,             0x14    ; Output data register
.equ GPIO_BSRR,            0x18    ; Bit set/reset register
.equ GPIO_AFRL,            0x20    ; Alternate function low register
.equ GPIO_AFRH,            0x24    ; Alternate function high register

; RCC Peripheral Base Address (STM32F4xx example)
.equ RCC_BASE,             0x40023800
.equ RCC_AHB1ENR,          0x30    ; AHB1 peripheral clock enable register
.equ RCC_APB1ENR,          0x40    ; APB1 peripheral clock enable register
.equ RCC_APB2ENR,          0x44    ; APB2 peripheral clock enable register

; Motor Constants
.equ MAX_MOTORS,           8       ; Maximum number of supported motors
.equ PWM_FREQUENCY,        400     ; PWM frequency in Hz
.equ PWM_PERIOD,           2500    ; PWM period in timer ticks (depends on timer clock)
.equ MIN_PULSE_WIDTH,      1000    ; Minimum pulse width in microseconds
.equ MAX_PULSE_WIDTH,      2000    ; Maximum pulse width in microseconds
.equ MOTOR_ARM_DELAY,      5000    ; Delay in milliseconds for arming motors

; Data section
.section .data
.align 4
motor_values:
    .space MAX_MOTORS * 4  ; Array to store motor values (0-1000)
motor_pins:
    .space MAX_MOTORS * 4  ; Array to store motor pin configurations
motor_count:
    .word 0                ; Number of configured motors

; Code section
.section .text
.align 4
.global motor_init
.global motor_configure
.global motor_set
.global motor_set_all
.global motor_arm
.global motor_disarm
.global motor_emergency_stop

;
; Initialize the motor controller
; Parameters:
;   r0 - Number of motors to control (1-8)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
motor_init:
    push {r4-r7, lr}
    
    ; Validate motor count
    cmp r0, #0
    ble motor_init_invalid_count
    cmp r0, #MAX_MOTORS
    bgt motor_init_invalid_count
    
    ; Store motor count
    ldr r1, =motor_count
    str r0, [r1]
    
    ; Initialize motor values to zero
    ldr r1, =motor_values
    mov r2, #0
    mov r3, r0
    
motor_init_clear_loop:
    str r2, [r1], #4
    subs r3, r3, #1
    bne motor_init_clear_loop
    
    ; Enable peripheral clocks
    bl enable_peripheral_clocks
    
    ; Configure timer for PWM generation
    bl configure_timer_pwm
    
    ; Success
    mov r0, #0
    pop {r4-r7, pc}
    
motor_init_invalid_count:
    ; Invalid motor count
    mov r0, #1
    pop {r4-r7, pc}

;
; Configure a motor
; Parameters:
;   r0 - Motor index (0 to motor_count-1)
;   r1 - GPIO port base address
;   r2 - GPIO pin number
;   r3 - Timer channel (1-4)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
motor_configure:
    push {r4-r7, lr}
    
    ; Save parameters
    mov r4, r0      ; Motor index
    mov r5, r1      ; GPIO port
    mov r6, r2      ; GPIO pin
    mov r7, r3      ; Timer channel
    
    ; Validate motor index
    ldr r0, =motor_count
    ldr r0, [r0]
    cmp r4, r0
    bge motor_configure_invalid_index
    
    ; Store motor pin configuration
    ldr r0, =motor_pins
    mov r1, r4
    lsl r1, r1, #2  ; Multiply by 4 (size of each entry)
    add r0, r0, r1
    
    ; Pack GPIO port, pin, and timer channel into a single word
    lsl r1, r5, #16
    orr r1, r1, r6
    orr r1, r1, r7, lsl #8
    str r1, [r0]
    
    ; Configure GPIO pin for alternate function
    mov r0, r5      ; GPIO port
    mov r1, r6      ; GPIO pin
    bl configure_gpio_af
    
    ; Success
    mov r0, #0
    pop {r4-r7, pc}
    
motor_configure_invalid_index:
    ; Invalid motor index
    mov r0, #1
    pop {r4-r7, pc}

;
; Set motor speed
; Parameters:
;   r0 - Motor index (0 to motor_count-1)
;   r1 - Motor value (0-1000, where 0 is off and 1000 is full throttle)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
motor_set:
    push {r4-r5, lr}
    
    ; Save parameters
    mov r4, r0      ; Motor index
    mov r5, r1      ; Motor value
    
    ; Validate motor index
    ldr r0, =motor_count
    ldr r0, [r0]
    cmp r4, r0
    bge motor_set_invalid_index
    
    ; Validate motor value
    cmp r5, #0
    blt motor_set_invalid_value
    cmp r5, #1000
    bgt motor_set_invalid_value
    
    ; Store motor value
    ldr r0, =motor_values
    mov r1, r4
    lsl r1, r1, #2  ; Multiply by 4 (size of each entry)
    add r0, r0, r1
    str r5, [r0]
    
    ; Get motor pin configuration
    ldr r0, =motor_pins
    mov r1, r4
    lsl r1, r1, #2  ; Multiply by 4 (size of each entry)
    add r0, r0, r1
    ldr r0, [r0]
    
    ; Extract timer channel
    lsr r1, r0, #8
    and r1, r1, #0xFF
    
    ; Convert motor value to pulse width
    mov r0, #MAX_PULSE_WIDTH - MIN_PULSE_WIDTH
    mul r0, r5, r0
    mov r2, #1000
    udiv r0, r0, r2
    add r0, r0, #MIN_PULSE_WIDTH
    
    ; Convert pulse width to timer ticks
    mov r2, #PWM_PERIOD
    mul r0, r0, r2
    mov r3, #20000  ; 20ms period (50Hz)
    udiv r0, r0, r3
    
    ; Set PWM duty cycle
    bl set_pwm_duty_cycle
    
    ; Success
    mov r0, #0
    pop {r4-r5, pc}
    
motor_set_invalid_index:
    ; Invalid motor index
    mov r0, #1
    pop {r4-r5, pc}
    
motor_set_invalid_value:
    ; Invalid motor value
    mov r0, #2
    pop {r4-r5, pc}

;
; Set all motor speeds
; Parameters:
;   r0 - Pointer to array of motor values (0-1000)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
motor_set_all:
    push {r4-r6, lr}
    
    ; Save parameter
    mov r4, r0      ; Pointer to motor values
    
    ; Get motor count
    ldr r5, =motor_count
    ldr r5, [r5]
    
    ; Set each motor
    mov r6, #0      ; Motor index
    
motor_set_all_loop:
    ldr r1, [r4], #4  ; Load motor value and increment pointer
    mov r0, r6      ; Motor index
    bl motor_set
    
    add r6, r6, #1
    cmp r6, r5
    blt motor_set_all_loop
    
    ; Success
    mov r0, #0
    pop {r4-r6, pc}

;
; Arm motors (send minimum throttle signal)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
motor_arm:
    push {r4-r5, lr}
    
    ; Get motor count
    ldr r4, =motor_count
    ldr r4, [r4]
    
    ; Set all motors to minimum throttle
    mov r5, #0      ; Motor index
    
motor_arm_loop:
    mov r0, r5      ; Motor index
    mov r1, #0      ; Minimum throttle
    bl motor_set
    
    add r5, r5, #1
    cmp r5, r4
    blt motor_arm_loop
    
    ; Delay to allow ESCs to arm
    mov r0, #MOTOR_ARM_DELAY
    bl delay_ms
    
    ; Success
    mov r0, #0
    pop {r4-r5, pc}

;
; Disarm motors (send zero throttle signal)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
motor_disarm:
    push {r4-r5, lr}
    
    ; Get motor count
    ldr r4, =motor_count
    ldr r4, [r4]
    
    ; Set all motors to zero throttle
    mov r5, #0      ; Motor index
    
motor_disarm_loop:
    mov r0, r5      ; Motor index
    mov r1, #0      ; Zero throttle
    bl motor_set
    
    add r5, r5, #1
    cmp r5, r4
    blt motor_disarm_loop
    
    ; Success
    mov r0, #0
    pop {r4-r5, pc}

;
; Emergency stop (immediately stop all motors)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
motor_emergency_stop:
    push {lr}
    
    ; Disarm motors
    bl motor_disarm
    
    ; Disable PWM output
    bl disable_pwm_output
    
    ; Success
    mov r0, #0
    pop {lr}
    bx lr

;
; Helper functions
;

; Enable peripheral clocks for timers and GPIO
enable_peripheral_clocks:
    push {lr}
    
    ; Enable GPIO clocks (GPIOA-D)
    ldr r0, =RCC_BASE
    ldr r1, [r0, #RCC_AHB1ENR]
    orr r1, r1, #0x0F  ; Enable GPIOA-D
    str r1, [r0, #RCC_AHB1ENR]
    
    ; Enable timer clocks (TIM1-4)
    ldr r1, [r0, #RCC_APB2ENR]
    orr r1, r1, #0x01  ; Enable TIM1
    str r1, [r0, #RCC_APB2ENR]
    
    ldr r1, [r0, #RCC_APB1ENR]
    orr r1, r1, #0x07  ; Enable TIM2-4
    str r1, [r0, #RCC_APB1ENR]
    
    pop {lr}
    bx lr

; Configure timer for PWM generation
configure_timer_pwm:
    push {r4-r5, lr}
    
    ; Configure TIM1 for PWM generation
    ldr r4, =TIM1_BASE
    
    ; Set prescaler to achieve desired frequency
    mov r0, #84      ; 84MHz / 84 = 1MHz timer clock
    str r0, [r4, #TIM_PSC]
    
    ; Set auto-reload value for 50Hz PWM (1MHz / 20000 = 50Hz)
    mov r0, #20000
    str r0, [r4, #TIM_ARR]
    
    ; Configure capture/compare mode register 1 (channels 1 & 2)
    ; PWM mode 1, preload enable
    ldr r0, =0x6868
    str r0, [r4, #TIM_CCMR1]
    
    ; Configure capture/compare mode register 2 (channels 3 & 4)
    ; PWM mode 1, preload enable
    ldr r0, =0x6868
    str r0, [r4, #TIM_CCMR2]
    
    ; Enable capture/compare outputs
    ldr r0, =0x1111
    str r0, [r4, #TIM_CCER]
    
    ; Enable main output (required for TIM1)
    ldr r0, =0x8000
    str r0, [r4, #TIM_BDTR]
    
    ; Enable counter
    ldr r0, [r4, #TIM_CR1]
    orr r0, r0, #0x01
    str r0, [r4, #TIM_CR1]
    
    ; Repeat for TIM2-4 if needed
    ; ...
    
    pop {r4-r5, pc}

; Configure GPIO pin for alternate function
; r0 - GPIO port base address
; r1 - GPIO pin number
configure_gpio_af:
    push {r4-r5, lr}
    
    mov r4, r0      ; GPIO port
    mov r5, r1      ; GPIO pin
    
    ; Set pin mode to alternate function (0x02)
    ldr r0, [r4, #GPIO_MODER]
    bic r0, r0, #(0x03 << (r5 * 2))
    orr r0, r0, #(0x02 << (r5 * 2))
    str r0, [r4, #GPIO_MODER]
    
    ; Set output type to push-pull (0x00)
    ldr r0, [r4, #GPIO_OTYPER]
    bic r0, r0, #(0x01 << r5)
    str r0, [r4, #GPIO_OTYPER]
    
    ; Set output speed to high (0x02)
    ldr r0, [r4, #GPIO_OSPEEDR]
    bic r0, r0, #(0x03 << (r5 * 2))
    orr r0, r0, #(0x02 << (r5 * 2))
    str r0, [r4, #GPIO_OSPEEDR]
    
    ; Set pull-up/pull-down to none (0x00)
    ldr r0, [r4, #GPIO_PUPDR]
    bic r0, r0, #(0x03 << (r5 * 2))
    str r0, [r4, #GPIO_PUPDR]
    
    ; Set alternate function (AF1 for TIM1/TIM2)
    cmp r5, #7
    ble configure_gpio_af_low
    
    ; High register (pins 8-15)
    sub r5, r5, #8
    ldr r0, [r4, #GPIO_AFRH]
    bic r0, r0, #(0x0F << (r5 * 4))
    orr r0, r0, #(0x01 << (r5 * 4))
    str r0, [r4, #GPIO_AFRH]
    b configure_gpio_af_done
    
configure_gpio_af_low:
    ; Low register (pins 0-7)
    ldr r0, [r4, #GPIO_AFRL]
    bic r0, r0, #(0x0F << (r5 * 4))
    orr r0, r0, #(0x01 << (r5 * 4))
    str r0, [r4, #GPIO_AFRL]
    
configure_gpio_af_done:
    pop {r4-r5, pc}

; Set PWM duty cycle
; r0 - Duty cycle value
; r1 - Timer channel (1-4)
set_pwm_duty_cycle:
    push {r4-r5, lr}
    
    mov r4, r0      ; Duty cycle
    mov r5, r1      ; Timer channel
    
    ; Select appropriate capture/compare register based on channel
    ldr r0, =TIM1_BASE
    
    cmp r5, #1
    beq set_pwm_duty_cycle_ch1
    cmp r5, #2
    beq set_pwm_duty_cycle_ch2
    cmp r5, #3
    beq set_pwm_duty_cycle_ch3
    cmp r5, #4
    beq set_pwm_duty_cycle_ch4
    
    ; Invalid channel
    b set_pwm_duty_cycle_done
    
set_pwm_duty_cycle_ch1:
    str r4, [r0, #TIM_CCR1]
    b set_pwm_duty_cycle_done
    
set_pwm_duty_cycle_ch2:
    str r4, [r0, #TIM_CCR2]
    b set_pwm_duty_cycle_done
    
set_pwm_duty_cycle_ch3:
    str r4, [r0, #TIM_CCR3]
    b set_pwm_duty_cycle_done
    
set_pwm_duty_cycle_ch4:
    str r4, [r0, #TIM_CCR4]
    
set_pwm_duty_cycle_done:
    pop {r4-r5, pc}

; Disable PWM output
disable_pwm_output:
    push {lr}
    
    ; Disable capture/compare outputs
    ldr r0, =TIM1_BASE
    mov r1, #0
    str r1, [r0, #TIM_CCER]
    
    ; Disable main output (for TIM1)
    ldr r1, [r0, #TIM_BDTR]
    bic r1, r1, #0x8000
    str r1, [r0, #TIM_BDTR]
    
    ; Repeat for TIM2-4 if needed
    ; ...
    
    pop {lr}
    bx lr

; Delay for specified milliseconds
; r0 - Delay in milliseconds
delay_ms:
    push {r4, lr}
    
    mov r4, r0
    
    ; Get current time
    bl get_time_ms
    mov r1, r0
    
delay_ms_loop:
    ; Get current time
    bl get_time_ms
    
    ; Check if enough time has elapsed
    sub r0, r0, r1
    cmp r0, r4
    blt delay_ms_loop
    
    pop {r4, pc}

; Get current time in milliseconds
; Returns: r0 - Current time in milliseconds
get_time_ms:
    ; Implementation depends on the specific microcontroller
    bx lr

.end