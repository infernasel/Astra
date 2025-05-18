;
; ASTRA Hardware Driver: MPU6050 Gyroscope/Accelerometer
; Architecture: ARM Cortex-M4
;
; This driver provides low-level access to the MPU6050 6-axis gyroscope and accelerometer
; commonly used in drone and spacecraft applications.
;

.syntax unified
.cpu cortex-m4
.thumb

; MPU6050 Register Addresses
.equ MPU6050_ADDR,         0x68    ; I2C address (AD0 low)
.equ REG_PWR_MGMT_1,       0x6B    ; Power Management 1
.equ REG_SMPLRT_DIV,       0x19    ; Sample Rate Divider
.equ REG_CONFIG,           0x1A    ; Configuration
.equ REG_GYRO_CONFIG,      0x1B    ; Gyroscope Configuration
.equ REG_ACCEL_CONFIG,     0x1C    ; Accelerometer Configuration
.equ REG_FIFO_EN,          0x23    ; FIFO Enable
.equ REG_INT_ENABLE,       0x38    ; Interrupt Enable
.equ REG_ACCEL_XOUT_H,     0x3B    ; Accelerometer X-axis high byte
.equ REG_ACCEL_XOUT_L,     0x3C    ; Accelerometer X-axis low byte
.equ REG_ACCEL_YOUT_H,     0x3D    ; Accelerometer Y-axis high byte
.equ REG_ACCEL_YOUT_L,     0x3E    ; Accelerometer Y-axis low byte
.equ REG_ACCEL_ZOUT_H,     0x3F    ; Accelerometer Z-axis high byte
.equ REG_ACCEL_ZOUT_L,     0x40    ; Accelerometer Z-axis low byte
.equ REG_TEMP_OUT_H,       0x41    ; Temperature high byte
.equ REG_TEMP_OUT_L,       0x42    ; Temperature low byte
.equ REG_GYRO_XOUT_H,      0x43    ; Gyroscope X-axis high byte
.equ REG_GYRO_XOUT_L,      0x44    ; Gyroscope X-axis low byte
.equ REG_GYRO_YOUT_H,      0x45    ; Gyroscope Y-axis high byte
.equ REG_GYRO_YOUT_L,      0x46    ; Gyroscope Y-axis low byte
.equ REG_GYRO_ZOUT_H,      0x47    ; Gyroscope Z-axis high byte
.equ REG_GYRO_ZOUT_L,      0x48    ; Gyroscope Z-axis low byte
.equ REG_USER_CTRL,        0x6A    ; User Control
.equ REG_WHO_AM_I,         0x75    ; Device ID (should return 0x68)

; Configuration Values
.equ GYRO_FS_250,          0x00    ; ±250 °/s
.equ GYRO_FS_500,          0x08    ; ±500 °/s
.equ GYRO_FS_1000,         0x10    ; ±1000 °/s
.equ GYRO_FS_2000,         0x18    ; ±2000 °/s
.equ ACCEL_FS_2,           0x00    ; ±2g
.equ ACCEL_FS_4,           0x08    ; ±4g
.equ ACCEL_FS_8,           0x10    ; ±8g
.equ ACCEL_FS_16,          0x18    ; ±16g

; I2C Peripheral Base Address (STM32F4xx example)
.equ I2C1_BASE,            0x40005400
.equ I2C_CR1,              0x00    ; Control register 1
.equ I2C_CR2,              0x04    ; Control register 2
.equ I2C_DR,               0x10    ; Data register
.equ I2C_SR1,              0x14    ; Status register 1
.equ I2C_SR2,              0x18    ; Status register 2

; I2C Flags
.equ I2C_SR1_SB,           0x0001  ; Start bit
.equ I2C_SR1_ADDR,         0x0002  ; Address sent
.equ I2C_SR1_BTF,          0x0004  ; Byte transfer finished
.equ I2C_SR1_RXNE,         0x0040  ; Data register not empty
.equ I2C_SR1_TXE,          0x0080  ; Data register empty

; Data buffer for sensor readings (6 axes × 2 bytes each + 2 bytes temperature)
.section .bss
.align 4
mpu6050_data:
    .space 14

; Code section
.section .text
.align 4
.global mpu6050_init
.global mpu6050_read_accel
.global mpu6050_read_gyro
.global mpu6050_read_temp
.global mpu6050_read_all

;
; Initialize the MPU6050 sensor
; Parameters:
;   r0 - Gyroscope full scale range (GYRO_FS_250, GYRO_FS_500, GYRO_FS_1000, or GYRO_FS_2000)
;   r1 - Accelerometer full scale range (ACCEL_FS_2, ACCEL_FS_4, ACCEL_FS_8, or ACCEL_FS_16)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
mpu6050_init:
    push {r4-r5, lr}
    
    ; Save parameters
    mov r4, r0      ; Gyro config
    mov r5, r1      ; Accel config
    
    ; Wake up the device (clear sleep bit)
    mov r0, #MPU6050_ADDR
    mov r1, #REG_PWR_MGMT_1
    mov r2, #0x00
    bl i2c_write_byte
    cmp r0, #0
    bne mpu6050_init_fail
    
    ; Set sample rate divider to 0 (maximum rate)
    mov r0, #MPU6050_ADDR
    mov r1, #REG_SMPLRT_DIV
    mov r2, #0x00
    bl i2c_write_byte
    cmp r0, #0
    bne mpu6050_init_fail
    
    ; Set digital low pass filter to 44Hz (0x03)
    mov r0, #MPU6050_ADDR
    mov r1, #REG_CONFIG
    mov r2, #0x03
    bl i2c_write_byte
    cmp r0, #0
    bne mpu6050_init_fail
    
    ; Configure gyroscope range
    mov r0, #MPU6050_ADDR
    mov r1, #REG_GYRO_CONFIG
    mov r2, r4
    bl i2c_write_byte
    cmp r0, #0
    bne mpu6050_init_fail
    
    ; Configure accelerometer range
    mov r0, #MPU6050_ADDR
    mov r1, #REG_ACCEL_CONFIG
    mov r2, r5
    bl i2c_write_byte
    cmp r0, #0
    bne mpu6050_init_fail
    
    ; Verify device ID
    mov r0, #MPU6050_ADDR
    mov r1, #REG_WHO_AM_I
    bl i2c_read_byte
    cmp r0, #0
    bne mpu6050_init_fail
    
    ; Check if device ID matches expected value (0x68)
    cmp r1, #0x68
    bne mpu6050_init_fail
    
    ; Success
    mov r0, #0
    pop {r4-r5, pc}
    
mpu6050_init_fail:
    ; Return error code
    mov r0, #1
    pop {r4-r5, pc}

;
; Read accelerometer data from MPU6050
; Parameters:
;   r0 - Pointer to buffer to store accelerometer data (6 bytes: X, Y, Z as 16-bit values)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
mpu6050_read_accel:
    push {r4, lr}
    
    ; Save buffer pointer
    mov r4, r0
    
    ; Read 6 bytes starting from ACCEL_XOUT_H
    mov r0, #MPU6050_ADDR
    mov r1, #REG_ACCEL_XOUT_H
    mov r2, r4
    mov r3, #6
    bl i2c_read_bytes
    
    pop {r4, pc}

;
; Read gyroscope data from MPU6050
; Parameters:
;   r0 - Pointer to buffer to store gyroscope data (6 bytes: X, Y, Z as 16-bit values)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
mpu6050_read_gyro:
    push {r4, lr}
    
    ; Save buffer pointer
    mov r4, r0
    
    ; Read 6 bytes starting from GYRO_XOUT_H
    mov r0, #MPU6050_ADDR
    mov r1, #REG_GYRO_XOUT_H
    mov r2, r4
    mov r3, #6
    bl i2c_read_bytes
    
    pop {r4, pc}

;
; Read temperature data from MPU6050
; Parameters:
;   r0 - Pointer to buffer to store temperature data (2 bytes as 16-bit value)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
mpu6050_read_temp:
    push {r4, lr}
    
    ; Save buffer pointer
    mov r4, r0
    
    ; Read 2 bytes starting from TEMP_OUT_H
    mov r0, #MPU6050_ADDR
    mov r1, #REG_TEMP_OUT_H
    mov r2, r4
    mov r3, #2
    bl i2c_read_bytes
    
    pop {r4, pc}

;
; Read all sensor data from MPU6050 (accelerometer, gyroscope, and temperature)
; Parameters:
;   r0 - Pointer to buffer to store all data (14 bytes total)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
mpu6050_read_all:
    push {r4, lr}
    
    ; Save buffer pointer
    mov r4, r0
    
    ; Read 14 bytes starting from ACCEL_XOUT_H
    mov r0, #MPU6050_ADDR
    mov r1, #REG_ACCEL_XOUT_H
    mov r2, r4
    mov r3, #14
    bl i2c_read_bytes
    
    pop {r4, pc}

;
; I2C helper functions (platform-specific implementations)
; These would typically be implemented elsewhere and linked with this code
;

; Write a byte to an I2C device
; r0 - Device address
; r1 - Register address
; r2 - Data byte
; Returns: r0 - 0 on success, non-zero on failure
i2c_write_byte:
    ; Implementation depends on the specific microcontroller
    bx lr

; Read a byte from an I2C device
; r0 - Device address
; r1 - Register address
; Returns: r0 - 0 on success, non-zero on failure
;          r1 - Data byte read
i2c_read_byte:
    ; Implementation depends on the specific microcontroller
    bx lr

; Read multiple bytes from an I2C device
; r0 - Device address
; r1 - Starting register address
; r2 - Pointer to buffer to store data
; r3 - Number of bytes to read
; Returns: r0 - 0 on success, non-zero on failure
i2c_read_bytes:
    ; Implementation depends on the specific microcontroller
    bx lr

.end