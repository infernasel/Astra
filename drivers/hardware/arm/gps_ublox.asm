;
; ASTRA Hardware Driver: u-blox GPS Module
; Architecture: ARM Cortex-M4
;
; This driver provides low-level access to u-blox GPS modules (NEO-M8, NEO-7, etc.)
; commonly used in drone and spacecraft applications for precise positioning.
;

.syntax unified
.cpu cortex-m4
.thumb

; UBX Protocol Constants
.equ UBX_SYNC_CHAR_1,      0xB5    ; First sync character (μ)
.equ UBX_SYNC_CHAR_2,      0x62    ; Second sync character (b)

; UBX Message Classes
.equ UBX_CLASS_NAV,        0x01    ; Navigation Results
.equ UBX_CLASS_RXM,        0x02    ; Receiver Manager Messages
.equ UBX_CLASS_INF,        0x04    ; Information Messages
.equ UBX_CLASS_ACK,        0x05    ; Acknowledgement Messages
.equ UBX_CLASS_CFG,        0x06    ; Configuration Input Messages
.equ UBX_CLASS_MON,        0x0A    ; Monitoring Messages
.equ UBX_CLASS_AID,        0x0B    ; AssistNow Aiding Messages
.equ UBX_CLASS_TIM,        0x0D    ; Timing Messages
.equ UBX_CLASS_ESF,        0x10    ; External Sensor Fusion Messages

; UBX Message IDs (NAV class)
.equ UBX_NAV_POSLLH,       0x02    ; Geodetic Position Solution
.equ UBX_NAV_STATUS,       0x03    ; Receiver Navigation Status
.equ UBX_NAV_SOL,          0x06    ; Navigation Solution Information
.equ UBX_NAV_VELNED,       0x12    ; Velocity Solution in NED
.equ UBX_NAV_TIMEUTC,      0x21    ; UTC Time Solution
.equ UBX_NAV_SVINFO,       0x30    ; Space Vehicle Information
.equ UBX_NAV_PVT,          0x07    ; Position, Velocity and Time Solution

; UBX Message IDs (CFG class)
.equ UBX_CFG_PRT,          0x00    ; Port Configuration
.equ UBX_CFG_MSG,          0x01    ; Message Configuration
.equ UBX_CFG_RATE,         0x08    ; Navigation/Measurement Rate Settings
.equ UBX_CFG_CFG,          0x09    ; Clear, Save and Load Configurations
.equ UBX_CFG_NAV5,         0x24    ; Navigation Engine Settings

; UBX Message IDs (ACK class)
.equ UBX_ACK_ACK,          0x01    ; Message Acknowledged
.equ UBX_ACK_NAK,          0x00    ; Message Not Acknowledged

; UART Peripheral Base Address (STM32F4xx example)
.equ USART2_BASE,          0x40004400
.equ USART_SR,             0x00    ; Status register
.equ USART_DR,             0x04    ; Data register
.equ USART_BRR,            0x08    ; Baud rate register

; UART Flags
.equ USART_SR_RXNE,        0x0020  ; Read data register not empty
.equ USART_SR_TXE,         0x0080  ; Transmit data register empty

; Buffer sizes
.equ RX_BUFFER_SIZE,       256     ; Receive buffer size
.equ TX_BUFFER_SIZE,       256     ; Transmit buffer size
.equ UBX_MAX_SIZE,         92      ; Maximum UBX message size

; Data buffers
.section .bss
.align 4
gps_rx_buffer:
    .space RX_BUFFER_SIZE
gps_tx_buffer:
    .space TX_BUFFER_SIZE
ubx_message_buffer:
    .space UBX_MAX_SIZE
gps_position:              ; Geodetic position (lat, lon, height)
    .space 12              ; 3 x 32-bit values
gps_velocity:              ; NED velocity (north, east, down)
    .space 12              ; 3 x 32-bit values
gps_time:                  ; UTC time
    .space 20              ; Year, month, day, hour, minute, second, valid flags

; Code section
.section .text
.align 4
.global gps_init
.global gps_configure
.global gps_read_position
.global gps_read_velocity
.global gps_read_time
.global gps_process_data

;
; Initialize the GPS module
; Parameters:
;   r0 - UART baud rate (typically 9600 or 38400)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
gps_init:
    push {r4, lr}
    
    ; Save baud rate
    mov r4, r0
    
    ; Initialize UART (platform-specific)
    bl uart_init
    
    ; Configure GPS module with default settings
    mov r0, #1      ; Enable NAV-PVT messages (1Hz)
    bl gps_configure
    
    ; Success
    mov r0, #0
    pop {r4, pc}

;
; Configure the GPS module
; Parameters:
;   r0 - Navigation rate in Hz (1-10)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
gps_configure:
    push {r4-r7, lr}
    
    ; Save navigation rate
    mov r4, r0
    
    ; Calculate message rate in milliseconds
    mov r0, #1000
    udiv r5, r0, r4
    
    ; Prepare CFG-RATE message
    adr r0, ubx_message_buffer
    
    ; Header
    mov r1, #UBX_SYNC_CHAR_1
    strb r1, [r0], #1
    mov r1, #UBX_SYNC_CHAR_2
    strb r1, [r0], #1
    mov r1, #UBX_CLASS_CFG
    strb r1, [r0], #1
    mov r1, #UBX_CFG_RATE
    strb r1, [r0], #1
    
    ; Payload length (6 bytes)
    mov r1, #6
    strb r1, [r0], #1
    mov r1, #0
    strb r1, [r0], #1
    
    ; Payload - Measurement rate in ms
    strh r5, [r0], #2
    
    ; Payload - Navigation rate (1 = every measurement)
    mov r1, #1
    strh r1, [r0], #2
    
    ; Payload - Time reference (0 = UTC)
    mov r1, #0
    strh r1, [r0], #2
    
    ; Calculate checksum
    adr r1, ubx_message_buffer
    add r1, #2      ; Skip sync chars
    mov r2, #6      ; Payload length + 4 (class, id, length)
    bl ubx_checksum
    
    ; Store checksum
    strb r6, [r0], #1  ; CK_A
    strb r7, [r0], #1  ; CK_B
    
    ; Send message
    adr r0, ubx_message_buffer
    mov r1, #12     ; Total message length
    bl uart_send
    
    ; Wait for acknowledgement (with timeout)
    mov r0, #100    ; 100ms timeout
    bl wait_for_ack
    
    ; Configure NAV-PVT message rate
    adr r0, ubx_message_buffer
    
    ; Header
    mov r1, #UBX_SYNC_CHAR_1
    strb r1, [r0], #1
    mov r1, #UBX_SYNC_CHAR_2
    strb r1, [r0], #1
    mov r1, #UBX_CLASS_CFG
    strb r1, [r0], #1
    mov r1, #UBX_CFG_MSG
    strb r1, [r0], #1
    
    ; Payload length (3 bytes)
    mov r1, #3
    strb r1, [r0], #1
    mov r1, #0
    strb r1, [r0], #1
    
    ; Payload - Message class and ID
    mov r1, #UBX_CLASS_NAV
    strb r1, [r0], #1
    mov r1, #UBX_NAV_PVT
    strb r1, [r0], #1
    
    ; Payload - Rate (1 = every navigation solution)
    mov r1, #1
    strb r1, [r0], #1
    
    ; Calculate checksum
    adr r1, ubx_message_buffer
    add r1, #2      ; Skip sync chars
    mov r2, #7      ; Payload length + 4 (class, id, length)
    bl ubx_checksum
    
    ; Store checksum
    strb r6, [r0], #1  ; CK_A
    strb r7, [r0], #1  ; CK_B
    
    ; Send message
    adr r0, ubx_message_buffer
    mov r1, #11     ; Total message length
    bl uart_send
    
    ; Wait for acknowledgement (with timeout)
    mov r0, #100    ; 100ms timeout
    bl wait_for_ack
    
    ; Success
    mov r0, #0
    pop {r4-r7, pc}

;
; Read the current GPS position
; Parameters:
;   r0 - Pointer to buffer to store position data (12 bytes: lat, lon, height as 32-bit values)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
gps_read_position:
    push {r4, lr}
    
    ; Copy position data to provided buffer
    mov r4, r0
    adr r1, gps_position
    ldm r1, {r0-r2}
    stm r4, {r0-r2}
    
    ; Success
    mov r0, #0
    pop {r4, pc}

;
; Read the current GPS velocity
; Parameters:
;   r0 - Pointer to buffer to store velocity data (12 bytes: north, east, down as 32-bit values)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
gps_read_velocity:
    push {r4, lr}
    
    ; Copy velocity data to provided buffer
    mov r4, r0
    adr r1, gps_velocity
    ldm r1, {r0-r2}
    stm r4, {r0-r2}
    
    ; Success
    mov r0, #0
    pop {r4, pc}

;
; Read the current GPS time
; Parameters:
;   r0 - Pointer to buffer to store time data (20 bytes: year, month, day, hour, minute, second, valid flags)
; Returns:
;   r0 - 0 on success, non-zero on failure
;
gps_read_time:
    push {r4, lr}
    
    ; Copy time data to provided buffer
    mov r4, r0
    adr r1, gps_time
    ldm r1, {r0-r3}
    stm r4!, {r0-r3}
    ldm r1, {r0}
    str r0, [r4]
    
    ; Success
    mov r0, #0
    pop {r4, pc}

;
; Process received GPS data
; This function should be called regularly to process incoming UART data
; Returns:
;   r0 - 1 if new position data available, 0 otherwise
;
gps_process_data:
    push {r4-r7, lr}
    
    ; Check if data is available
    bl uart_data_available
    cmp r0, #0
    beq gps_process_no_data
    
    ; Read data from UART
    bl uart_read_byte
    
    ; Check for UBX sync char 1
    cmp r0, #UBX_SYNC_CHAR_1
    bne gps_process_continue
    
    ; Read next byte
    bl uart_data_available
    cmp r0, #0
    beq gps_process_no_data
    
    bl uart_read_byte
    
    ; Check for UBX sync char 2
    cmp r0, #UBX_SYNC_CHAR_2
    bne gps_process_continue
    
    ; Start of UBX message detected
    adr r4, ubx_message_buffer
    
    ; Store sync chars
    mov r0, #UBX_SYNC_CHAR_1
    strb r0, [r4], #1
    mov r0, #UBX_SYNC_CHAR_2
    strb r0, [r4], #1
    
    ; Read message class
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq gps_process_timeout
    
    mov r5, r0      ; Save message class
    strb r0, [r4], #1
    
    ; Read message ID
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq gps_process_timeout
    
    mov r6, r0      ; Save message ID
    strb r0, [r4], #1
    
    ; Read payload length (2 bytes, little endian)
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq gps_process_timeout
    
    mov r7, r0      ; Low byte of length
    strb r0, [r4], #1
    
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq gps_process_timeout
    
    mov r1, r0      ; High byte of length
    strb r0, [r4], #1
    
    lsl r1, r1, #8
    orr r7, r7, r1  ; r7 now contains payload length
    
    ; Check if payload length is reasonable
    cmp r7, #UBX_MAX_SIZE
    bgt gps_process_continue
    
    ; Read payload
    mov r0, r7      ; Payload length
    mov r1, r4      ; Buffer pointer
    bl uart_read_bytes_timeout
    cmp r0, #-1
    beq gps_process_timeout
    
    add r4, r4, r7  ; Update buffer pointer
    
    ; Read checksum (2 bytes)
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq gps_process_timeout
    
    mov r1, r0      ; CK_A
    strb r0, [r4], #1
    
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq gps_process_timeout
    
    mov r2, r0      ; CK_B
    strb r0, [r4], #1
    
    ; Verify checksum
    adr r0, ubx_message_buffer
    add r0, #2      ; Skip sync chars
    mov r3, r7
    add r3, #4      ; Add header length
    bl ubx_checksum
    
    ; Compare calculated checksum with received checksum
    cmp r6, r1
    bne gps_process_continue
    cmp r7, r2
    bne gps_process_continue
    
    ; Process message based on class and ID
    cmp r5, #UBX_CLASS_NAV
    bne gps_process_not_nav
    
    cmp r6, #UBX_NAV_PVT
    bne gps_process_not_pvt
    
    ; Process NAV-PVT message
    adr r0, ubx_message_buffer
    add r0, #6      ; Skip header
    bl process_nav_pvt
    
    ; New position data available
    mov r0, #1
    pop {r4-r7, pc}
    
gps_process_not_pvt:
    ; Other NAV messages
    b gps_process_continue
    
gps_process_not_nav:
    ; Process other message classes if needed
    b gps_process_continue
    
gps_process_timeout:
    ; Timeout occurred while reading message
    b gps_process_continue
    
gps_process_continue:
    ; Continue processing
    mov r0, #0
    pop {r4-r7, pc}
    
gps_process_no_data:
    ; No data available
    mov r0, #0
    pop {r4-r7, pc}

;
; Process NAV-PVT message
; Parameters:
;   r0 - Pointer to NAV-PVT payload
; Returns:
;   None
;
process_nav_pvt:
    push {r4-r5, lr}
    
    mov r4, r0      ; Save payload pointer
    
    ; Extract position data (lat, lon, height)
    ldr r0, [r4, #28]  ; Latitude (deg * 1e-7)
    ldr r1, [r4, #24]  ; Longitude (deg * 1e-7)
    ldr r2, [r4, #32]  ; Height above ellipsoid (mm)
    
    ; Store position data
    adr r5, gps_position
    stm r5, {r0-r2}
    
    ; Extract velocity data (north, east, down)
    ldr r0, [r4, #48]  ; North velocity (mm/s)
    ldr r1, [r4, #52]  ; East velocity (mm/s)
    ldr r2, [r4, #56]  ; Down velocity (mm/s)
    
    ; Store velocity data
    adr r5, gps_velocity
    stm r5, {r0-r2}
    
    ; Extract time data
    ldrh r0, [r4, #4]   ; Year
    ldrb r1, [r4, #6]   ; Month
    ldrb r2, [r4, #7]   ; Day
    ldrb r3, [r4, #8]   ; Hour
    ldrb r5, [r4, #9]   ; Minute
    ldrb r12, [r4, #10] ; Second
    ldrb r14, [r4, #11] ; Valid flags
    
    ; Store time data
    adr r5, gps_time
    str r0, [r5, #0]
    str r1, [r5, #4]
    str r2, [r5, #8]
    str r3, [r5, #12]
    str r12, [r5, #16]
    str r14, [r5, #20]
    
    pop {r4-r5, pc}

;
; Calculate UBX message checksum
; Parameters:
;   r0 - Pointer to message (starting after sync chars)
;   r1 - Length of message to checksum
; Returns:
;   r6 - CK_A
;   r7 - CK_B
;
ubx_checksum:
    push {r4-r5, lr}
    
    mov r4, r0      ; Save message pointer
    mov r5, r1      ; Save length
    
    mov r6, #0      ; CK_A
    mov r7, #0      ; CK_B
    
ubx_checksum_loop:
    ldrb r0, [r4], #1
    add r6, r6, r0
    add r7, r7, r6
    
    ; Keep checksums in range 0-255
    and r6, r6, #0xFF
    and r7, r7, #0xFF
    
    subs r5, r5, #1
    bne ubx_checksum_loop
    
    pop {r4-r5, pc}

;
; Wait for acknowledgement message
; Parameters:
;   r0 - Timeout in milliseconds
; Returns:
;   r0 - 0 on success (ACK received), non-zero on failure (NAK or timeout)
;
wait_for_ack:
    push {r4-r7, lr}
    
    mov r4, r0      ; Save timeout
    
    ; Start timeout
    bl get_time_ms
    mov r5, r0      ; Save start time
    
wait_for_ack_loop:
    ; Check if data is available
    bl uart_data_available
    cmp r0, #0
    beq wait_for_ack_check_timeout
    
    ; Read data from UART
    bl uart_read_byte
    
    ; Check for UBX sync char 1
    cmp r0, #UBX_SYNC_CHAR_1
    bne wait_for_ack_loop
    
    ; Read next byte
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq wait_for_ack_timeout
    
    ; Check for UBX sync char 2
    cmp r0, #UBX_SYNC_CHAR_2
    bne wait_for_ack_loop
    
    ; Read message class
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq wait_for_ack_timeout
    
    ; Check for ACK class
    cmp r0, #UBX_CLASS_ACK
    bne wait_for_ack_loop
    
    ; Read message ID
    bl uart_read_byte_timeout
    cmp r0, #-1
    beq wait_for_ack_timeout
    
    ; Check for ACK-ACK or ACK-NAK
    cmp r0, #UBX_ACK_ACK
    beq wait_for_ack_success
    cmp r0, #UBX_ACK_NAK
    beq wait_for_ack_failure
    
    b wait_for_ack_loop
    
wait_for_ack_check_timeout:
    ; Check if timeout has elapsed
    bl get_time_ms
    sub r0, r0, r5
    cmp r0, r4
    blt wait_for_ack_loop
    
wait_for_ack_timeout:
    ; Timeout occurred
    mov r0, #2
    pop {r4-r7, pc}
    
wait_for_ack_success:
    ; ACK received
    mov r0, #0
    pop {r4-r7, pc}
    
wait_for_ack_failure:
    ; NAK received
    mov r0, #1
    pop {r4-r7, pc}

;
; UART and timing helper functions (platform-specific implementations)
; These would typically be implemented elsewhere and linked with this code
;

; Initialize UART
; r0 - Baud rate
; Returns: None
uart_init:
    ; Implementation depends on the specific microcontroller
    bx lr

; Send data over UART
; r0 - Pointer to data
; r1 - Number of bytes to send
; Returns: None
uart_send:
    ; Implementation depends on the specific microcontroller
    bx lr

; Check if data is available on UART
; Returns: r0 - Number of bytes available
uart_data_available:
    ; Implementation depends on the specific microcontroller
    bx lr

; Read a byte from UART
; Returns: r0 - Byte read
uart_read_byte:
    ; Implementation depends on the specific microcontroller
    bx lr

; Read a byte from UART with timeout
; Returns: r0 - Byte read or -1 on timeout
uart_read_byte_timeout:
    ; Implementation depends on the specific microcontroller
    bx lr

; Read multiple bytes from UART with timeout
; r0 - Number of bytes to read
; r1 - Pointer to buffer
; Returns: r0 - 0 on success, -1 on timeout
uart_read_bytes_timeout:
    ; Implementation depends on the specific microcontroller
    bx lr

; Get current time in milliseconds
; Returns: r0 - Current time in milliseconds
get_time_ms:
    ; Implementation depends on the specific microcontroller
    bx lr

.end