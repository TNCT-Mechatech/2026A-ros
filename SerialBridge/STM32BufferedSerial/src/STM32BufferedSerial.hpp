/**
 * @file STM32BufferedSerial.hpp
 * @brief STM32 HAL環境で割り込み駆動のバッファ付きシリアル通信を提供するクラス
 * @author shoyo
 * @date 2025/10/14
 */

#ifndef STM32_BUFFERED_SERIAL_HPP
#define STM32_BUFFERED_SERIAL_HPP

#include "main.h"
#include "stm32f4xx_hal.h"
#include <cstdint>

class STM32BufferedSerial {
public:
    explicit STM32BufferedSerial(UART_HandleTypeDef* huart, uint16_t bufSize = 256);

    // 通信開始（受信割り込み開始）
    void begin();

    // データ送受信
    int read();                        // 1バイト読み込み（データなし: -1）
    int write(uint8_t data);           // 1バイト送信
    int write(const uint8_t* data, uint16_t len); // 複数バイト送信

    // バッファ状態
    bool available() const;            // 受信データがあるか
    int readable_len() const;        // 受信データ長
    void flushRx();                    // RXバッファをクリア
    void flushTx();                    // TXバッファをクリア

    // 割り込みコールバック
    void handleRxComplete();           // HAL_UART_RxCpltCallback()から呼ばれる
    void handleTxComplete();           // HAL_UART_TxCpltCallback()から呼ばれる

    UART_HandleTypeDef* getHandle() const { return _huart; }

    // HALコールバック登録用
    static void registerInstance(UART_HandleTypeDef* huart, STM32BufferedSerial* instance);
    static STM32BufferedSerial* fromHandle(UART_HandleTypeDef* huart);

private:
    UART_HandleTypeDef* _huart;        // UARTハンドル
    uint8_t* _rxBuf;                   // 受信バッファ
    uint8_t* _txBuf;                   // 送信バッファ
    uint16_t _rxSize;                  // バッファサイズ
    uint16_t _txSize;
    volatile uint16_t _rxHead, _rxTail;
    volatile uint16_t _txHead, _txTail;
    uint8_t _rxTmp;                    // 割り込み受信用1byte

    static constexpr int MAX_UARTS = 6;
    static STM32BufferedSerial* instance_table_[MAX_UARTS];

    // 内部処理
    void _startRxInterrupt();          // 受信割り込み開始
    void _startTxInterrupt();          // 送信割り込み開始

    void push(uint8_t c);
    int pop();

};

#endif
