#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <unistd.h> // for usleep

// SPI設定
#define SPI_CHANNEL 0   // CE0を使用
#define SPI_SPEED   1000000 // 1MHz

// 監視するチャンネルの範囲
#define START_CH    1   // CH1から読み取りを開始
#define END_CH      6   // CH6まで読み取りを行う

/**
 * @brief MCP3008からアナログ値を読み取る関数
 * @param channel 読み取るチャンネル番号 (0-7)
 * @return 読み取った10bitのデジタル値 (0-1023), エラー時は-1
 */
int read_mcp3008_channel(int channel) {
    unsigned char spi_data[3];
    int adc_value;

    if (channel < 0 || channel > 7) {
        return -1;
    }

    // 制御バイトの準備: スタートビット(1) + シングルエンドモード(1) + チャンネル指定
    spi_data[0] = 1;
    spi_data[1] = (8 + channel) << 4;
    spi_data[2] = 0;

    // SPI通信の実行
    if (wiringPiSPIDataRW(SPI_CHANNEL, spi_data, 3) == -1) {
        perror("SPI通信エラー");
        return -1;
    }

    // 10bitのデジタル値を結合
    // spi_data[1] の下2bitと spi_data[2] の8bitを結合
    adc_value = ((spi_data[1] & 0x03) << 8) + spi_data[2];

    return adc_value;
}

int main(void) {
    int adc_value;
    double voltage;
    const double VREF = 3.3; // 基準電圧 (ラズパイの3.3V)
    const int MAX_ADC = 1023; // 10bit分解能の最大値
    int ch;

    // wiringPiのセットアップ
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "wiringPiのセットアップに失敗しました。\n");
        return 1;
    }

    // SPIインターフェースの初期化
    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
        fprintf(stderr, "SPIのセットアップに失敗しました。\n");
        return 1;
    }

    printf("複数チャンネル (CH%d - CH%d) の読み取りを開始します。\n", START_CH, END_CH);
    printf("----------------------------------------\n");

    while (1) {
        printf("--- New Reading Cycle ---\n");
        
        // 設定された範囲のチャンネルを順番に読み取る
        for (ch = START_CH; ch <= END_CH; ch++) {
            
            // 指定されたチャンネルからデータを読み取る
            adc_value = read_mcp3008_channel(ch);

            if (adc_value != -1) {
                // デジタル値を電圧に変換
                voltage = ((double)adc_value / MAX_ADC) * VREF;

                printf("CH%d: ADC Value: %4d (0-1023) | Voltage: %.3f V\n", ch, adc_value, voltage);
            } else {
                fprintf(stderr, "CH%d の読み取りに失敗しました。\n", ch);
            }
        }
        
        // 500ms待機 (全チャンネルの読み取り後に待機)
        usleep(500000); 
    }
    return 0;
}
